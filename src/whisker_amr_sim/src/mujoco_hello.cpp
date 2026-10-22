// mujoco_hello.cpp — Week-1 sanity check for the whisker-AMR project.
//
// What it does (no ROS yet, that's week 2):
//   1. Loads the MJCF world and checks the model compiled.
//   2. Drives the robot in an arc for 5 s of sim time using the wheel velocity servos.
//   3. Prints ground-truth pose and wheel-encoder readings at 1 Hz.
//   4. Casts one ray forward from the LiDAR site twice: once seeing ALL geoms and
//      once excluding geom group 3 (glass). This is the core trick behind the
//      glass-transparent LiDAR you'll build in week 3.
//
// Usage:  ros2 run whisker_amr_sim mujoco_hello [path/to/model.xml]

#include <mujoco/mujoco.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

namespace {

// RAII wrappers so the model/data are always freed, even on early return.
struct ModelDeleter { void operator()(mjModel* m) const { mj_deleteModel(m); } };
struct DataDeleter  { void operator()(mjData* d)  const { mj_deleteData(d); } };
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr  = std::unique_ptr<mjData, DataDeleter>;

// Yaw (rad) from a MuJoCo quaternion (w, x, y, z).
double yawFromQuat(const mjtNum* q) {
  return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                    1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

// Read a named sensor; returns pointer into d->sensordata.
const mjtNum* sensor(const mjModel* m, const mjData* d, const char* name) {
  const int id = mj_name2id(m, mjOBJ_SENSOR, name);
  return id < 0 ? nullptr : d->sensordata + m->sensor_adr[id];
}

// Cast a ray along the robot's +x axis from the lidar site.
// include_glass=false masks out geom group 3.
double castForwardRay(const mjModel* m, const mjData* d, bool include_glass, int* hit_geom) {
  const int site = mj_name2id(m, mjOBJ_SITE, "lidar_site");
  const int body = mj_name2id(m, mjOBJ_BODY, "base_link");
  const mjtNum* pnt = d->site_xpos + 3 * site;
  const mjtNum* R = d->site_xmat + 9 * site;              // row-major 3x3
  const mjtNum dir[3] = {R[0], R[3], R[6]};               // site's x axis in world

  // geomgroup: 1 = include that group, 0 = exclude. Groups 0..5.
  mjtByte groups[mjNGROUP] = {1, 1, 1, static_cast<mjtByte>(include_glass), 1, 1};
  int geomid[1] = {-1};
  const mjtNum dist = mj_ray(m, d, pnt, dir, groups, /*flg_static=*/1,
                             /*bodyexclude=*/body, geomid, /*normal=*/nullptr);
  *hit_geom = geomid[0];
  return dist;  // -1 if nothing hit
}

}  // namespace

int main(int argc, char** argv) {
  std::string path;
  if (argc > 1) {
    path = argv[1];
  } else {
    path = ament_index_cpp::get_package_share_directory("whisker_amr_sim") +
           "/models/amr_world.xml";
  }

  std::array<char, 1000> err{};
  ModelPtr m(mj_loadXML(path.c_str(), nullptr, err.data(), err.size()));
  if (!m) {
    std::fprintf(stderr, "Failed to load %s\n%s\n", path.c_str(), err.data());
    return 1;
  }
  DataPtr d(mj_makeData(m.get()));

  std::printf("MuJoCo %s | model '%s' | nbody=%ld ngeom=%ld nu=%ld nsensor=%ld | dt=%.4f s\n",
              mj_versionString(), path.c_str(), static_cast<long>(m->nbody),
              static_cast<long>(m->ngeom), static_cast<long>(m->nu), static_cast<long>(m->nsensor),
              m->opt.timestep);

  // Arc: right wheel faster than left -> turns left.
  d->ctrl[0] = 6.0;   // left wheel, rad/s
  d->ctrl[1] = 8.0;   // right wheel, rad/s

  const double t_end = 5.0;
  double next_print = 0.0;
  mj_forward(m.get(), d.get());   // populate sensors for t = 0
  while (d->time < t_end + 1e-9) {
    if (d->time >= next_print - 1e-9) {
      const mjtNum* p = sensor(m.get(), d.get(), "gt_pos");
      const mjtNum* q = sensor(m.get(), d.get(), "gt_quat");
      const mjtNum* el = sensor(m.get(), d.get(), "enc_left");
      const mjtNum* er = sensor(m.get(), d.get(), "enc_right");
      std::printf("t=%4.1f  x=%+6.3f y=%+6.3f yaw=%+6.1f deg  enc[L,R]=[%5.2f %5.2f] rad/s\n",
                  d->time, p[0], p[1], yawFromQuat(q) * 180.0 / M_PI, el[0], er[0]);
      next_print += 1.0;
    }
    mj_step(m.get(), d.get());
  }

  // Glass-transparency demo: reset, then face the interior glass pane at x=1.2.
  mj_resetData(m.get(), d.get());
  mj_forward(m.get(), d.get());
  int g_all = -1, g_noglass = -1;
  const double r_all = castForwardRay(m.get(), d.get(), /*include_glass=*/true, &g_all);
  const double r_noglass = castForwardRay(m.get(), d.get(), /*include_glass=*/false, &g_noglass);
  auto name = [&](int g) {
    const char* n = g >= 0 ? mj_id2name(m.get(), mjOBJ_GEOM, g) : nullptr;
    return n ? n : "none";
  };
  auto fmt = [](double r) { return r < 0 ? std::string("no return") : std::to_string(r) + " m"; };
  std::printf("\nForward ray, glass visible : %s  (hit %s)\n", fmt(r_all).c_str(), name(g_all));
  std::printf("Forward ray, glass masked  : %s  (hit %s)\n", fmt(r_noglass).c_str(), name(g_noglass));
  std::printf("-> A real LiDAR 'sees through' glass like the second ray. "
              "That gap is what the whiskers will cover.\n");
  return 0;
}
