# whisker_amr_sim

MuJoCo simulation of a differential-drive AMR with LiDAR, camera and
rat-whisker-inspired tactile sensors, bridged to ROS 2 Jazzy.
Goal: keep navigating when LiDAR and vision fail (glass, fog).

## Status — Week 1

- `models/amr_world.xml`: 6 m × 6 m room, diff-drive robot (r = 0.05 m,
  track = 0.26 m), IMU, wheel encoders, front camera, LiDAR mount site,
  a glass east wall and an interior glass pane.
- `src/mujoco_hello.cpp`: loads the model, drives an arc for 5 s, prints
  ground-truth pose and encoders, and demonstrates glass transparency with
  a geom-group-masked ray.

## Conventions

| Geom group | Meaning |
|---|---|
| 0 | Normal geometry: collides, visible to LiDAR and camera |
| 3 | Glass: collides physically, ignored by the LiDAR |

Ground-truth frame sensors use `objtype="xbody"` (body frame).
`objtype="body"` would report the inertial frame instead.

## Build and run

```bash
cd ~/whisker_amr_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run whisker_amr_sim mujoco_hello
```

View the world interactively:

```bash
ros2 run mujoco_vendor simulate src/whisker_amr_sim/models/amr_world.xml
```
