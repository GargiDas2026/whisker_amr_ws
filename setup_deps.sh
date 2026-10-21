#!/usr/bin/env bash
# setup_deps.sh — one-time dependency install for the whisker-AMR project
# Target: Ubuntu 24.04 (noble) + ROS 2 Jazzy, Intel Iris Xe (Mesa OpenGL)
# Usage:  bash setup_deps.sh      (asks for your sudo password)
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  source /opt/ros/jazzy/setup.bash
fi

echo "==> [1/5] System build tools and C++ libraries"
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config clang-format clangd gdb \
  libeigen3-dev libyaml-cpp-dev libgtest-dev libgmock-dev \
  libglfw3-dev mesa-utils \
  python3-pip python3-venv

echo "==> [2/5] ROS 2 build tooling"
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-vcstool
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo "==> [3/5] MuJoCo (via ROS vendor package) + ROS packages for the project"
sudo apt install -y \
  ros-jazzy-mujoco-vendor \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-map-server \
  ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-eigen \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-rviz2 ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common ros-jazzy-ament-cmake-gtest

echo "==> [4/5] OSQP + osqp-eigen (for MPC, built from source into /usr/local)"
BUILD_DIR="$(mktemp -d)"
pushd "$BUILD_DIR" >/dev/null
git clone --depth 1 --branch v0.6.3 --recursive https://github.com/osqp/osqp.git
cmake -S osqp -B osqp/build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build osqp/build --target install -j"$(nproc)"
git clone --depth 1 --branch v0.8.1 https://github.com/robotology/osqp-eigen.git
cmake -S osqp-eigen -B osqp-eigen/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
sudo cmake --build osqp-eigen/build --target install -j"$(nproc)"
sudo ldconfig
popd >/dev/null
rm -rf "$BUILD_DIR"

echo "==> [5/5] Python tools (in a venv that can still see ROS Python packages)"
python3 -m venv --system-site-packages ~/.venvs/whisker_amr
~/.venvs/whisker_amr/bin/pip install --upgrade pip
~/.venvs/whisker_amr/bin/pip install mujoco numpy scipy matplotlib pandas evo

echo
echo "Done. Quick checks:"
echo "  ros2 pkg prefix mujoco_vendor"
echo "  ros2 run mujoco_vendor simulate      # MuJoCo viewer window should open"
