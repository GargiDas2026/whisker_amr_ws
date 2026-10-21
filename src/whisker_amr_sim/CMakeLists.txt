cmake_minimum_required(VERSION 3.16)
project(whisker_amr_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # for clangd / VS Code IntelliSense

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mujoco_vendor REQUIRED)      # provides imported target mujoco::mujoco

# ---- Week 1: standalone MuJoCo sanity check ----
add_executable(mujoco_hello src/mujoco_hello.cpp)
target_link_libraries(mujoco_hello mujoco::mujoco ament_index_cpp::ament_index_cpp)

install(TARGETS mujoco_hello DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY models launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)   # skip copyright header check for now
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
