cmake_minimum_required(VERSION 3.5)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

project(better_lidar)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS graphics)

add_library(better_lidar SHARED
  src/plugin/BetterLidar.cpp
  src/plugin/BetterLidarPlugin.cpp
  src/plugin/GlRenderer.cpp
  src/plugin/mesh_generator.cpp
  src/plugin/glad/src/gl.c
  src/plugin/glad/src/egl.c
)
target_include_directories(better_lidar PRIVATE
  src/plugin
  src/plugin/glad/include
)
target_compile_definitions(better_lidar PRIVATE
  SHADER_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/shader/"
)
target_link_libraries(better_lidar
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5-graphics
  EGL
  dl
)
set_target_properties(better_lidar PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")

install(TARGETS better_lidar
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  src/example/launch
  src/example/worlds
  src/example/config
  src/example/urdf
  src/example/models
  src/plugin/shader
  DESTINATION share/${PROJECT_NAME})

ament_package()