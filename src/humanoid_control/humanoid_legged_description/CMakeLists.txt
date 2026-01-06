cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(humanoid_legged_description)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)


install(
        DIRECTORY config launch urdf rviz mjcf meshes
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})

if (BUILD_TESTING)

endif ()
ament_package()
