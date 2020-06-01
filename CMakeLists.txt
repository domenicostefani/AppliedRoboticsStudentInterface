cmake_minimum_required(VERSION 2.8.3)
project(student_project)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11
                    -O3
                    -Wall
                    -Wpedantic)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
 src/clipper-lib-cpp/
)

## CREATE EXECUTABLE LIBRARIES

add_library(student SHARED
   src/student_interface.cpp
)

add_library(dubins SHARED
  src/dubins.cpp
)

add_library(hsvpanel SHARED
  src/hsv_panel.cpp
)

add_library(clipperlib SHARED
  src/clipper-lib-cpp/clipper.cpp
)


## LINK LIBRARIES

target_link_libraries(student
  ${OpenCV_LIBRARIES}
)

target_link_libraries(hsvpanel
  ${OpenCV_LIBRARIES}
)

target_link_libraries(student
  dubins
)

target_link_libraries(student
  hsvpanel
)

target_link_libraries(student
    clipperlib
)

