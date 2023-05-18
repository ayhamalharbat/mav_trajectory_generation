cmake_minimum_required(VERSION 2.8.3)
project(mav_trajectory_generation_example)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

############
# BINARIES #
############
add_executable(trajectory_generation_example
        src/example_planner.cpp
        src/example_planner_node.cpp
)

add_dependencies(trajectory_generation_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(trajectory_generation_example
${catkin_LIBRARIES}
)

##########
# EXPORT #
##########
# install()
# export()
