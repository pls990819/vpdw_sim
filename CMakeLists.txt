cmake_minimum_required(VERSION 3.0.2)
project(vpdw_sim)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  visualization_msgs
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7)

add_message_files(
  FILES
  Terrain.msg
  # Message2.msg
)

generate_messages(
 DEPENDENCIES
#  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES learning_pkg
 CATKIN_DEPENDS roscpp rospy roslib std_msgs visualization_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
)

add_executable(vpdw_walker
  src/walker.cpp
  src/biped_robot.cpp
  )
target_link_libraries(vpdw_walker ${catkin_LIBRARIES})

add_executable(vpdw_plot
  src/plot_walking.cpp
  )
target_link_libraries(vpdw_plot ${catkin_LIBRARIES})

add_executable(ctc_controller
  src/ctc_controller.cpp
  )
target_link_libraries(ctc_controller ${catkin_LIBRARIES})

add_executable(adaptive_controller
  src/adaptive_controller.cpp
  )
target_link_libraries(adaptive_controller ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(adaptive_controller PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(plotlimitcycle
  src/plotlimitcycle.cpp
  )
target_link_libraries(plotlimitcycle ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(plotlimitcycle PRIVATE ${PYTHON_INCLUDE_DIRS})