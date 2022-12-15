cmake_minimum_required(VERSION 2.8.3)
project(sdh_interface)


find_package(catkin REQUIRED COMPONENTS sensor_msgs
)


catkin_package()
catkin_python_setup()
  
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


