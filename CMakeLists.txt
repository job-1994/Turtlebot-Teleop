cmake_minimum_required(VERSION 2.8.3)
project(teleop)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


set(CURSES_LIBRARY "/usr/lib/x86_64-linux-gnu/libncurses.so")
set(CURSES_INCLUDE_PATH "/usr/include")

find_package(Curses REQUIRED)


catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIRS}
)

add_executable(teleop_talker src/teleop_talker.cpp)
target_link_libraries(teleop_talker ${catkin_LIBRARIES} ${CURSES_LIBRARIES})
add_dependencies(teleop_talker teleop_generate_messages_cpp)

add_executable(key_numbers src/key_numbers.cpp)
target_link_libraries(key_numbers ${catkin_LIBRARIES} ${CURSES_LIBRARIES})
add_dependencies(key_numbers teleop_generate_messages_cpp)
