cmake_minimum_required(VERSION 3.0.2)
project(learning_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
  std_msgs
  message_generation
  roslib
)

add_service_files(
  FILES
  Turtle_pose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES learning_tf
#  CATKIN_DEPENDS roscpp rospy tf turtlesim
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})

add_executable(listenerClass src/listenerClass.cpp)
target_link_libraries(listenerClass ${catkin_LIBRARIES})

add_executable(frame_tf_broadcaster src/frame_tf_broadcaster.cpp)
target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})

add_executable(start_turtlesim_snake src/start_turtlesim_snake.cpp)
target_link_libraries(start_turtlesim_snake ${catkin_LIBRARIES})

add_executable(start_turtlesim_snake_terminal src/start_turtlesim_snake_terminal.cpp)
target_link_libraries(start_turtlesim_snake_terminal ${catkin_LIBRARIES})

add_executable(turtle_spawn src/turtle_spawn.cpp)
target_link_libraries(turtle_spawn ${catkin_LIBRARIES})

add_executable(snake_game src/snake_game.cpp)
target_link_libraries(snake_game ${catkin_LIBRARIES})
