cmake_minimum_required(VERSION 3.5)
project(bt_bumpgo_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED PATHS /usr/local/share/behaviortree_cpp/cmake)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ZMQ QUIET)

if(ZMQ_FOUND)
  message(STATUS "ZeroMQ found.")
  add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(dependencies
  rclcpp
  behaviortree_cpp
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include ${ZMQ_INCLUDE_DIRS})

ament_add_gtest(bt_action_test bt_action_test.cpp)
ament_target_dependencies(bt_action_test ${dependencies})

add_executable(bt_forward bt_forward_main.cpp)
ament_target_dependencies(bt_forward ${dependencies})
target_link_libraries(bt_forward ${ZMQ_LIBRARIES})

install(TARGETS
  bt_forward
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
