cmake_minimum_required(VERSION 3.5)
project(bt_bumpgoo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ZMQ REQUIRED)

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

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(br2_forward_bt_node SHARED src/bt_bumpgo/Forward.cpp)
add_library(br2_back_bt_node SHARED src/bt_bumpgo/Back.cpp)
add_library(br2_turn_bt_node SHARED src/bt_bumpgo/Turn.cpp)
add_library(br2_is_obstacle_bt_node SHARED src/bt_bumpgo/IsObstacle.cpp)

list(APPEND plugin_libs
  br2_forward_bt_node
  br2_back_bt_node
  br2_turn_bt_node
  br2_is_obstacle_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(bt_bumpgo_main src/bt_bumpgo_main.cpp)
ament_target_dependencies(bt_bumpgo_main ${dependencies})
target_link_libraries(bt_bumpgo_main ${plugin_libs} ${ZMQ_LIBRARIES})

install(TARGETS
  br2_forward_bt_node
  br2_back_bt_node
  br2_turn_bt_node
  br2_is_obstacle_bt_node
  bt_bumpgo_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY behavior_tree_xml/
  DESTINATION share/${PROJECT_NAME}/behavior_tree_xml
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
