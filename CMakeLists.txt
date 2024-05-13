cmake_minimum_required(VERSION 3.0.2)
project(graph_ros1_tests)

add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  graph_ros1
  graph_display
  )

find_package(graph_core REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES graph_ros1_tests
  CATKIN_DEPENDS graph_ros1 graph_display
  DEPENDS graph_core
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(test_solver src/${PROJECT_NAME}/test_solver.cpp)
target_link_libraries(test_solver
  ${catkin_LIBRARIES}
  graph_core::graph_core
  )

add_executable(test_path src/${PROJECT_NAME}/test_path.cpp)
target_link_libraries(test_path
  ${catkin_LIBRARIES}
  graph_core::graph_core
  )
