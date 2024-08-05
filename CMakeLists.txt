cmake_minimum_required(VERSION 3.0.2)
project(graph_ros_tests)

add_compile_options(-std=c++2a -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  moveit_collision_checker
  graph_display
  )

find_package(graph_core REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES graph_ros_tests
  CATKIN_DEPENDS moveit_collision_checker graph_display
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

add_executable(test_sampler src/${PROJECT_NAME}/test_sampler.cpp)
target_link_libraries(test_sampler
  ${catkin_LIBRARIES}
  graph_core::graph_core
  )

add_executable(test_parallel_checker src/${PROJECT_NAME}/test_parallel_checker.cpp)
target_link_libraries(test_parallel_checker
  ${catkin_LIBRARIES}
  graph_core::graph_core
  )


# Install
install(TARGETS test_solver test_path test_sampler test_parallel_checker
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config script
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
