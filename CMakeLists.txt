cmake_minimum_required(VERSION 3.0.2)
project(graph_ros_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-funroll-loops -Wall -Ofast -flto) #-O3
  endif()
  if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-Wall -g -Og)
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY script
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.sh"
    PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ)
