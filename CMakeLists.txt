cmake_minimum_required(VERSION 3.0.2)
project(graph_ros1_tests)

add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  graph_ros1
  graph_display
  )

find_package(cnr_logger REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time filesystem)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

#message("CNR LOGGER FOUND?")
#message(${cnr_logger_FOUND})
#message("BOOST FOUND?")
#message(${Boost_FOUND})
#message("YAML CPP FOUND?")
#message(${YAML_CPP_LIBRARIES})
#message("graph core FOUND?")
#message(${graph_core_FOUND})

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES graph_ros1_tests
    CATKIN_DEPENDS graph_ros1 graph_display
  # DEPENDS
  )

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}

  ${cnr_logger_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

add_executable(test_graph_core src/${PROJECT_NAME}/test_graph_core.cpp)
target_link_libraries(test_graph_core
  ${catkin_LIBRARIES}

  ${Boost_LIBRARIES}
  ${cnr_logger_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  )
