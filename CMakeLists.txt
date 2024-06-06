cmake_minimum_required(VERSION 3.8)
project(graph_ros_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-funroll-loops -Wall -Ofast)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(moveit_collision_checker REQUIRED)
find_package(graph_display REQUIRED)
find_package(graph_core REQUIRED)

add_executable(test_solver src/${PROJECT_NAME}/test_solver.cpp)
target_link_libraries(test_solver  graph_core::graph_core)
ament_target_dependencies(test_solver graph_display moveit_collision_checker)

add_executable(test_path src/${PROJECT_NAME}/test_path.cpp)
target_link_libraries(test_path graph_core::graph_core)
ament_target_dependencies(test_path graph_display moveit_collision_checker)

# =======
# INSTALL
# =======
install(TARGETS
        test_solver
        test_path
    DESTINATION lib/${PROJECT_NAME}
    )

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

# ====
# TEST
# ====
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_cmake_gmock REQUIRED)
#   # ament_add_gtest(some_test <test_sources>)
#   # ament_add_gmock(some_test <test_sources>)
# endif()

ament_package()
