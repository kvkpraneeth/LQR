cmake_minimum_required(VERSION 3.5)

project(lqr)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(lqrtest src/test.cpp)
ament_target_dependencies(lqrtest Eigen3)
target_include_directories(lqrtest PUBLIC include)

add_library(ros2_lqr SHARED src/ros2_lqr.cpp)
ament_target_dependencies(ros2_lqr rclcpp Eigen3)
target_include_directories(ros2_lqr PUBLIC include)

install(DIRECTORY include/
    DESTINATION include
)

install(TARGETS lqrtest
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ros2_lqr
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

endif()

set(INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
    include
)

set(dependencies
    rclcpp
    Eigen3)

ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(ros2_lqr)
ament_export_dependencies(${dependencies})
ament_package()
