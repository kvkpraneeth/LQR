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

set(INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
    include
)

set(dependencies
    rclcpp
    Eigen3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(lqr src/main.cpp)
ament_target_dependencies(lqr ${dependencies})
target_include_directories(lqr PUBLIC include)

install(DIRECTORY include/
    DESTINATION include
)

install(TARGETS lqr
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

endif()

ament_export_include_directories(${INCLUDE_DIRS})
ament_export_dependencies(${dependencies})
ament_package()
