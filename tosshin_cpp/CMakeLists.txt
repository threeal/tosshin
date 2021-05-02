cmake_minimum_required(VERSION 3.5)

project(tosshin_cpp)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(keisan REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tosshin_interfaces REQUIRED)

install(DIRECTORY "include" DESTINATION ".")

add_executable(navigation_dummy "examples/navigation_dummy.cpp")
add_executable(navigation_teleop "examples/navigation_teleop.cpp")

foreach(target navigation_dummy navigation_teleop)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(${target}
    keisan rclcpp tosshin_interfaces)

  install(TARGETS ${target}
    DESTINATION "lib/${PROJECT_NAME}")
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(keisan rclcpp tosshin_interfaces)
ament_export_include_directories("include")

ament_package()
