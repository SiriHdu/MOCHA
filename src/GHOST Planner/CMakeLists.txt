cmake_minimum_required(VERSION 3.8)
project(ghost_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -DNDEBUG)
  message(STATUS "Build type: Release with -O3 optimization")
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_compile_options(-O2 -g)
  message(STATUS "Build type: RelWithDebInfo with -O2 optimization")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O0 -g)
  message(STATUS "Build type: Debug (no optimization)")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# CGAL may be optional depending on system
find_package(CGAL QUIET)

set(dependencies
  rclcpp
  rclcpp_action
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GlobalPlan.action"
  "msg/Trajectory.msg"
  "msg/CommittedTrajectory.msg"
  "msg/ControlPath.msg"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

# MOCHA planner
add_executable(global_planner
  src/global_planner_action_server.cpp
  src/cgal_homotopy_planner.cpp
  src/mocha_optimizer.cpp
)
ament_target_dependencies(global_planner ${dependencies})
target_include_directories(global_planner PRIVATE include third_party)

if(CGAL_FOUND)
  target_compile_definitions(global_planner PRIVATE HAVE_CGAL)
  target_link_libraries(global_planner CGAL::CGAL)
endif()

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(global_planner ${cpp_typesupport_target})

install(TARGETS global_planner
  DESTINATION lib/${PROJECT_NAME})

# MOCHA verification planner
add_executable(verifation_global_planner
  src/verifation_global_planner.cpp
)
ament_target_dependencies(verifation_global_planner ${dependencies})
target_include_directories(verifation_global_planner PRIVATE include)
rosidl_get_typesupport_target(cpp_typesupport_target_verif ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(verifation_global_planner ${cpp_typesupport_target_verif})

install(TARGETS verifation_global_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz)

# Additional executables for TF and obstacle publishing
add_executable(fake_robot_tf_publisher
  src/fake_robot_tf_publisher.cpp
)
ament_target_dependencies(fake_robot_tf_publisher rclcpp tf2_ros geometry_msgs)
target_include_directories(fake_robot_tf_publisher PRIVATE include)
install(TARGETS fake_robot_tf_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(random_cylinder_publisher
  src/random_cylinder_publisher.cpp
)
ament_target_dependencies(random_cylinder_publisher rclcpp visualization_msgs)
target_include_directories(random_cylinder_publisher PRIVATE include)
rosidl_get_typesupport_target(cpp_typesupport_target_random ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(random_cylinder_publisher ${cpp_typesupport_target_random})
install(TARGETS random_cylinder_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()


