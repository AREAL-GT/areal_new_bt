cmake_minimum_required(VERSION 3.5)
project(areal_new_bt)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Set to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

#added by Ethan
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(areal_landing_uav_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  # rclcpp_lifecycle

  areal_landing_uav_interfaces

  behaviortree_cpp
)

#some ament stuff, should learn later
set(library_name new_bt)


add_executable(new_bt src/main.cpp)
target_include_directories(new_bt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> behaviortree_cpp)


 ament_target_dependencies(new_bt ${dependencies})
#  ament_target_dependencies(behavior_tree_sequencer BT::behaviortree_cpp)
 target_link_libraries(new_bt BT::behaviortree_cpp)

# install targets my_node new_bt
install(TARGETS
  new_bt
  DESTINATION lib/${PROJECT_NAME}
)

# bring xml file into binary directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/my_tree.xml
    ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(${dependencies})



ament_package()
