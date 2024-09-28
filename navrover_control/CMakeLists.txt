cmake_minimum_required(VERSION 3.8)
project(navrover_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)  # Add any other necessary dependencies
# find_package(gz_ros REQUIRED)  # If using Gazebo
# find_package(controller_interface REQUIRED)  # If using ROS 2 control
install(
    DIRECTORY config description
    DESTINATION share/$(PROJECT_NAME)
)

# Uncomment the following line in order to fill in
# further dependencies manually
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # The following line skips the linter which checks for copyrights 
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # The following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()


