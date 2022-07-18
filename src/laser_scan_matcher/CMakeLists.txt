# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 23rd March 2022 9:59:40 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    scan-tools
# @brief      CMake file for `laser_scan_matcher` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(laser_scan_matcher)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Require C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ========================================================== Dependencies ========================================================== #

# Librarie's link dependencies
list(APPEND LINK_DEPENDENCIES
    
    # ROS dependencies
    rclcpp
    rclcpp_components
    node_common_cpp
    # Interfaces dependencies
    geometry_msgs
    sensor_msgs
    nav_msgs
    scan_tools_msgs
    # TF dependencies
    tf2
    tf2_ros
    tf2_geometry_msgs
    # External dependencies
    csm
    pcl_conversions
    pcl_ros
    range-v3
)

# Librarie's dependencies
list(APPEND DEPENDENCIES

    # Link dependencies
    ${LINK_DEPENDENCIES}
    # Find external dependencies
    PCL

)

# Common dependencies
find_package(cmake_common REQUIRED)
# Ament dependency (always present)
find_packages(

    # Ament dependencies
    ament_cmake
    ament_cmake_python
    # Find dependencies
    ${DEPENDENCIES}

)

# ============================================================= Source ============================================================= #

# Add node
add_ros_component_node(

    # Library target
    LIBRARY_NAME ${PROJECT_NAME}
    # Plugin target
    PLUGIN_NAME "scan_tools::LaserScanMatcher"
    # Executable target
    EXECUTABLE_NAME ${PROJECT_NAME}_node

    # Sources
    SOURCES
        src/laser_scan_matcher_algo.cpp
        src/laser_scan_matcher_callbacks.cpp
        src/laser_scan_matcher_helpers.cpp
        src/laser_scan_matcher_init.cpp
        src/laser_scan_matcher_publish.cpp

    # Ament dependencies
    AMENT_DEPENDENCIES ${LINK_DEPENDENCIES}
    
)

# ========================================================= Share installs ========================================================= #

# Install shared data
install(
    DIRECTORY
        config
    DESTINATION
        share/${PROJECT_NAME}
)

# ============================================================== Tests ============================================================= #

# Test builds
if(BUILD_TESTING)

    # Find linter
    find_package(ament_lint_auto REQUIRED)
    # Find test dependencies using info from `package.xml`
    ament_lint_auto_find_test_dependencies()

endif()

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
