# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 23rd March 2022 9:59:40 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    scan-tools
# @brief      CMake file for `point_cloud_assembler` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(point_cloud_assembler)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Require C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ========================================================== Dependencies ========================================================== #

# Librarie's link dependencies
list(APPEND LINK_DEPENDENCIES
    rclcpp
    rclcpp_components
    pcl_ros
    node_common_cpp
)

# Librarie's dependencies
list(APPEND DEPENDENCIES

    # Link dependencies
    ${LINK_DEPENDENCIES}
    # Additional dependencies
    tf2_ros
    PCL
    pcl_conversions
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
    PLUGIN_NAME "scan_tools::PointCloudAssembler"
    # Executable target
    EXECUTABLE_NAME ${PROJECT_NAME}_node

    # Sources
    SOURCES
        src/point_cloud_assembler.cpp

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
