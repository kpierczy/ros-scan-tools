# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Alexander Karavaev (alexander.karavaev@jetbrains.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 12th April 2022 1:47:39 am
# @modified   Tuesday, 21st June 2022 6:26:32 pm
# @project    engineering-thesis
# @brief      CMake file for the `CSM` library package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.5)
# Project header
project(csm)

# ========================================================== Configuration ========================================================= #

# Add win-specific optiomizations
if(WIN32)
    add_definitions(-DWINDOWS)
    if(MSVC)
        add_definitions("/arch:SSE2")
    endif()
endif()


# ========================================================== Dependencies ========================================================== #


# Librarie's dependencies
list(APPEND DEPENDENCIES
    rclcpp
    Eigen3
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

# CSM sources
set(CSM_SRC
  src/csm/laser_data.cpp
  src/csm/math_utils.cpp
  src/csm/math_utils_gsl.cpp
  src/csm/utils.cpp
  src/csm/logging.cpp
  src/csm/sm_options.cpp
  src/csm/orientation.cpp
  src/csm/clustering.cpp)

# EGSL sources
set(EGSL_SRC
  src/egsl/egsl.cpp
  src/egsl/egsl_conversions.cpp
  src/egsl/egsl_misc.cpp
  src/egsl/egsl_ops.cpp)

# GPC sources
set(GPC_SRC
  src/gpc/gpc.cpp
  src/gpc/gpc_utils.cpp)

# ICP sources
set(ICP_SRC
  src/icp/icp.cpp
  src/icp/icp_corr_dumb.cpp
  src/icp/icp_corr_tricks.cpp
  src/icp/icp_covariance.cpp
  src/icp/icp_debug.cpp
  src/icp/icp_loop.cpp
  src/icp/icp_outliers.cpp)

# Options sources
set(OPTIONS_SRC
  src/options/options.cpp
  src/options/options_interface.cpp)
  
# ============================================================= Targets ============================================================ #

# Add target
add_ament_library(

    # Library name
    LIBRARY_NAME ${PROJECT_NAME}

    # Library sources
    SOURCES
        ${CSM_SRC}
        ${EGSL_SRC}
        ${GPC_SRC}
        ${ICP_SRC}
        ${OPTIONS_SRC}

    # Includes
    ADDITIONAL_INCLUDES
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src>"
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}

    # Dependencies
    AMENT_DEPENDENCIES ${LIB_DEPENDENCIES}
    
)

# Link target dependencies
ament_target_dependencies(${PROJECT_NAME} 
    PUBLIC
        rclcpp
        Eigen3
)

# ============================================================= Shared ============================================================= #

# Install shared resources
install(
    DIRECTORY
        resource
    DESTINATION
        share/${PROJECT_NAME}
)

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
