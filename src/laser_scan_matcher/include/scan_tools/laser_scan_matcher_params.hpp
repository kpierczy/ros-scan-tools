/* ============================================================================================================================ *//**
 * @file       laser_scan_matcher_params.hpp
 * @author     Ivan Dryanovski (ccnyroboticslab@gmail.com)
 * @author     William Morris
 * @author     Andrea Censi
 * @author     Carlos (cjaramillo@gc.cuny.edu)
 * @author     Isaac I.Y. Saito (130s@2000.jukuin.keio.ac.jp)
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 30th March 2022 9:30:14 am
 * @modified   Wednesday, 25th May 2022 11:09:56 pm
 * @project    scan-tools
 * @brief      Definitions of ROS parameters of the `laser_scan_matcher` node
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __LASER_SCAN_MATCHER_PARAMS_H__
#define __LASER_SCAN_MATCHER_PARAMS_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <string>
// Private includes
#include "node_common/parameters.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* =========================================================== Constants ========================================================== */

/// Number of diagonal elements in the position covariance matrix 
static constexpr std::size_t POSITION_COVARIANCE_MATRIX_DIAGONAL_ELEMENTS = 3;
/// Number of diagonal elements in the orientation covariance matrix 
static constexpr std::size_t ORIENTATION_COVARIANCE_MATRIX_DIAGONAL_ELEMENTS = 3;

/* ======================================================== Base parameters ======================================================= */

static constexpr node_common::parameters::ParamDescriptor<std::string> REFERENCE_FRAME_PARAM_DESCRIPTOR {
    .name           = "reference_frame",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = "odom",
    .description    = "Reference frame of the estimated pose of the robot"
};

static constexpr node_common::parameters::ParamDescriptor<std::string> BASE_FRAME_PARAM_DESCRIPTOR {
    .name           = "base_frame",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = "base_link",
    .description    = "Base frame of the robot"
};

static constexpr node_common::parameters::ParamDescriptor<std::string> TARGET_FRAME_PARAM_DESCRIPTOR {
    .name           = "target_frame",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = "base_link",
    .description    = "Target frame representing estimated pose of the robot in the reference frame"
};

/* ======================================================= Motion prediction ====================================================== */

static constexpr node_common::parameters::ParamDescriptor<bool> USE_IMU_PARAM_DESCRIPTOR {
    .name           = "use_imu",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = true,
    .description    = "Whether to use an imu for the theta prediction of the scan registration. Requires input on /imu/data topic"
};

static constexpr node_common::parameters::ParamDescriptor<bool> USE_ODOM_PARAM_DESCRIPTOR {
    .name           = "use_odom",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = true,
    .description    = "Whether to use wheel odometry for the x-, y-, and theta prediction of the scan registration. Requires input on odom topic"
};

static constexpr node_common::parameters::ParamDescriptor<bool> USE_VEL_PARAM_DESCRIPTOR {
    .name           = "use_vel",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "Whether to use constant velocity model for the x-, y-, and theta prediction of the scan registration. Requires input on vel topic"
};

static constexpr node_common::parameters::ParamDescriptor<bool> STAMPED_VEL_PARAM_DESCRIPTOR {
    .name           = "stamped_vel",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "Whether to the input velocity topic broadcasts geometry_msgs/Twist or geometry_msgs/TwistStamped messages"
};

/* ======================================================= Input parameters ======================================================= */
 
static constexpr node_common::parameters::ParamDescriptor<double> RANGE_MIN_PARAM_DESCRIPTOR {
    .name           = "range_min",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.1,
    .description    = "The minimum range to the pointto be taking into account in calculations in [m]"
};
 
static constexpr node_common::parameters::ParamDescriptor<double> RANGE_MAX_PARAM_DESCRIPTOR {
    .name           = "range_max",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 50.0,
    .description    = "The maximum range to the pointto be taking into account in calculations in [m]"
};

/* ==================================================== Cloud input parameters ==================================================== */ 

static constexpr node_common::parameters::ParamDescriptor<bool> USE_CLOUD_INPUT_PARAM_DESCRIPTOR {
    .name           = "use_cloud_input",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "If 'True' the points cloud topic will be used as an input data stream "
                      "instead of laser scan topic"
};

static constexpr node_common::parameters::ParamDescriptor<double> CLOUD_RES_PARAM_DESCRIPTOR {
    .name           = "cloud_res",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.05,
    .description    = "Resolution of the cloud (minimum acceptable distance between two points to be taken "
                      "into account in computations"
};
 
static constexpr node_common::parameters::ParamDescriptor<bool> CLOUD_ACCEPT_NAN_PARAM_DESCRIPTOR {
    .name           = "cloud_accept_nan",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = true,
    .description    = "When set to 'true' node will silently mark points containing NaN values. Otherwise "
                      "it will print an error each time a NaN value is read from incoming cloud."
};
 
static constexpr node_common::parameters::ParamDescriptor<bool> CLOUD_SORTED_PARAM_DESCRIPTOR {
    .name           = "cloud_sorted",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = true,
    .description    = "Underlying CSM library requires reading to be sorted with respect to incrementing angles "
                      "of scanned poitns in the robot-centred polar coordinates system. If input clouds are not "
                      "sorted in such a way, the parameter may be set to 'false' to handle sorting in the node."
};

/* ============================================================ Output ============================================================ */

static constexpr node_common::parameters::ParamDescriptor<bool> PUBLISH_TF_PARAM_DESCRIPTOR {
    .name           = "publish_tf",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = true,
    .description    = "Whether to publish scan matcher's estimation for the position of the base frame in the world frame as a transform"
};

static constexpr node_common::parameters::ParamDescriptor<bool> PUBLISH_POSE_PARAM_DESCRIPTOR {
    .name           = "publish_pose",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = true,
    .description    = "Whether to publish scan matcher's estimation for the position of the base frame in the world frame as a geometry_msgs/Pose2D"
};

static constexpr node_common::parameters::ParamDescriptor<bool> PUBLISH_POSE_STAMPED_PARAM_DESCRIPTOR {
    .name           = "publish_pose_stamped",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "Whether to publish scan matcher's estimation for the position of the base frame in the world frame as a geometry_msgs/PoseStamped"
};

static constexpr node_common::parameters::ParamDescriptor<bool> PUBLISH_POSE_WITH_COVARIANCE_PARAM_DESCRIPTOR {
    .name           = "publish_pose_with_covariance",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "Whether to publish scan matcher's estimation for the position of the base frame in the world frame as a geometry_msgs/PoseWithCovariance"
};

static constexpr node_common::parameters::ParamDescriptor<bool> PUBLISH_POSE_WITH_COVARIANCE_STAMPED_PARAM_DESCRIPTOR {
    .name           = "publish_pose_with_covariance_stamped",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "Whether to publish scan matcher's estimation for the position of the base frame in the world frame as a geometry_msgs/PoseWithCovarianceStamped"
};

static constexpr node_common::parameters::ParamDescriptor<bool> PUBLISH_ODOM_PARAM_DESCRIPTOR {
    .name           = "publish_odom",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "Whether to publish scan matcher's estimations as a nav_msgs/Odometry"
};

static constexpr node_common::parameters::ParamDescriptor<
    std::vector<double>,
    POSITION_COVARIANCE_MATRIX_DIAGONAL_ELEMENTS /* Number of elements in the 'default value' array */ 
> POSITION_COVARIANCE_PARAM_DESCRIPTOR {
    .name           = "position_covariance",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = std::array{ 1e-9, 1e-9, 1e-9 },
    .description    = "3D vector defining diagonal elements of the position covariance matrix"
};

static constexpr node_common::parameters::ParamDescriptor<
    std::vector<double>,
    ORIENTATION_COVARIANCE_MATRIX_DIAGONAL_ELEMENTS /* Number of elements in the 'default value' array */ 
> ORIENTATION_COVARIANCE_PARAM_DESCRIPTOR {
    .name           = "orientation_covariance",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = std::array{ 1e-9, 1e-9, 1e-9 },
    .description    = "3D vector defining diagonal elements of the orientation covariance matrix"
};

/* =========================================================== Keyframes ========================================================== */

static constexpr node_common::parameters::ParamDescriptor<double> KF_DIST_LINEAR_PARAM_DESCRIPTOR {
    .name           = "kf_dist_linear",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.10,
    .description    = "What distance the fixed frame needs to move before updating the keyframe scan (in [m])"
};

static constexpr node_common::parameters::ParamDescriptor<double> KF_DIST_ANGULAR_PARAM_DESCRIPTOR {
    .name           = "kf_dist_angular",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.175,
    .description    = "What angle the fixed frame needs to move before updating the keyframe scan (in [rad])"
};

/* ========================================================= Scan matching ======================================================== */

static constexpr node_common::parameters::ParamDescriptor<int> MAX_ITERATIONS_PARAM_DESCRIPTOR {
    .name           = "max_iterations",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 10,
    .description    = "Maximum ICP cycle iterations"
};

static constexpr node_common::parameters::ParamDescriptor<double> MAX_CORRESPONDENCE_DIST_PARAM_DESCRIPTOR {
    .name           = "max_correspondence_dist",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.3,
    .description    = "Maximum distance for a correspondence to be valid"
};

static constexpr node_common::parameters::ParamDescriptor<double> MAX_ANGULAR_CORRECTION_DEG_PARAM_DESCRIPTOR {
    .name           = "max_angular_correction_deg",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 45.0,
    .description    = "Maximum angular displacement between scans [deg]"
};

static constexpr node_common::parameters::ParamDescriptor<double> MAX_LINEAR_CORRECTION_PARAM_DESCRIPTOR {
    .name           = "max_linear_correction",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.50,
    .description    = "Maximum translation between scans [m]"
};

static constexpr node_common::parameters::ParamDescriptor<double> EPSILON_XY_PARAM_DESCRIPTOR {
    .name           = "epsilon_xy",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.000001,
    .description    = "Threshold for stopping [m]"
};

static constexpr node_common::parameters::ParamDescriptor<double> EPSILON_THETA_PARAM_DESCRIPTOR {
    .name           = "epsilon_theta",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.000001,
    .description    = "Threshold for stopping [rad]"
};

static constexpr node_common::parameters::ParamDescriptor<double> OUTLIERS_MAX_PRC_PARAM_DESCRIPTOR {
    .name           = "outliers_max_prc",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.90,
    .description    = "Percentage of correspondences to consider: if 0.90, always discard the top 10% of correspondences with more error"
};

/* =================================================== Scan matching (advanced) =================================================== */

static constexpr node_common::parameters::ParamDescriptor<double> SIGMA_PARAM_DESCRIPTOR {
    .name           = "sigma",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.010,
    .description    = "Noise in the scan [m] (Not sure if changing this has any effect in the current implementation)"
};

static constexpr node_common::parameters::ParamDescriptor<bool> USE_CORR_TRICKS_PARAM_DESCRIPTOR {
    .name           = "use_corr_tricks",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = true,
    .description    = "If 'True', use smart tricks for finding correspondences (see paper)."
};

static constexpr node_common::parameters::ParamDescriptor<bool> RESTART_PARAM_DESCRIPTOR {
    .name           = "restart",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "Restart: If 'True', restart if error is over threshold"
};

static constexpr node_common::parameters::ParamDescriptor<double> RESTART_THRESHOLD_MEAN_ERROR_PARAM_DESCRIPTOR {
    .name           = "restart_threshold_mean_error",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.01,
    .description    = "Restart: Threshold for restarting"
};

static constexpr node_common::parameters::ParamDescriptor<double> RESTART_DT_PARAM_DESCRIPTOR {
    .name           = "restart_dt",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 1.0,
    .description    = "Restart: displacement for restarting. [m]"
};

static constexpr node_common::parameters::ParamDescriptor<double> RESTART_DTHETA_PARAM_DESCRIPTOR {
    .name           = "restart_dtheta",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.1,
    .description    = "Restart: displacement for restarting. [rad]"
};

static constexpr node_common::parameters::ParamDescriptor<double> CLUSTERING_THRESHOLD_PARAM_DESCRIPTOR {
    .name           = "clustering_threshold",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.25,
    .description    = "Max distance for staying in the same clustering"
};

static constexpr node_common::parameters::ParamDescriptor<int> ORIENTATION_NEIGHBOURHOOD_PARAM_DESCRIPTOR {
    .name           = "orientation_neighbourhood",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 10,
    .description    = "Number of neighbour rays used to estimate the orientation"
};

static constexpr node_common::parameters::ParamDescriptor<bool> USE_POINT_TO_LINE_DISTANCE_PARAM_DESCRIPTOR {
    .name           = "use_point_to_line_distance",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = true,
    .description    = "If 'False', it's vanilla ICP"
};

static constexpr node_common::parameters::ParamDescriptor<bool> DO_ALPHA_TEST_PARAM_DESCRIPTOR {
    .name           = "do_alpha_test",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "If 'True', discard correspondences based on the angles"
};

static constexpr node_common::parameters::ParamDescriptor<double> DO_ALPHA_TEST_THRESHOLD_DEG_PARAM_DESCRIPTOR {
    .name           = "do_alpha_test_threshold_deg",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 20.0,
    .description    = "Discard correspondences based on the angles - threshold angle, in degrees"
};

static constexpr node_common::parameters::ParamDescriptor<double> OUTLIERS_ADAPTIVE_ORDER_PARAM_DESCRIPTOR {
    .name           = "outliers_adaptive_order",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 0.7,
    .description    = "Parameters describing a simple adaptive algorithm for discarding. 1) Order the errors. 2) "
                      "Choose the percentile according to outliers_adaptive_order (if it is 0.7, get the 70% "
                      "percentile). 3) Define an adaptive threshold multiplying outliers_adaptive_mult with the "
                      "value of the error at the chosen percentile. 4) Discard correspondences over the threshold. "
                      "This is useful to be conservative; yet remove the biggest errors."
};

static constexpr node_common::parameters::ParamDescriptor<double> OUTLIERS_ADAPTIVE_MUL_PARAM_DESCRIPTOR {
    .name           = "outliers_adaptive_mul",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = 2.0,
    .description    = "Parameters describing a simple adaptive algorithm for discarding. 1) Order the errors. 2) "
                      "Choose the percentile according to outliers_adaptive_order (if it is 0.7, get the 70% "
                      "percentile). 3) Define an adaptive threshold multiplying outliers_adaptive_mult with the "
                      "value of the error at the chosen percentile. 4) Discard correspondences over the threshold. "
                      "This is useful to be conservative; yet remove the biggest errors."
};

static constexpr node_common::parameters::ParamDescriptor<bool> DO_VISIBILITY_TEST_PARAM_DESCRIPTOR {
    .name           = "do_visibility_test",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "If you already have a guess of the solution, you can compute the polar angle of the points of "
                      "one scan in the new position. If the polar angle is not a monotone function of the readings index, "
                      "it means that the surface is not visible in the next position. If it is not visible, then we don't "
                      "use it for matching."
};

static constexpr node_common::parameters::ParamDescriptor<bool> OUTLIERS_REMOVE_DOUBLES_PARAM_DESCRIPTOR {
    .name           = "outliers_remove_doubles",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = true,
    .description    = "If 'True', no two points in laser_sens can have the same correspondence"
};

static constexpr node_common::parameters::ParamDescriptor<bool> DO_COMPUTE_COVARIANCE_PARAM_DESCRIPTOR {
    .name           = "do_compute_covariance",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "If 'True', computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov (Changing "
                      "this has no effect in the current implementation)"
};

static constexpr node_common::parameters::ParamDescriptor<bool> DEBUG_VERIFY_TRICK_PARAM_DESCRIPTOR {
    .name           = "debug_verify_trick",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "If 'True', checks that find_correspondences_tricks gives the right answer"
};

static constexpr node_common::parameters::ParamDescriptor<bool> USE_ML_WEIGHTS_PARAM_DESCRIPTOR {
    .name           = "use_ml_weights",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "If 'True', the field 'true_alpha' (or 'alpha') in the first scan is used to compute the incidence beta, "
                      "and the factor (1/cos^2(beta)) used to weight the correspondence. (Changing this has no effect in the "
                      "current implementation)"
};

static constexpr node_common::parameters::ParamDescriptor<bool> USE_SIGMA_WEIGHTS_PARAM_DESCRIPTOR {
    .name           = "use_sigma_weights",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "If 'True', the field 'readings_sigma' in the second scan is used to weight the correspondence by 1/sigma^2 "
                      "(Not sure if changing this has any effect in the current implementation)"
};

static constexpr node_common::parameters::ParamDescriptor<bool> DEBUG_CSM_PARAM_DESCRIPTOR {
    .name           = "debug_csm",
    .read_only      = true,
    .dynamic_typing = false,
    .default_value  = false,
    .description    = "If 'True', debug mode of the CSM library will be enabled"
};

/* ================================================================================================================================ */

} // End namespace scan_tools

#endif
