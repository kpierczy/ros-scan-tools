/* ============================================================================================================================ *//**
 * @file       laser_scan_matcher_init.cpp
 * @author     Ivan Dryanovski (ccnyroboticslab@gmail.com)
 * @author     William Morris
 * @author     Andrea Censi
 * @author     Carlos (cjaramillo@gc.cuny.edu)
 * @author     Isaac I.Y. Saito (130s@2000.jukuin.keio.ac.jp)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 30th March 2022 9:30:14 am
 * @modified   Wednesday, 25th May 2022 11:12:18 pm
 * @project    scan-tools
 * @brief      Definitions of the initialization methods of ROS2 node class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <algorithm>
// Private includes
#include "scan_tools/laser_scan_matcher.hpp"

/* ========================================================= Declarations ========================================================= */

// Private function of the CSM library (used for debug purpose)
void sm_debug_write(int enabled);

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* ========================================================= Ctors & dtors ======================================================== */

LaserScanMatcher::LaserScanMatcher(const rclcpp::NodeOptions & options) : 
    rclcpp::Node("laser_scan_matcher", options)
{
    // Initialize parameters
    initialize_params();

    // Initialize topics
    initialize_topics();

    // Initialize algorithm-specific data
    initialize_algorithm();

    // Log starting message
    RCLCPP_INFO_STREAM(this->get_logger(), "The '" << this->get_fully_qualified_name() << "' node has been started");
}

LaserScanMatcher::~LaserScanMatcher() {
    RCLCPP_INFO_STREAM(this->get_logger(), "The '" << this->get_fully_qualified_name() << "' node has been stoppeed");
}

/* =================================================== Parameters initialization ================================================== */

void LaserScanMatcher::initialize_params() {

    /* ------------------------ Register parameters ------------------------ */
    
    // Register TF-related parameters
    auto reference_frame = node_common::parameters::declare_parameter_and_get(*this, REFERENCE_FRAME_PARAM_DESCRIPTOR);
    auto base_frame = node_common::parameters::declare_parameter_and_get(*this, BASE_FRAME_PARAM_DESCRIPTOR);
    auto target_frame = node_common::parameters::declare_parameter_and_get(*this, TARGET_FRAME_PARAM_DESCRIPTOR);

    // Register input-related parameters
    auto use_imu = node_common::parameters::declare_parameter_and_get(*this, USE_IMU_PARAM_DESCRIPTOR);
    auto use_odom = node_common::parameters::declare_parameter_and_get(*this, USE_ODOM_PARAM_DESCRIPTOR);
    auto use_vel = node_common::parameters::declare_parameter_and_get(*this, USE_VEL_PARAM_DESCRIPTOR);
    auto use_cloud_input = node_common::parameters::declare_parameter_and_get(*this, USE_CLOUD_INPUT_PARAM_DESCRIPTOR);
    auto stamped_vel = node_common::parameters::declare_parameter_and_get(*this, STAMPED_VEL_PARAM_DESCRIPTOR);
    // Register output-related parameters
    auto publish_tf = node_common::parameters::declare_parameter_and_get(*this, PUBLISH_TF_PARAM_DESCRIPTOR);
    auto publish_pose = node_common::parameters::declare_parameter_and_get(*this, PUBLISH_POSE_PARAM_DESCRIPTOR);
    auto publish_pose_stamped = node_common::parameters::declare_parameter_and_get(*this, PUBLISH_POSE_STAMPED_PARAM_DESCRIPTOR);
    auto publish_pose_with_covariance = node_common::parameters::declare_parameter_and_get(*this, PUBLISH_POSE_WITH_COVARIANCE_PARAM_DESCRIPTOR);
    auto publish_pose_with_covariance_stamped = node_common::parameters::declare_parameter_and_get(*this, PUBLISH_POSE_WITH_COVARIANCE_STAMPED_PARAM_DESCRIPTOR);
    auto publish_odom = node_common::parameters::declare_parameter_and_get(*this, PUBLISH_ODOM_PARAM_DESCRIPTOR);
    auto position_covariance = node_common::parameters::declare_parameter_and_get(*this, POSITION_COVARIANCE_PARAM_DESCRIPTOR);
    auto orientation_covariance = node_common::parameters::declare_parameter_and_get(*this, ORIENTATION_COVARIANCE_PARAM_DESCRIPTOR);

    // Register input related parameters
    auto range_min = node_common::parameters::declare_parameter_and_get(*this, RANGE_MIN_PARAM_DESCRIPTOR);
    auto range_max = node_common::parameters::declare_parameter_and_get(*this, RANGE_MAX_PARAM_DESCRIPTOR);

    // Register cloud-input related parameters
    auto cloud_res = node_common::parameters::declare_parameter_and_get(*this, CLOUD_RES_PARAM_DESCRIPTOR);
    auto cloud_accept_nan = node_common::parameters::declare_parameter_and_get(*this, CLOUD_ACCEPT_NAN_PARAM_DESCRIPTOR);
    auto cloud_sorted = node_common::parameters::declare_parameter_and_get(*this, CLOUD_SORTED_PARAM_DESCRIPTOR);
    
    // Register KF related parameters
    auto kf_dist_linear = node_common::parameters::declare_parameter_and_get(*this, KF_DIST_LINEAR_PARAM_DESCRIPTOR);
    auto kf_dist_angular = node_common::parameters::declare_parameter_and_get(*this, KF_DIST_ANGULAR_PARAM_DESCRIPTOR);

    // Register scann matcher parameters
    auto max_iterations = node_common::parameters::declare_parameter_and_get(*this, MAX_ITERATIONS_PARAM_DESCRIPTOR);
    auto max_correspondence_dist = node_common::parameters::declare_parameter_and_get(*this, MAX_CORRESPONDENCE_DIST_PARAM_DESCRIPTOR);
    auto max_angular_correction_deg = node_common::parameters::declare_parameter_and_get(*this, MAX_ANGULAR_CORRECTION_DEG_PARAM_DESCRIPTOR);
    auto max_linear_correction = node_common::parameters::declare_parameter_and_get(*this, MAX_LINEAR_CORRECTION_PARAM_DESCRIPTOR);
    auto epsilon_xy = node_common::parameters::declare_parameter_and_get(*this, EPSILON_XY_PARAM_DESCRIPTOR);
    auto epsilon_theta = node_common::parameters::declare_parameter_and_get(*this, EPSILON_THETA_PARAM_DESCRIPTOR);
    auto outliers_max_prc = node_common::parameters::declare_parameter_and_get(*this, OUTLIERS_MAX_PRC_PARAM_DESCRIPTOR);
    // Register scann matcher parameters (advanced)
    auto sigma = node_common::parameters::declare_parameter_and_get(*this, SIGMA_PARAM_DESCRIPTOR);
    auto use_corr_tricks = node_common::parameters::declare_parameter_and_get(*this, USE_CORR_TRICKS_PARAM_DESCRIPTOR);
    auto restart = node_common::parameters::declare_parameter_and_get(*this, RESTART_PARAM_DESCRIPTOR);
    auto restart_threshold_mean_error = node_common::parameters::declare_parameter_and_get(*this, RESTART_THRESHOLD_MEAN_ERROR_PARAM_DESCRIPTOR);
    auto restart_dt = node_common::parameters::declare_parameter_and_get(*this, RESTART_DT_PARAM_DESCRIPTOR);
    auto restart_dtheta = node_common::parameters::declare_parameter_and_get(*this, RESTART_DTHETA_PARAM_DESCRIPTOR);
    auto clustering_threshold = node_common::parameters::declare_parameter_and_get(*this, CLUSTERING_THRESHOLD_PARAM_DESCRIPTOR);
    auto orientation_neighbourhood = node_common::parameters::declare_parameter_and_get(*this, ORIENTATION_NEIGHBOURHOOD_PARAM_DESCRIPTOR);
    auto use_point_to_line_distance = node_common::parameters::declare_parameter_and_get(*this, USE_POINT_TO_LINE_DISTANCE_PARAM_DESCRIPTOR);
    auto do_alpha_test = node_common::parameters::declare_parameter_and_get(*this, DO_ALPHA_TEST_PARAM_DESCRIPTOR);
    auto do_alpha_test_threshold_deg = node_common::parameters::declare_parameter_and_get(*this, DO_ALPHA_TEST_THRESHOLD_DEG_PARAM_DESCRIPTOR);
    auto outliers_adaptive_order = node_common::parameters::declare_parameter_and_get(*this, OUTLIERS_ADAPTIVE_ORDER_PARAM_DESCRIPTOR);
    auto outliers_adaptive_mul = node_common::parameters::declare_parameter_and_get(*this, OUTLIERS_ADAPTIVE_MUL_PARAM_DESCRIPTOR);
    auto do_visibility_test = node_common::parameters::declare_parameter_and_get(*this, DO_VISIBILITY_TEST_PARAM_DESCRIPTOR);
    auto outliers_remove_doubles = node_common::parameters::declare_parameter_and_get(*this, OUTLIERS_REMOVE_DOUBLES_PARAM_DESCRIPTOR);
    auto do_compute_covariance = node_common::parameters::declare_parameter_and_get(*this, DO_COMPUTE_COVARIANCE_PARAM_DESCRIPTOR);
    auto debug_verify_trick = node_common::parameters::declare_parameter_and_get(*this, DEBUG_VERIFY_TRICK_PARAM_DESCRIPTOR);
    auto use_ml_weights = node_common::parameters::declare_parameter_and_get(*this, USE_ML_WEIGHTS_PARAM_DESCRIPTOR);
    auto use_sigma_weights = node_common::parameters::declare_parameter_and_get(*this, USE_SIGMA_WEIGHTS_PARAM_DESCRIPTOR);
    auto debug_csm = node_common::parameters::declare_parameter_and_get(*this, DEBUG_CSM_PARAM_DESCRIPTOR);
    
    /* ------------------------- Validate parameters ----------------------- */

    // Check whether base frame has been given
    if(base_frame->empty())
        rclcpp::exceptions::InvalidParametersException("'base_frame' cannot be empty");
    // Check whether reference frame has been given
    if(reference_frame->empty())
        rclcpp::exceptions::InvalidParametersException("'reference_frame' cannot be empty");
    // Check whether target frame has been given
    if(target_frame->empty())
        rclcpp::exceptions::InvalidParametersException("'target_frame' cannot be empty");

    // Check if valid range restriction has been given
    if(*range_min < 0.0)
        rclcpp::exceptions::InvalidParametersException("'range_min' range cannot be negative");
    if(*range_max < 0.0)
        rclcpp::exceptions::InvalidParametersException("'range_max' range cannot be negative");

    // Check if valid cloud resolution has been given
    if(*cloud_res < 0.0)
        rclcpp::exceptions::InvalidParametersException("'cloud_res' resolution cannot be negative");

    // Check if valid kf linear distancehas been given
    if(*kf_dist_linear < 0.0)
        rclcpp::exceptions::InvalidParametersException("'kf_dist_linear' distance cannot be negative");

    // Check if valid kf angular distancehas been given
    if(*kf_dist_angular < 0.0)
        rclcpp::exceptions::InvalidParametersException("'kf_dist_angular' distance cannot be negative");

    // Helper macro checking whether a covariance matrix diagonal elements are valid
    auto covariance_matrix_valid = [](const auto &diagonal, const std::size_t &required_elems) {

        // Required matching size
        bool covariance_size_valid = (diagonal->size() == required_elems);
        // Required non-negative elements
        bool covariance_values_valid = not std::any_of(
            diagonal->begin(),
            diagonal->end(),
            [](const auto &elem) { return elem < 0.0; }
        ); 

        return covariance_size_valid && covariance_values_valid;
    };
    
    // Check if valid position covariance matrix has been given
    if(not covariance_matrix_valid(position_covariance, POSITION_COVARIANCE_MATRIX_DIAGONAL_ELEMENTS))
        rclcpp::exceptions::InvalidParametersException("'position_covariance' is invalid");

    // Check if valid otientation covariance matrix has been given
    if(not covariance_matrix_valid(orientation_covariance, ORIENTATION_COVARIANCE_MATRIX_DIAGONAL_ELEMENTS))
        rclcpp::exceptions::InvalidParametersException("'orientation_covariance' is invalid");
    
    // Check if number of max iterations has been given
    if(*max_iterations <= 0)
        rclcpp::exceptions::InvalidParametersException("'max_iterations' shall be positive");

    // Check if valid max correspondence distance has been given
    if(*max_correspondence_dist <= 0)
        rclcpp::exceptions::InvalidParametersException("'max_correspondence_dist' shall be positive");

    // Check if valid max angular correction distance has been given
    if(*max_angular_correction_deg <= 0)
        rclcpp::exceptions::InvalidParametersException("'max_angular_correction_deg' shall be positive");
        
    // Check if valid max linear correction distance has been given
    if(*max_linear_correction <= 0)
        rclcpp::exceptions::InvalidParametersException("'max_linear_correction' shall be positive");

    // Check if valid epsilon XY distance has been given
    if(*epsilon_xy <= 0)
        rclcpp::exceptions::InvalidParametersException("'epsilon_xy' shall be positive");

    // Check if valid epsilon angular distance has been given
    if(*epsilon_theta <= 0)
        rclcpp::exceptions::InvalidParametersException("'epsilon_theta' shall be positive");

    // Check if valid max outliers distance has been given
    if(*outliers_max_prc <= 0)
        rclcpp::exceptions::InvalidParametersException("'outliers_max_prc' shall be positive");

    // Check if valid 'sigma' has been given
    if(*sigma <= 0)
        rclcpp::exceptions::InvalidParametersException("'sigma' shall be positive");

    // Check if valid 'restart_threshold_mean_error' has been given
    if(*restart_threshold_mean_error <= 0)
        rclcpp::exceptions::InvalidParametersException("'restart_threshold_mean_error' shall be positive");

    // Check if valid 'restart_dt' has been given
    if(*restart_dt <= 0)
        rclcpp::exceptions::InvalidParametersException("'restart_dt' shall be positive");

    // Check if valid 'restart_dtheta' has been given
    if(*restart_dtheta <= 0)
        rclcpp::exceptions::InvalidParametersException("'restart_dtheta' shall be positive");

    // Check if valid 'clustering_threshold' has been given
    if(*clustering_threshold <= 0)
        rclcpp::exceptions::InvalidParametersException("'clustering_threshold' shall be positive");

    // Check if valid 'orientation_neighbourhood' has been given
    if(*orientation_neighbourhood <= 0)
        rclcpp::exceptions::InvalidParametersException("'orientation_neighbourhood' shall be positive");

    // Check if valid 'do_alpha_test_threshold_deg' has been given
    if(*do_alpha_test_threshold_deg <= 0)
        rclcpp::exceptions::InvalidParametersException("'do_alpha_test_threshold_deg' shall be positive");

    // Check if valid 'outliers_adaptive_order' has been given
    if(*outliers_adaptive_order <= 0)
        rclcpp::exceptions::InvalidParametersException("'outliers_adaptive_order' shall be positive");

    // Check if valid 'outliers_adaptive_mul' has been given
    if(*outliers_adaptive_mul <= 0)
        rclcpp::exceptions::InvalidParametersException("'outliers_adaptive_mul' shall be positive");

    /* ---------------- Copy parameters to the node's memory --------------- */
    
    // Parse TF-related parameters
    this->base_frame      = *base_frame;
    this->reference_frame = *reference_frame;
    this->target_frame    = *target_frame;
    
    // Parse input-related parameters
    this->use_cloud_input = *use_cloud_input;
    this->use_imu         = *use_imu;
    this->use_odom        = *use_odom;
    this->use_vel         = *use_vel;
    this->stamped_vel     = *stamped_vel;
    
    // Parse output-related parameters    
    this->publish_tf                           = *publish_tf;
    this->publish_pose                         = *publish_pose;
    this->publish_pose_stamped                 = *publish_pose_stamped;
    this->publish_pose_with_covariance         = *publish_pose_with_covariance;
    this->publish_pose_with_covariance_stamped = *publish_pose_with_covariance_stamped;
    this->publish_odom                         = *publish_odom;
    // Parse initial pose variances
    this->pose_variance.x     = position_covariance->at(0);
    this->pose_variance.y     = position_covariance->at(1);
    this->pose_variance.z     = position_covariance->at(2);
    this->pose_variance.roll  = orientation_covariance->at(0);
    this->pose_variance.pitch = orientation_covariance->at(1);
    this->pose_variance.yaw   = orientation_covariance->at(2);
    
    // Parse input related parameters
    input.min_reading = *range_min;
    input.max_reading = *range_max;
    
    // Parse cloud-input related parameters
    this->cloud_min_res_m   = *cloud_res;
    this->cloud_accept_nan  = *cloud_accept_nan;
    this->cloud_sorted      = *cloud_sorted;
    
    // Parse KF related parameters    
    this->kf_dist_linear_m    = *kf_dist_linear;
    this->kf_dist_angular_rad = *kf_dist_angular;
    
    // Parse scann matcher parameters
    input.max_iterations             = *max_iterations;
    input.max_correspondence_dist    = *max_correspondence_dist;
    input.max_angular_correction_deg = *max_angular_correction_deg;
    input.max_linear_correction      = *max_linear_correction;
    input.epsilon_xy                 = *epsilon_xy;
    input.epsilon_theta              = *epsilon_theta;
    input.outliers_maxPerc           = *outliers_max_prc;
    
    // Parse scann matcher parameters (advanced)
    input.sigma                        = *sigma;
    input.use_corr_tricks              = *use_corr_tricks;
    input.restart                      = *restart;
    input.restart_threshold_mean_error = *restart_threshold_mean_error;
    input.restart_dt                   = *restart_dt;
    input.restart_dtheta               = *restart_dtheta;
    input.clustering_threshold         = *clustering_threshold;
    input.orientation_neighbourhood    = *orientation_neighbourhood;
    input.use_point_to_line_distance   = *use_point_to_line_distance;
    input.do_alpha_test                = *do_alpha_test;
    input.do_alpha_test_thresholdDeg   = *do_alpha_test_threshold_deg;
    input.outliers_adaptive_order      = *outliers_adaptive_order;
    input.outliers_adaptive_mult       = *outliers_adaptive_mul;
    input.do_visibility_test           = *do_visibility_test;
    input.outliers_remove_doubles      = *outliers_remove_doubles;
    input.do_compute_covariance        = *do_compute_covariance;
    input.debug_verify_tricks          = *debug_verify_trick;
    input.use_ml_weights               = *use_ml_weights;
    input.use_sigma_weights            = *use_sigma_weights;
    
    // Enable/disable debug mode
    if(*debug_csm) 
        sm_debug_write(1);
}

/* ===================================================== Topics initialization ==================================================== */

void LaserScanMatcher::initialize_topics() {

    /* ------------------ Register subscribing interfaces ------------------ */

    // Initialize laser-scans input topic
    if(not use_cloud_input) {
        *node_common::communication::make_subscriber_builder(laser_sub)
            .node(*this)
            .name(LASER_SUB_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE)
            .callback(*this, &LaserScanMatcher::laser_callback);
    } else {
        *node_common::communication::make_subscriber_builder(cloud_sub)
            .node(*this)
            .name(CLOUD_SUB_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE)
            .callback(*this, &LaserScanMatcher::cloud_callback);   
    }

    // Initialize IMU-measurements input topic
    if(use_imu) {
        *node_common::communication::make_subscriber_builder(imu_sub)
            .node(*this)
            .name(IMU_SUB_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE)
            .callback(*this, &LaserScanMatcher::imu_callback);
    }

    // Initialize odometry-data input topic
    if(use_odom) {
        *node_common::communication::make_subscriber_builder(odom_sub)
            .node(*this)
            .name(ODOM_SUB_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE)
            .callback(*this, &LaserScanMatcher::odom_callback);
    }

    // Initialize odometry-data input topic
    if(use_vel) {
        if(stamped_vel) {
            *node_common::communication::make_subscriber_builder(velocity_sub)
                .node(*this)
                .name(VELOCITY_SUB_TOPIC_NAME)
                .qos(TOPIC_QUEUE_SIZE)
                .callback(*this, &LaserScanMatcher::velocity_callback);
        } else {
            *node_common::communication::make_subscriber_builder(velocity_stamped_sub)
                .node(*this)
                .name(VELOCITY_STAMPED_SUB_TOPIC_NAME)
                .qos(TOPIC_QUEUE_SIZE)
                .callback(*this, &LaserScanMatcher::velocity_stamped_callback);
        }
    }
    
    /* ------------------- Register publishing interfaces ------------------ */

    // Initialize pose output topic
    if(publish_pose) {
        *node_common::communication::make_publisher_builder(pose_pub)
            .node(*this)
            .name(POSE_PUB_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE);
    }

    // Initialize pose stamped output topic
    if(publish_pose_stamped) {
        *node_common::communication::make_publisher_builder(pose_stamped_pub)
            .node(*this)
            .name(POSE_STAMPED_PUB_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE);
    }

    // Initialize pose with covariance output topic
    if(publish_pose_with_covariance) {
        *node_common::communication::make_publisher_builder(pose_with_covariance_pub)
            .node(*this)
            .name(POSE_WITH_COVARIANCE_PUB_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE);
    }

    // Initialize pose with covariance stamped output topic
    if(publish_pose_with_covariance_stamped) {
        *node_common::communication::make_publisher_builder(pose_with_covariance_stamped_pub)
            .node(*this)
            .name(POSE_WITH_COVARIANCE_STAMPED_PUB_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE);
    }

    // Initialize odom output topic
    if(publish_odom) {
        *node_common::communication::make_publisher_builder(odom_pub)
            .node(*this)
            .name(ODOM_PUB_TOPIC_NAME)
            .qos(TOPIC_QUEUE_SIZE);
    }

    /* -------------------- Register service interfaces -------------------- */

    // Initialize set-pose service topic
    *node_common::communication::make_service_builder(set_pose_srv)
        .node(*this)
        .name(SET_POSE_SRV_TOPIC_NAME)
        .callback(*this, &LaserScanMatcher::set_pose_callback);

    /* ----------------------- Register TF interfaces ---------------------- */

    // Initialize TF2 broadcaster
    if(publish_tf) {
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    
    // Prepare TF2 buffer for the TF2 transform-listener interface
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // Create TF2 transform-listener interface with the given buffer
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

/* =============================================== Algorithm-specific initialization ============================================== */

void LaserScanMatcher::initialize_algorithm() {

    // Reset pose of the scan reference with respect to robot
    std::fill_n(input.laser, std::size(input.laser), 0.0);
    // Reset component relate to covariance computation
    output.cov_x_m = 0;
    output.dx_dy1_m = 0;
    output.dx_dy2_m = 0;

}

/* ================================================================================================================================ */

} // End namespace scan_tools

/* ======================================================== Nodes' registry ======================================================= */

RCLCPP_COMPONENTS_REGISTER_NODE(scan_tools::LaserScanMatcher)

/* ================================================================================================================================ */

