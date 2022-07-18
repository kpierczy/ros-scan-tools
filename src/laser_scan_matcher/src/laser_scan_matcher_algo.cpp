/* ============================================================================================================================ *//**
 * @file       laser_scan_matcher_algo.cpp
 * @author     Ivan Dryanovski (ccnyroboticslab@gmail.com)
 * @author     William Morris
 * @author     Andrea Censi
 * @author     Carlos (cjaramillo@gc.cuny.edu)
 * @author     Isaac I.Y. Saito (130s@2000.jukuin.keio.ac.jp)
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 30th March 2022 9:30:14 am
 * @modified   Wednesday, 25th May 2022 11:11:27 pm
 * @project    scan-tools
 * @brief      Definitions of the core algorithm methods of ROS2 node class
 * 
 * @see http://wiki.ros.org/laser_scan_matcher to learn about implementation details like e.g. 'keyframe'
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <algorithm>
#include <cmath>
// TF2 includes
#include "tf2/utils.h"
// Private includes
#include "scan_tools/laser_scan_matcher.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* ============================================================ Helpers =========================================================== */

tf2::Transform LaserScanMatcher::get_pose_change_prediction(const rclcpp::Duration &dt) {

    constexpr double PI = M_PI;

    // By default assume that no component of the robot pose has changed (zero-motion model)
    tf2::Transform pose_change { tf2::Transform::getIdentity() };

    // If odometry data is used
    if(use_odom and last_odom_msg.received) {

        // Convert it into the current TF transformation [ base -> refernce ]
        tf2::Transform current_base_to_reference_tf = odom_to_tf(last_odom_msg.msg);
        // Convert last odometry data used into the previous TF transformation [ base -> refernce ]
        tf2::Transform previous_base_to_reference_tf = odom_to_tf(last_odom_msg_used);
        
        // Calculate transformations chain [ current-base -> reference -> previous-base ]
        pose_change = previous_base_to_reference_tf.inverse() * current_base_to_reference_tf;
        // Cache last odometry data being used for prediction
        last_odom_msg_used = last_odom_msg.msg;

    // Else, if robot's 'velocity hint' is used
    } else if(use_vel and last_velocity_msg.received) {
        
        geometry_msgs::msg::Pose2D pose_change_2d;

        // Calculate estimated change using linear extrapolation
        pose_change_2d.x     = dt.seconds() * last_velocity_msg.msg.linear.x;
        pose_change_2d.y     = dt.seconds() * last_velocity_msg.msg.linear.y;
        pose_change_2d.theta = dt.seconds() * last_velocity_msg.msg.angular.z;
        // Limit orientation change in range [-180; 180] [deg]
        pose_change_2d.theta = std::remainder(pose_change_2d.theta, 2 * PI);
        // Convert estimated change into the transformation
        pose_change = pose_2d_to_tf(pose_change_2d);

    }

    // If IMU measurements are used
    if(use_imu and last_imu_msg.received) {

        geometry_msgs::msg::Pose2D pose_change_2d;

        // Parse translation change if calculated with previous stepd
        pose_change_2d.x = pose_change.getOrigin().x();
        pose_change_2d.y = pose_change.getOrigin().y();
        // Calculate the orientation change from incoming IMU data messages
        pose_change_2d.theta = tf2::getYaw(last_imu_msg.msg.orientation) - tf2::getYaw(last_imu_msg_used.orientation);

        // Limit orientation change in range [-180; 180] [deg]
        pose_change_2d.theta = std::remainder(pose_change_2d.theta, 2 * PI);  
        // Convert estimated change into the transformation
        pose_change = pose_2d_to_tf(pose_change_2d);   
        // Cache last IMU data being used for prediction
        last_imu_msg_used = last_imu_msg.msg;

    }

    return pose_change;
}


bool LaserScanMatcher::is_new_key_frame_needed(const tf2::Transform& pose_correction) {

    // If change in orientation is large enough, consider new key frame required
    if( std::fabs(tf2::getYaw(pose_correction.getRotation())) > kf_dist_angular_rad )
        return true;

    // Calculate change in the robot's position
    double dx = pose_correction.getOrigin().getX();
    double dy = pose_correction.getOrigin().getY();

    // If change in position is large enough, consider new key frame required
    if( dx * dx + dy * dy > kf_dist_linear_m * kf_dist_linear_m)
        return true;

    return false;
}

/* ======================================================== Core algorithm ======================================================== */

void LaserScanMatcher::process_scan(const rclcpp::Time &stamp, LDP &current_scan) {
    
    // Get start stamp of the processing routine (for debug purposes)
    auto start = this->get_clock()->now();

    /**
     * @brief CSM library is used in the following way:
     * 
     *     1) The scans are always in the laser reference frame
     *     2) The reference scan (last_scan) has a pose of [0, 0, 0]
     *     3) The new scan (current_scan) has a pose equal to the movement
     *     4) of the laser in the laser frame since the last scan
     *     5) The computed correction is then propagated using the tf machinery
     * 
     */

    /* ----- Calculate initial pose-correction guess based on other data sources ----- */

    // Estimate change in the robot's pose basing on auxiliary data
    tf2::Transform pose_change_transform = get_pose_change_prediction(stamp - last_scan_stamp);

    /**
     * @note The transformation provided in the original `laser_scan_matcher` package frm `scan_tools`
     *    has the invalid transformations of the position-correction guess. These however are invalid.
     *    Current implementation that fixed this problem has been taken from [1]. The reasoning behind
     *    the implementation is following:
     * 
     *      1. We need estimated transformation from the new laser reference frame to old laser reference 
     *         frame which is (in case of keyframe-based scans matching approach) the last keyframed 
     *         laser reference (i.e. laser frame of reference assumed the robot was in the 'keyframe' 
     *         pose) [(new) laser -> (keyframe) laser]
     *      2. We have estimation of change of the robot's pose in the frame of refernce being an old
     *         pose of the robot's base [(new) base -> (old) base] as
     *      3. Using constant transformations between laser and base we can calculate desired transformation
     *         with the following transformations chain
     *  
     *            - (new) laser -> (new) base       [constant laser->base transform]
     *            - (new) base  -> (old) base       [predicted change]
     *            - (old) base  -> reference        [last estimation]
     *            - reference   -> keyframe         [last keyframe]
     *            - keyframe    -> (keyframe) laser [constant base->laser transform]
     *            = (new) laser -> (keyframe) laser 
     * 
     * @note Original implemetation of the @ref get_pose_change_prediction(...) method provided invalid results 
     *    by calculating difference betwen last known and estimate pose using coordinates in the @b 'reference'
     *    frame of reference
     * @note [1] uses the same name convention for ransformaitons as the original `laser_scan_matcher` 
     *    implementation which is the opposite of this implementation. In this implementation @b X_to_Y 
     *    transformation means that multiplying vector given in the X coordinate system by the transformation
     *    will result in coordinates of the given vector in the Y coordinate system. In the [1] implementation
     *    this means that the @a X_to_Y has been created with (x,y,θ) coordinates being a coordinate of the Y frame
     *    of reference in the X frame of reference
     * 
     * @see [1] github.com/EwingKang/scan_tools
     */

    // Transform the predicted change of the laser's position into the laser frame
    pose_change_transform = base_to_laser * (keyframe_to_reference.inverse() * base_to_reference) * pose_change_transform * laser_to_base;

    /* ------------------------- Prepare CSM data structures ------------------------- */

    constexpr auto POSE_COMPONENTS_NUM = 3;
    
    // Reset last odometry-base estimation of the pose
    std::fill_n(last_scan->odometry, POSE_COMPONENTS_NUM, 0.0);
    // Reset last estimation of the pose
    std::fill_n(last_scan->estimate, POSE_COMPONENTS_NUM, 0.0);
    // Reset last pose
    std::fill_n(last_scan->true_pose, POSE_COMPONENTS_NUM, 0.0);

    // Pass previous and current scan to CSM
    input.laser_ref  = last_scan;
    input.laser_sens = current_scan;

    // Pass initial 'educated' guess to CSM
    input.first_guess[0] = pose_change_transform.getOrigin().getX();
    input.first_guess[1] = pose_change_transform.getOrigin().getY();
    input.first_guess[2] = tf2::getYaw(pose_change_transform.getRotation());

    // If they are non-Null, free covariance gsl matrices to avoid leaking memory
    auto clear_cov_matrix = [](gsl_matrix *matrix) {
        if (matrix) {
            gsl_matrix_free(matrix);
            matrix = nullptr;
        };
    };

    // Free actual matrices
    clear_cov_matrix(output.cov_x_m);
    clear_cov_matrix(output.dx_dy1_m);
    clear_cov_matrix(output.dx_dy2_m);

    /* ------------------------------- Scan match frames ----------------------------- */

    // Perform scan matching using point to line icp from CSM
    sm_icp(&input, &output);
    
    /* ---------------------------------- Parse outpus ------------------------------- */

    // Prepare container for correction of robot's base pose (in the reference frame)
    tf2::Transform pose_correction;

    // If algorithm succeeded
    if (output.valid) {

        geometry_msgs::msg::Pose2D algo_output;

        /**
         * Parse output of the algorithm (i.e. [dx, dy, dθ] vector indicating change in the 
         * lasser's pose given in previouse reference frame of the laser)
         */
        algo_output.x     = output.x[0];
        algo_output.y     = output.x[1];
        algo_output.theta = output.x[2];
        
        /**
         * @note The transformation provided in the original `laser_scan_matcher` package from `scan_tools`
         *    has the following transformations of the position-correction guess. The pose-correction is given
         *    by the ICP in the laser's frame and refers to the change of pose of the scans' reference frame 
         *    (usually centre of laser). The original implementation, however, treat the correction as if it
         *    was correction of the robot's base pose and calculates resulting change of laser's pose by
         *    back-and-forth-like <i>base_to_laser * laser_pose_correction * laser_to_base</i> transformation
         *    which is given:
         * 
         *       - from the old pose of the laser
         *       - to the new pose of the laser
         *       - in the old pose of the laser as a frame of refernce (FoR)
         *       - assuming that the ICP result refers to robot's base pose change in it's old pose FoR
         * 
         *    This is invalid behaviour. In fact, one needs the transformation describing situation the other
         *    way around. We need to deduce transformation:
         * 
         *       - from the old pose of the robot's base
         *       - to the new pose of the robot's base
         *       - in the old pose of the robot's base as a frame of refernce (FoR)
         *       - assuming that the ICP result refers to laser's pose change in it's old pose FoR
         * 
         *    This can eb echieved by the following chain of transformations:
         * 
         *       * (new) base  to (new) laser [constant transformation]
         *       * (new) laser to (old) laser [correction]
         *       * (old) laser to (old) base  [constant transformation]
         *       = (new) base  to (old) base  
         * 
         * @code [old code]
         * 
         *    // Calculate the correction of laser pose (in the laser reference frame)
         *    tf2::Transform laser_pose_correction = pose_2d_to_tf(algo_output);
         *    // Transform the correction of laser pose (in the laser reference frame) into the robot base's position correction (in the base frame)
         *    pose_correction = base_to_laser * laser_pose_correction * laser_to_base;
         *    // Update the robot pose in the reference frame
         *    base_to_reference = keyframe_to_reference * pose_correction;
         * 
         * @endcode 
         * @see [1] github.com/EwingKang/scan_tools
         */

        // Calculate correction of laser pose (in the reference frame being old pose of the laser)
        tf2::Transform laser_pose_correction = pose_2d_to_tf(algo_output);
        // Calculate correction of robot's base pose (in the reference frame being old pose of the robot's base) [essential odometry information]
        pose_correction = laser_to_base * laser_pose_correction * base_to_laser;
        // Update the robot pose in the reference frame based on the odometrical correction
        base_to_reference = keyframe_to_reference * pose_correction;

        RCLCPP_DEBUG(this->get_logger(), 
            "Correcting current pose by [ %.4f, %.4f, %.4f ] (position) [ %.4f, %.4f, %.4f, %.4f ] (orientation)",
            pose_correction.getOrigin().x(),
            pose_correction.getOrigin().y(),
            pose_correction.getOrigin().z(),
            pose_correction.getRotation().w(),
            pose_correction.getRotation().x(),
            pose_correction.getRotation().y(),
            pose_correction.getRotation().z()
        );

        // Publish results
        publish_estimation(stamp);
        // Update 'previous' estimation
        last_base_to_reference = base_to_reference;
        
    // If algorithm failed to match frames assume no correction in the pose
    } else {

        // Log warning
        RCLCPP_WARN_STREAM(this->get_logger(), "Error in scan matching");
        
        // If this is the first iteration of the algorithm and the process fails, mark node for reinitialization at the next scan
        if(last_scan == current_scan) {

            RCLCPP_WARN_STREAM(this->get_logger(), "  --> failure on first attempt - reinitializing ICP...");

            // Free memory of the scan
            ld_free(last_scan);
            // Reset algorithm state
            last_scan = nullptr;
            first_scan_received = false;
            // Return immediatelly 
            return;

        // Else, just free current scan and return
        } else {
            ld_free(current_scan);
            return;
        }

    }
    
    /* ------------------------------------ Cleanup ---------------------------------- */

    // Update keyframe scan if required
    if (is_new_key_frame_needed(pose_correction)) {

        RCLCPP_DEBUG(this->get_logger(), "Updating keyframe...");

        // If this is NOT the first pose correction computed
        if(last_scan != current_scan) {

            // Free memory of the old key frame
            ld_free(last_scan);
            // Update 'last_scan' data
            last_scan = current_scan;
            
        }
        
        // Update ref->kf transformation
        keyframe_to_reference = base_to_reference;

    // Otherwise, if this is NOT the first iteration of the algorithm, deallocate current scan
    } else if(last_scan != current_scan)
        ld_free(current_scan);

    // Update timestamp of the last iteration
    last_scan_stamp = stamp;

    // Print some debug statistics
    RCLCPP_DEBUG_STREAM(this->get_logger(),
        "Scan matcher total duration: " << (this->get_clock()->now() - start).seconds() << " [s]");
}

/* ================================================================================================================================ */

} // End namespace scan_tools
