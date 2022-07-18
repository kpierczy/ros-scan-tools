/* ============================================================================================================================ *//**
 * @file       laser_scan_matcher_callbacks.cpp
 * @author     Ivan Dryanovski (ccnyroboticslab@gmail.com)
 * @author     William Morris
 * @author     Andrea Censi
 * @author     Carlos (cjaramillo@gc.cuny.edu)
 * @author     Isaac I.Y. Saito (130s@2000.jukuin.keio.ac.jp)
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 30th March 2022 9:30:14 am
 * @modified   Wednesday, 25th May 2022 11:11:37 pm
 * @project    scan-tools
 * @brief      Definitions of the callback methods of ROS2 node class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <algorithm>
// TF2 includes
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// Private includes
#include "scan_tools/laser_scan_matcher.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* =========================================================== Callbacks ========================================================== */

void LaserScanMatcher::set_pose_callback(
    const scan_tools_msgs::srv::SetPose::Request::SharedPtr req,
    scan_tools_msgs::srv::SetPose::Response::SharedPtr res
) {

    /* ---------------------------- Acquire TF transform ----------------------------- */

    geometry_msgs::msg::TransformStamped pose_guess_tf;
    
    // Try to acquire reference->incoming-reference frame transform
    try {

        static constexpr double TRANSFORM_WAITING_TIMEOUT_S = 1.0;

        // Wait for the transformation to be available
        pose_guess_tf = tf_buffer->lookupTransform(
            reference_frame,
            req->pose.header.frame_id,
            req->pose.header.stamp,
            rclcpp::Duration::from_seconds( TRANSFORM_WAITING_TIMEOUT_S )
        );

    // If failed to acquire the transformation, return
    } catch (tf2::TransformException &ex) {

        // Log warning
        RCLCPP_WARN(this->get_logger(), "Could not get transform from '%s' to '%s' frame, (%s)",
            req->pose.header.frame_id.c_str(), reference_frame.c_str(), ex.what());
        RCLCPP_WARN(this->get_logger(), " --> discarding pose guess");
        // Fill response with failure
        res->success = false;
        res->reason = "Failed to transform incoming pose into the 'reference' frame";
        
        return;
    }

    /* --------------------------- Reset pose estimation ----------------------------- */

    // Mark node as unitialized
    first_scan_received = false;

    // Mark cached messages as non-arrived
    last_imu_msg.reset();
    last_odom_msg.reset();
    last_velocity_msg.reset();

    // Deallocate last scan data
    if(last_scan != nullptr) {
        ld_free(last_scan);
        last_scan = nullptr;
    }
    
    // Convert incoming pose to the current keyframe
    tf2::fromMsg(pose_guess_tf.transform, keyframe_to_reference);
    // Set current pose estimation to keyframe pose estimation
    base_to_reference = keyframe_to_reference;
    
    // Reinitialize algorithm-specific data
    initialize_algorithm();

    RCLCPP_INFO(this->get_logger(), "Current pose guess has been reset");

    /* --------------------------- Publish current guess ----------------------------- */

    // Publish estimation
    publish_estimation(req->pose.header.stamp);

    /* ------------------------------------------------------------------------------- */

    // Fill response with success
    res->success = true;

}

template<typename T>
void LaserScanMatcher::scan_callback(const T &msg) {

    static_assert(std::is_same_v<T, sensor_msgs::msg::LaserScan> || std::is_same_v<T, sensor_msgs::msg::PointCloud2>,
        "[LaserScanMatcher::scan_callback] Invalid T type has been given");

    // Get message-converting method depending on the template parameter
    auto convert_msg = [this](const T &m) {
        if constexpr(std::is_same_v<T, sensor_msgs::msg::LaserScan>)
            return laser_to_ldp(m);
        else
            return cloud_to_ldp(m);
    };

    std::optional<LDP> ldp_scan;

    // If the first scan arrived
    if(not first_scan_received) {

        // For laser scan cache limits of the angular range of incoming scans
        if constexpr(std::is_same_v<T, sensor_msgs::msg::LaserScan>) {

            /**
             * @note In the original 'laser_scan_matcher' implementation, on the first
             *    incoming laser scan, set of sinuses and cosinuses of all scanned points
             *    has beeen cached in dedicated std::vectors. This information has been
             *    not used wanywhere else in the code and so has been removed.
             */

            input.min_reading = msg.range_min;
            input.max_reading = msg.range_max;
        }

        // Get transformation between scan's reference frame and robot's base
        if(not get_base_laser_transform(msg.header.frame_id)) {
            RCLCPP_INFO_STREAM(this->get_logger(), 
                "Skipping incoming scan with '" << msg.header.frame_id << "' reference frame (no baser->laser transform available)");
            return;
        }

        // Initialize 'previous scan' with the current one
        if(ldp_scan = convert_msg(msg); ldp_scan.has_value()) {
            
            // Initialize 'previous scan' with the current one
            last_scan = *ldp_scan;
            // Initialize timestamp of the last scan
            last_scan_stamp = msg.header.stamp;
            // Mark first scan as received
            first_scan_received = true;
            
        }

    // Else, just parse incoming cloud into the LDP structure
    } else
        ldp_scan = convert_msg(msg);

    // Process incoming scan
    if(ldp_scan.has_value())
        process_scan(msg.header.stamp, *ldp_scan);
    // If too few points has been scanned, skip processing
    else
        RCLCPP_INFO_STREAM(this->get_logger(), "Skipping incoming scan with '" << msg.header.frame_id << "' reference frame (too few points)");
}

void LaserScanMatcher::laser_callback(const sensor_msgs::msg::LaserScan &msg) {
    scan_callback(msg);
}


void LaserScanMatcher::cloud_callback(const sensor_msgs::msg::PointCloud2 &msg) {
    scan_callback(msg);
}


void LaserScanMatcher::imu_callback(const sensor_msgs::msg::Imu &msg) {

    // Cache incoming message
    last_imu_msg.msg = msg;
    // If the first message arrives, initialize 'last used' message
    if(not last_imu_msg.received) {
        last_imu_msg.received = true;
        last_imu_msg_used = msg;
    }

}


void LaserScanMatcher::odom_callback(const nav_msgs::msg::Odometry &msg) {

    // Cache incoming message
    last_odom_msg.msg = msg;
    // If the first message arrives, initialize 'last used' message
    if(not last_odom_msg.received) {
        last_odom_msg.received = true;
        last_odom_msg_used = msg;
    }

}


void LaserScanMatcher::velocity_callback(const geometry_msgs::msg::Twist &msg) {

    // Cache incoming message
    last_velocity_msg.msg = msg;
    // Mark message as received
    last_velocity_msg.received = true;

}


void LaserScanMatcher::velocity_stamped_callback(const geometry_msgs::msg::TwistStamped &msg) {
    velocity_callback(msg.twist);
}

/* ================================================================================================================================ */

} // End namespace scan_tools
