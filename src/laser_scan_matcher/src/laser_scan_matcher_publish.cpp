/* ============================================================================================================================ *//**
 * @file       laser_scan_matcher_init.cpp
 * @author     Ivan Dryanovski (ccnyroboticslab@gmail.com)
 * @author     William Morris
 * @author     Andrea Censi
 * @author     Carlos (cjaramillo@gc.cuny.edu)
 * @author     Isaac I.Y. Saito (130s@2000.jukuin.keio.ac.jp)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 23rd March 2022 10:09:09 pm
 * @modified   Wednesday, 25th May 2022 11:12:27 pm
 * @project    scan-tools
 * @brief      Definitions of the publising methods of ROS2 node class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <algorithm>
#include <cmath>
// TF2 includes
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// Private includes
#include "scan_tools/laser_scan_matcher.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* ========================================================== Definitions ========================================================= */

void LaserScanMatcher::publish_estimation(const rclcpp::Time &stamp) {

    // Publish geometry_msgs::msg::Pose2D output if required
    if (publish_pose) {
        
        geometry_msgs::msg::Pose2D msg;

        // Fill the message
        msg.x = base_to_reference.getOrigin().x();
        msg.y = base_to_reference.getOrigin().y();
        msg.theta = tf2::getYaw(base_to_reference.getRotation());
        // Publish the message
        pose_pub->publish(msg);

    }

    // Publish geometry_msgs::msg::PoseStamped output if required
    if (publish_pose_stamped) {

        geometry_msgs::msg::PoseStamped msg;

        // Fill message's header
        msg.header.stamp    = stamp;
        msg.header.frame_id = reference_frame;
        // Fill message's body
        tf2::toMsg(base_to_reference, msg.pose);
        // Publish the message
        pose_stamped_pub->publish(msg);
        
    }

    // Number of components in the pose vector
    constexpr std::size_t POSE_COMPONENTS_NUM = 6;

    // Helper lambda constructing covariance matrix (represented as std::vector)
    auto construct_cov_matrix = [this](
        double x_variance,
        double y_variance,
        double theta_variance
    ) -> std::array<double, POSE_COMPONENTS_NUM * POSE_COMPONENTS_NUM> {
        return {
            x_variance,        0.0,             0.0,                0.0,                 0.0,            0.0,
                   0.0, y_variance,             0.0,                0.0,                 0.0,            0.0,
                   0.0,        0.0, pose_variance.z,                0.0,                 0.0,            0.0,
                   0.0,        0.0,             0.0, pose_variance.roll,                 0.0,            0.0,
                   0.0,        0.0,             0.0,                0.0, pose_variance.pitch,            0.0,
                   0.0,        0.0,             0.0,                0.0,                 0.0, theta_variance,
        };
    };

    // Publish geometry_msgs::msg::PoseWithCovariance output if required
    if (publish_pose_with_covariance) {
        
        geometry_msgs::msg::PoseWithCovariance msg;

        // Fill message's pose
        tf2::toMsg(base_to_reference, msg.pose);
        // Compute covariance, if requested
        if (input.do_compute_covariance) {
            msg.covariance = construct_cov_matrix(
                gsl_matrix_get(output.cov_x_m, 0, 0),
                gsl_matrix_get(output.cov_x_m, 0, 1),
                gsl_matrix_get(output.cov_x_m, 0, 2)
            );
        // Otherwise use initial covariance
        } else {
            msg.covariance = construct_cov_matrix(
                pose_variance.x,
                pose_variance.y,
                pose_variance.yaw
            );
        }

        // Publish the message
        pose_with_covariance_pub->publish(msg);
        
    }

    // Publish geometry_msgs::msg::PoseWithCovarianceStamped output if required
    if (publish_pose_with_covariance_stamped) {
        
        geometry_msgs::msg::PoseWithCovarianceStamped msg;

        // Fill message's header
        msg.header.stamp    = stamp;
        msg.header.frame_id = reference_frame;
        // Fill message's pose
        tf2::toMsg(base_to_reference, msg.pose.pose);
        // Compute covariance, if requested
        if (input.do_compute_covariance) {
            msg.pose.covariance = construct_cov_matrix(
                gsl_matrix_get(output.cov_x_m, 0, 0),
                gsl_matrix_get(output.cov_x_m, 0, 1),
                gsl_matrix_get(output.cov_x_m, 0, 2)
            );
        // Otherwise use initial covariance
        } else {
            msg.pose.covariance = construct_cov_matrix(
                pose_variance.x,
                pose_variance.y,
                pose_variance.yaw
            );
        }
        
        // Publish the message
        pose_with_covariance_stamped_pub->publish(msg);
    }

    // Publish nav_msgs::msg::Odometry output if required
    if (publish_odom) {
        
        nav_msgs::msg::Odometry msg;

        // Fill message's header
        msg.header.stamp    = stamp;
        msg.header.frame_id = reference_frame;
        msg.child_frame_id  = base_frame;
        
        // Fill message's pose
        tf2::toMsg(base_to_reference, msg.pose.pose);
        // Compute covariance, if requested
        if (input.do_compute_covariance) {
            msg.pose.covariance = construct_cov_matrix(
                gsl_matrix_get(output.cov_x_m, 0, 0),
                gsl_matrix_get(output.cov_x_m, 0, 1),
                gsl_matrix_get(output.cov_x_m, 0, 2)
            );
        // Otherwise use initial covariance
        } else {
            msg.pose.covariance = construct_cov_matrix(
                pose_variance.x,
                pose_variance.y,
                pose_variance.yaw
            );
        }
        
        // Compute transformation between current and previous estimation
        tf2::Transform current_estimation_to_previous = base_to_reference.inverse() * last_base_to_reference;
        // Compute time difference
        auto dt = stamp - last_scan_stamp;

        // Compute current twist assuming constant-speed model (linear part)
        msg.twist.twist.linear.x = current_estimation_to_previous.getOrigin().x() / dt.seconds();
        msg.twist.twist.linear.y = current_estimation_to_previous.getOrigin().y() / dt.seconds();
        msg.twist.twist.linear.z = current_estimation_to_previous.getOrigin().z() / dt.seconds();
        // Compute current twist assuming constant-speed model (angular part)
        msg.twist.twist.angular.z = tf2::getYaw(current_estimation_to_previous.getRotation()) / dt.seconds();

        // Publish the message
        odom_pub->publish(msg);
    }

    // Publish TF transformation if required
    if (publish_tf) {

        geometry_msgs::msg::TransformStamped transform;

        // Fill transformation's header
        transform.header.stamp    = stamp;
        transform.header.frame_id = reference_frame;
        // Fill transformation's body
        transform.child_frame_id = target_frame;
        transform.transform = tf2::toMsg(base_to_reference);
        // Publish transformation
        tf_broadcaster->sendTransform(transform);
        
    }
}

/* ================================================================================================================================ */

} // End namespace scan_tools
