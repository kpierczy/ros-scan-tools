/* ============================================================================================================================ *//**
 * @file       laser_scan_matcher_helpers.cpp
 * @author     Ivan Dryanovski (ccnyroboticslab@gmail.com)
 * @author     William Morris
 * @author     Andrea Censi
 * @author     Carlos (cjaramillo@gc.cuny.edu)
 * @author     Isaac I.Y. Saito (130s@2000.jukuin.keio.ac.jp)
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 30th March 2022 9:30:14 am
 * @modified   Wednesday, 25th May 2022 11:11:53 pm
 * @project    scan-tools
 * @brief      Definitions of the helper methods of ROS2 node class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <algorithm>
#include <cmath>
// ROS includes
#include "geometry_msgs/msg/transform_stamped.hpp"
// TF2 includes
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// External includes
#include <range/v3/core.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view/zip.hpp>
// Private includes
#include "scan_tools/laser_scan_matcher.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* ========================================================= Local helpers ======================================================== */

/**
 * @brief Helper functor getting N'th element from the tuple-like object
 * @tparam N 
 *    index of the element to be referred
 */
template <std::size_t N>
struct get_n {

    template <typename T>
    auto operator()(T&& t) const ->
        decltype(std::get<N>(std::forward<T>(t))) {
        return std::get<N>(std::forward<T>(t));
    }
    
};

/* ============================================================ Helpers =========================================================== */

bool LaserScanMatcher::get_base_laser_transform(const std::string &laser_frame_id) {

    // Get current time
    auto now = this->get_clock()->now();

    geometry_msgs::msg::TransformStamped laser_to_base_tf;
    
    // Try to acquire laser-reference -> robot-base transform
    try {

        static constexpr double TRANSFORM_WAITING_TIMEOUT_S = 1.0;

        // Wait for the transformation to be available
        laser_to_base_tf = tf_buffer->lookupTransform(
            base_frame,
            laser_frame_id,
            now,
            rclcpp::Duration::from_seconds( TRANSFORM_WAITING_TIMEOUT_S )
        );

    // If failed to acquire the transformation, return
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get initial transform from '%s' to '%s' frame, (%s)",
            laser_frame_id.c_str(), base_frame.c_str(), ex.what());
        return false;
    }

    // Convert transform to the TF-specific type
    tf2::fromMsg(laser_to_base_tf.transform, laser_to_base);
    // Cache inverse transformation
    base_to_laser = laser_to_base.inverse();

    return true;

}


std::optional<LDP> LaserScanMatcher::laser_to_ldp(const sensor_msgs::msg::LaserScan &msg) {

    // Calculate numer of points scanned of the laser
    auto scans_num = msg.ranges.size();
    // If too few scans present, return immediatelly
    if(scans_num < 2U)
        return std::optional<LDP>{};

    // Allocate a new LDP structure
    LDP ret = ld_alloc_new(static_cast<int>(scans_num));

    // Iterate over scanned points and convert measurement to set of polar-coordinates-described points
    for (std::size_t i = 0; i < scans_num; ++i) {

        // Get distance to the scanned points in laser reference frame
        double range = msg.ranges[i];

        // If measurement in acceptable range is given, add it to the output structure
        if (range > msg.range_min && range < msg.range_max) {
            
            // Cache range measurement
            ret->valid[i] = true;
            ret->readings[i] = range;
            // Calculate angle of the point in the polar coordinate system
            ret->theta[i] = msg.angle_min + i * msg.angle_increment;

        // Otherwise, add invalid point to the output structure
        } else {

            ret->valid[i] = false;
            ret->readings[i] = -1;
            
        }

        // (Copied from the original code) [?]
        ret->cluster[i]  = -1;
        
    }

    // Write down limits of angular range of the scan
    ret->min_theta = (msg.angle_increment > 0) ? ret->theta[0]             : ret->theta[scans_num - 1];
    ret->max_theta = (msg.angle_increment > 0) ? ret->theta[scans_num - 1] : ret->theta[0];

    constexpr auto POSE_COMPONENTS_NUM = 3;

    // Clear odometry estimation of the robot's position
    std::fill_n(ret->odometry, POSE_COMPONENTS_NUM, 0.0);
    // Clear estimation of the robot's position
    std::fill_n(ret->true_pose, POSE_COMPONENTS_NUM, 0.0);

    return ret;
}


std::optional<LDP> LaserScanMatcher::cloud_to_ldp(const sensor_msgs::msg::PointCloud2 &msg) {

    // Prepare PCL-specific cloud representation of the incoming message
    CloudType original_cloud;
    // Convert incoming message into the PCL cloud
    message_to_cloud(msg, original_cloud);

    // Calculate numer of points scanned
    auto scans_num = original_cloud.points.size();
    // If too few scans present, return immediatelly
    if(scans_num < 2U)
        return std::optional<LDP>{};

    // Prepare PCL cloud for result of filtering of the incoming cloud
    CloudType cloud;
    // Always push the first point into the (copied from the original code, but why?)
    cloud.points.push_back(original_cloud.points[0]);

    // Compute squeare of minimal cloud resolution to save on multiplications
    double min_cloud_resolution_squared = cloud_min_res_m * cloud_min_res_m;
    // Filter incoming cloud removing too densly arranged points 
    for (std::size_t i = 1; i < original_cloud.points.size(); ++i) {

        // Get reference to the last filtered point
        const PointType& last_point = cloud.points[cloud.points.size() - 1];
        // Get reference to the point to bt filtered
        const PointType& point_to_filter = original_cloud.points[i];

        // Compute distance in the X and in the Y direction
        double dx = last_point.x - point_to_filter.x;
        double dy = point_to_filter.y - point_to_filter.y;

        // If points are far enough, add them to the resulting cloud
        if( dx*dx + dy*dy > min_cloud_resolution_squared)
            cloud.points.push_back(point_to_filter);

    }
    
    // Calculate number of filtered points
    auto points_num = cloud.points.size();
    // If too few points left after filtering, return
    if(points_num < 2U)
        return std::optional<LDP>{};

    // Allocate new CSM scan data structure
    LDP ret = ld_alloc_new(static_cast<int>(points_num));

    // Iterate over points and convert them to the LDP structure
    for (std::size_t i = 0; i < points_num; i++) {

        // Calculate position of the  in laser frame
        if (not std::isnan(cloud.points[i].x) and not std::isnan(cloud.points[i].y)) {
            
            // Calculate range to the point in the polar coordinates system
            double range = std::sqrt( cloud.points[i].x * cloud.points[i].x + cloud.points[i].y * cloud.points[i].y );

            // If valid point has been given, parse it into the resulting structure
            if (range > input.min_reading && range <= input.max_reading) {
                
                // Cache range measurement
                ret->valid[i] = true;
                ret->readings[i] = range;
                // Calculate angle of the point in the polar coordinate system
                ret->theta[i] = atan2(cloud.points[i].y, cloud.points[i].x);
            
            // Else insert an invalid points
            } else {

                ret->valid[i] = false;
                ret->readings[i] = -1;

            }

        // If invalid point has been given, print warning
        } else  {

            // Insert invalid point
            ret->valid[i] = false;
            ret->readings[i] = -1;
            // Log warning, if configured
            if(not cloud_accept_nan)
                RCLCPP_WARN_STREAM(this->get_logger(), "Cloud input contains NaN values. Please use a filtered cloud input.");
                
        }

        // (Copied from the original code) [?]
        ret->cluster[i] = -1;
    }

    // Sort input data with respect to increasing theta angle
    if(not cloud_sorted) {
        
        // Zip arrays to be sorted
        auto zipped = ranges::view::zip(
            ranges::subrange(ret->valid,    ret->valid    + points_num),
            ranges::subrange(ret->readings, ret->readings + points_num),
            ranges::subrange(ret->theta,    ret->theta    + points_num)
        );

        // Sort arrays
        ranges::sort(
            zipped,               // Arrays to be sorted
            std::less<double>{ }, // Sort in theta-non-descending order
            get_n<2>{ }           // Sort according to the 'theta' vector
        );
        
    }

    // Write down limits of angular range of the scan
    ret->min_theta = ret->theta[0];
    ret->max_theta = ret->theta[points_num - 1];

    constexpr auto POSE_COMPONENTS_NUM = 3;

    // Clear odometry estimation of the robot's position
    std::fill_n(ret->odometry, POSE_COMPONENTS_NUM, 0.0);
    // Clear estimation of the robot's position
    std::fill_n(ret->true_pose, POSE_COMPONENTS_NUM, 0.0);

    return ret;
}


tf2::Transform LaserScanMatcher::pose_2d_to_tf(const geometry_msgs::msg::Pose2D &pose) {

    tf2::Transform ret;

    // Set translation
    ret.setOrigin(tf2::Vector3{ pose.x, pose.y, 0.0 });

    tf2::Quaternion q;

    // Calculate rotation
    q.setRPY(0.0, 0.0, pose.theta);
    // Set rotation
    ret.setRotation(q);

    return ret;
}


tf2::Transform LaserScanMatcher::odom_to_tf(const nav_msgs::msg::Odometry &odom) {

    tf2::Transform ret;

    // Set translation
    ret.setOrigin(tf2::Vector3{ odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0 });

    tf2::Quaternion q;

    // Calculate rotation
    q.setRPY(0.0, 0.0, tf2::getYaw(odom.pose.pose.orientation));
    // Set rotation
    ret.setRotation(q);

    return ret;
}

/* ================================================================================================================================ */

} // End namespace scan_tools
