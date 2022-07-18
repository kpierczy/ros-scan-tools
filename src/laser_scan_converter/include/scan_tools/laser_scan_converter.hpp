/* ============================================================================================================================ *//**
 * @file       laser_scan_converter.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 11:07:59 pm
 * @project    scan-tools
 * @brief      Declaration of the ROS2 node class converting sensor_msgs::msg::LaserScan data into the PCL-specific PointCloud
 *             representation
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SCAN_TOOLS_LASER_SCAN_CONVERTER_H__
#define __SCAN_TOOLS_LASER_SCAN_CONVERTER_H__

/* =========================================================== Includes =========================================================== */

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// Message includes
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// Private includes
#include "node_common/communication.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* ============================================================= Node ============================================================= */

/**
 * @brief ROS2 node class converting sensor_msgs::msg::LaserScan data into the PCL-specific PointCloud
 *    representation
 */
class RCLCPP_PUBLIC LaserScanConverter: public rclcpp::Node {

public: /* ------------------------------------------------- Topics's parameters -------------------------------------------------- */

    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 1000;

    /// Name of the input laser scan topic
    static constexpr auto SCAN_SUB_TOPIC_NAME = "scan";
    
    /// Name of the output point cloud topic 
    static constexpr auto CLOUD_PUB_TOPIC_NAME = "cloud";

public: /* ------------------------------------------------- Public ctors & dtors ------------------------------------------------- */

    /**
     * @brief Construct a new Laser Scan Converter object
     * 
     * @param options 
     *    configuration of the node
     */
    LaserScanConverter(const rclcpp::NodeOptions & options);

    /**
     * @brief Destroy the Laser Scan Converter object logging goodbye message 
     *    to the rosout
     */
    ~LaserScanConverter();

private: /* ------------------------------------------------- Point clouds typing ------------------------------------------------- */

    /// Point type used when interacting with PCL library
    using PointType = pcl::PointXYZI;
    /// Cloud type used when interacting with PCL library
    using CloudType = pcl::PointCloud<PointType>;

    /// Alias for cloud-to-message conversion function
    static inline void cloud_to_message(const CloudType &cloud, sensor_msgs::msg::PointCloud2 &cloud_msg) {
        pcl::toROSMsg(cloud, cloud_msg);
    }
    
private: /* ------------------------------------------------------ Callbacks ------------------------------------------------------ */

    /**
     * @brief Callback for the incoming slaser scan messages
     * @param scan_msg 
     *    incoming message
     */
    void scan_callback(const sensor_msgs::msg::LaserScan &scan_msg);

private: /* ---------------------------------------------------- ROS interfaces --------------------------------------------------- */

	/// Subscriber interface used to acquire input laser scans
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
	/// Publisher interface used to provide output point cloud
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
    
};

/* ================================================================================================================================ */

} // End namespace scan_tools

#endif
