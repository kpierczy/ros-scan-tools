/* ============================================================================================================================ *//**
 * @file       laser_scan_matcher.hpp
 * @author     Ivan Dryanovski (ccnyroboticslab@gmail.com)
 * @author     William Morris
 * @author     Andrea Censi
 * @author     Carlos (cjaramillo@gc.cuny.edu)
 * @author     Isaac I.Y. Saito (130s@2000.jukuin.keio.ac.jp)
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 30th March 2022 9:30:14 am
 * @modified   Wednesday, 25th May 2022 11:10:36 pm
 * @project    scan-tools
 * @brief      Declaration of the ROS2 node class providing laser-scan-based odometry
 * 
 * 
 * @note This package uses Canonical Scan Matcher [1], written by Andrea Censi
 * @note This package has been based on the original `laser_scan_matcher` package written by Ivan Dryanovski and William Morris
 * @see [1] A. Censi, "An ICP variant using a point-to-line metric"  Proceedings of the IEEE International Conference on Robotics
 *    and Automation (ICRA), 2008
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SCAN_TOOLS_LASER_SCAN_MATCHER_H__
#define __SCAN_TOOLS_LASER_SCAN_MATCHER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <optional>
#include <vector>
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// Message includes
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "scan_tools_msgs/srv/set_pose.hpp"
// TF includes
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
// PCL includes
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
// CSM includes
#include "csm/csm.h"
// Private includes
#include "node_common/communication.hpp"
#include "scan_tools/laser_scan_matcher_params.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* ============================================================= Node ============================================================= */

/**
 * @brief ROS2 node class providing laser-scan-based odometry
 */
class RCLCPP_PUBLIC LaserScanMatcher: public rclcpp::Node {
    
public: /* -------------------------------------------- Parameters of provided topcis --------------------------------------------- */

    // Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 50;

    /**
     * @note The "velocity" topic is expected to provide robot's twist in the
     *    robot's local frame of reference (i.e. one referred by the 'base_frame'
     *    parameter)
     */

    /// Name of the input laser topic
    static constexpr auto LASER_SUB_TOPIC_NAME = "scan";
    /// Name of the input cloud topic
    static constexpr auto CLOUD_SUB_TOPIC_NAME = "cloud";
    /// Name of the input IMU topic
    static constexpr auto IMU_SUB_TOPIC_NAME = "imu";
    /// Name of the input odometry topic
    static constexpr auto ODOM_SUB_TOPIC_NAME = "odom";
    /// Name of the input velocity topic
    static constexpr auto VELOCITY_SUB_TOPIC_NAME = "velocity";
    /// Name of the input velocity stamped topic
    static constexpr auto VELOCITY_STAMPED_SUB_TOPIC_NAME = "velocity_stamped";

    /// Name of the output pose topic
    static constexpr auto POSE_PUB_TOPIC_NAME = "odom/laser/pose";
    /// Name of the output pose stamped topic
    static constexpr auto POSE_STAMPED_PUB_TOPIC_NAME = "odom/laser/pose_stamped";
    /// Name of the output pose with covariance topic
    static constexpr auto POSE_WITH_COVARIANCE_PUB_TOPIC_NAME = "odom/laser/pose_with_covariance";
    /// Name of the output pose with covariance stamped topic
    static constexpr auto POSE_WITH_COVARIANCE_STAMPED_PUB_TOPIC_NAME = "odom/laser/pose_with_covariance_stamped";
    /// Name of the output odometry topic
    static constexpr auto ODOM_PUB_TOPIC_NAME = "odom/laser";

    /// Name of the service pose topic setting current pose estimation to the given value
    static constexpr auto SET_POSE_SRV_TOPIC_NAME = "set_pose";

public: /* ---------------------------------------------------- Ctors & dtors ----------------------------------------------------- */

    /**
     * @brief Construct a new LaserScanMatcher object
     * 
     * @param options 
     *    configuration of the node
     */
    LaserScanMatcher(const rclcpp::NodeOptions & options);

    /**
     * @brief Destroy the LaserScanMatcher object logging goodbye message 
     *    to the rosout
     */
    ~LaserScanMatcher();

private: /* ------------------------------------------------- Initializing methods ------------------------------------------------ */

    /**
     * @brief Declares and parses parameters of the `laser_scan_matcher` node
     */
    void initialize_params();

    /**
     * @brief Initializes topic interfaces of the `laser_scan_matcher` node
     */
    void initialize_topics();

    /**
     * @brief Initializes algorithm-specific data structured
     */
    void initialize_algorithm();

private: /* ------------------------------------------------ Topic callbacks methods ---------------------------------------------- */
    
    /**
     * @brief Callback template for for the incoming scan messages
     * @param msg
     *    incoming laser scan message (either LaserScan or PointCloud2)
     */
    template<typename T>
    void scan_callback(const T &msg);

    /**
     * @brief Callback for the incoming laser scan messages
     * @param msg
     *    incoming laser scan message
     */
    void laser_callback(const sensor_msgs::msg::LaserScan &msg);

    /**
     * @brief Callback for the incoming point cloud messages
     * @param msg
     *    incoming message
     */
    void cloud_callback(const sensor_msgs::msg::PointCloud2 &msg);

    /**
     * @brief Callback for the incoming IMU measurements messages
     * @param msg
     *    incoming IMU measurements message
     */
    void imu_callback(const sensor_msgs::msg::Imu &msg);

    /**
     * @brief Callback for the incoming odometry messages
     * @param msg
     *    incoming odometry message
     */
    void odom_callback(const nav_msgs::msg::Odometry &msg);

    /**
     * @brief Callback for the incoming velocity messages
     * @param msg
     *    incoming velocity message
     */
    void velocity_callback(const geometry_msgs::msg::Twist &msg);

    /**
     * @brief Callback for the incoming velocity stamped messages
     * @param msg
     *    incoming velocity stamped message
     */
    void velocity_stamped_callback(const geometry_msgs::msg::TwistStamped &msg);


    /**
     * @brief Callback for the incoming pose set request
     * @param msg
     *    incoming pose to be set
     */
    void set_pose_callback(
        const scan_tools_msgs::srv::SetPose::Request::SharedPtr req,
        scan_tools_msgs::srv::SetPose::Response::SharedPtr res
    );

private: /* ---------------------------------------------------- Helper methods --------------------------------------------------- */

    /**
     * @brief tries to read TF transformations between robot's base frame and 
     *    the reference frame of incoming scan
     * 
     * @param laser_frame_id 
     *    name of the TF reference frame of incoming scans
     * @returns 
     *   @retval @c true on success
     *   @retval @c false on failure
     */
    bool get_base_laser_transform(const std::string &laser_frame_id);

    /**
     * @brief Converts laser sacan message into the LDP scan structure
     * 
     * @param msg 
     *    message to be converted
     * @returns 
     *    cloud converted to LDP structure
     */
    static std::optional<LDP> laser_to_ldp(const sensor_msgs::msg::LaserScan &msg);

    /**
     * @brief Converts point cloud message into the LDP scan structure
     * 
     * @param msg 
     *    message to be converted
     * @returns 
     *    cloud converted to LDP structure
     */
    std::optional<LDP> cloud_to_ldp(const sensor_msgs::msg::PointCloud2 &msg);

    /**
     * @brief Transforms 2D pose to the TF transformation
     * 
     * @param pose 
     *    pose to be transformed
     * @returns 
     *    TF transformation from @p pose reference frame to some fixed frame 
     */
    static tf2::Transform pose_2d_to_tf(const geometry_msgs::msg::Pose2D &pose);

    /**
     * @brief Transforms odometry message to the TF transformation from 'base' frame
     *    to the 'reference' frame
     * 
     * @param odom 
     *    odometry message to be transformed
     * @returns 
     *    TF transformation from @p odom reference frame to the 'reference' reference frame
     */
    static tf2::Transform odom_to_tf(const nav_msgs::msg::Odometry &odom);

private: /* ------------------------------------------------ Algotihm-core methods ------------------------------------------------ */

    /**
     * @brief Returns the predicted change in pose based on the external sensorical data. The return
     *    transform is a transformation from the current 'base' frame of reference to the 'base' frame
     *    of reference between the predicted change
     * 
     * @param dt 
     *    duration from the timestamp of the last incoming scan to the currently
     *    incoming scan
     * 
     * @returns 
     *    predicted pose of the robot
     */
    tf2::Transform get_pose_change_prediction(const rclcpp::Duration &dt);

    /**
     * @brief Checks whether a new keyframe (i.e. so called 'previous' frame used for pose matching
     *    with the new frame) is required (i.e. whether it should be replaced with the current frame)
     *    based on the magnitude of estimated change in the robot's pose
     * 
     * @param pose_correction 
     *    estimated change in the robot pose
     * @returns 
     *    @retval @c true if correction is large anough to trigger keyframe change
     *    @retval @c false otherwise
     */
    bool is_new_key_frame_needed(const tf2::Transform& pose_correction);

    /**
     * @brief Processes incoming scan
     * 
     * @param stamp 
     *    timestamp of the incoming scan
     * @param current_scan 
     *    incoming scan
     * @returns 
     *    cloud converted to LDP structure
     */
    void process_scan(const rclcpp::Time &stamp, LDP &current_scan);

    /**
     * @brief Publishes current output of algorithm
     * 
     * @param stamp 
     *    timestamp of the incoming scan
     */
    void publish_estimation(const rclcpp::Time &stamp);

private: /* ------------------------------------------------ PCL-related components ----------------------------------------------- */

    /// Point type used when interacting with PCL library
    using PointType = pcl::PointXYZ;
    /// Cloud type used when interacting with PCL library
    using CloudType = pcl::PointCloud<PointType>;

    /// Alias for message-to-cloud conversion function
    static inline void message_to_cloud(const sensor_msgs::msg::PointCloud2 &cloud_msg, CloudType &cloud) {
        pcl::fromROSMsg(cloud_msg, cloud);
    }

    /// Alias for cloud-to-message conversion function
    static inline void cloud_to_message(const CloudType &cloud, sensor_msgs::msg::PointCloud2 &cloud_msg) {
        pcl::toROSMsg(cloud, cloud_msg);
    }

private: /* ------------------------------------------------ ROS-interface objects ------------------------------------------------ */

	/// Subscriber interfaces for incoming scans
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
	/// Subscriber interfaces for incoming clouds
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
	/// Subscriber interfaces for incoming IMU measurements
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
	/// Subscriber interfaces for incoming encoders-based odometry reading 
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	/// Subscriber interfaces for incoming velocity
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub;
	/// Subscriber interfaces for incoming velocity stamped
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_stamped_sub;

	/// Publisher interface used to broadcast estimated robot's pose
	rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub;
	/// Publisher interface used to broadcast estimated robot's pose
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub;
	/// Publisher interface used to broadcast estimated robot's pose
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovariance>::SharedPtr pose_with_covariance_pub;
	/// Publisher interface used to broadcast estimated robot's pose
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_covariance_stamped_pub;
	/// Publisher interface used to broadcast estimated robot's pose
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

	/// Service interface for requesting setting current pose estimation to the given value
	rclcpp::Service<scan_tools_msgs::srv::SetPose>::SharedPtr set_pose_srv;

    /// TF2 publishing object for broadcastign output frames
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    
    /// TF2 buffer utilitized by the @a transform_listener to buffer incoming frames
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    /// TF2 interface listening for required transformation frames
    std::shared_ptr<tf2_ros::TransformListener> transform_listener{ nullptr };

private: /* ---------------------------------------------------- Node parameters -------------------------------------------------- */

    /* ------ TF-related parameters ------ */

    /// Name of the reference frame for the estimated position
    std::string reference_frame;
    /// Name of the robot's base frame
    std::string base_frame;
    /// Name of the target frame representing estimated pose of the robot in the reference frame
    std::string target_frame;

    /* ------ Input-related parameters ------ */

    /// Boolean flag indicating whether points cloud input should be used instead of laser-scan input
    bool use_cloud_input;
    /// Boolean flag indicating whether points IMU input should be used
    bool use_imu;
    /// Boolean flag indicating whether points odometry input should be used
    bool use_odom;
    /// Boolean flag indicating whether points controls input should be used
    bool use_vel;
    /// Boolean flag indicating whether controls input is given as a stamped message
    bool stamped_vel;

    /* ------ Output-related parameters ------ */

    /// Boolean flag indicating whether estimated position should be broadcasted Pose2D message
    bool publish_pose;
    /// Boolean flag indicating whether estimated position should be broadcasted PoseStamped message
    bool publish_pose_stamped;
    /// Boolean flag indicating whether estimated position should be broadcasted PoseWithCoveriance message
    bool publish_pose_with_covariance;
    /// Boolean flag indicating whether estimated position should be broadcasted PoseWithCoverianceStamped message
    bool publish_pose_with_covariance_stamped;
    /// Boolean flag indicating whether estimated position should be broadcasted Odometry message
    bool publish_odom;
    /// Boolean flag indicating whether estimated position should be broadcasted as TF transformation 
    bool publish_tf;

    /// Array holding variances of pose vectors' elements ( @note these are diagonal elements of pose covariance matrix )
    struct {
        double x     { 0.0 };
        double y     { 0.0 };
        double z     { 0.0 };
        double roll  { 0.0 };
        double pitch { 0.0 };
        double yaw   { 0.0 };
    } pose_variance;

    /* ------ Cloud-input related parameters ------ */

    /// Minimal acceptable range to the point given in the input cloud
    double cloud_range_min_m;
    /// Maximal acceptable range to the point given in the input cloud
    double cloud_range_max_m;
    /// Maximal acceptable distance between distinguishable points in the points cloud in [m]
    double cloud_min_res_m;
    /// Flag indicating whether NaN values in the incoming clouds should be treated as erronous
    bool cloud_accept_nan;
    /// Flag indicating whether incoming clouds are properly sorted
    bool cloud_sorted;

    /* ------ KF related parameters ------ */

    /// Value indicating what distance the fixed frame needs to move before updating the keyframe scan (in [m])
    double kf_dist_linear_m;
    /// Value indicating what angle the fixed frame needs to move before updating the keyframe scan (in [rad])
    double kf_dist_angular_rad;

private: /* ----------------------------------------------- Algorithm-specific data ----------------------------------------------- */

    /// Boolean flag indicating whether the first laser scan/points cloud has been already received
    bool first_scan_received { false };

    // Transform from robot's base frame to the reference frame of incoming scan
    tf2::Transform base_to_laser;
    // Transform from the reference frame of incoming scan to robot's base frame
    tf2::Transform laser_to_base;

    /**
     * @brief Helper pair-like structure binding a message object with the boolean
     *    flag indicating whether a new message has been received since the last
     *    computation iteration
     */
    template<typename T>
    struct AuxiliaryMsg {
        
        /// Bufferred message
        T msg { rosidl_runtime_cpp::MessageInitialization::ZERO };
        /// Helper boolean flag indicating whether any message has arived yet
        bool received { false };

        // Helper reset action
        void reset() { *this = AuxiliaryMsg<T>{}; }

    };

    /// Buffer storing the last Imu message arrived
    AuxiliaryMsg<sensor_msgs::msg::Imu> last_imu_msg;
    /// Buffer storing the last Imu message used in computations
    sensor_msgs::msg::Imu last_imu_msg_used { rosidl_runtime_cpp::MessageInitialization::ZERO };
    /// Buffer storing the last Odometry message arrived
    AuxiliaryMsg<nav_msgs::msg::Odometry> last_odom_msg;
    /// Buffer storing the last Odometry message used in computations
    nav_msgs::msg::Odometry last_odom_msg_used { rosidl_runtime_cpp::MessageInitialization::ZERO };
    /// Buffer storing the last Twist message arrived
    AuxiliaryMsg<geometry_msgs::msg::Twist> last_velocity_msg;

    /// Tmestamp of the last processing iteration (time of the last scan)
    rclcpp::Time last_scan_stamp;
    /// CSM input parameters
    sm_params input;
    /// CSM result
    sm_result output;
    /// Previous arrived scan in form of the LDP structure
    LDP last_scan { nullptr };

    /// Transform from robot's base frame to the reference frame (previous estimation of robot's pose)
    tf2::Transform last_base_to_reference { tf2::Transform::getIdentity() };
    /// Transform from robot's base frame to the reference frame (current estimation of robot's pose)
    tf2::Transform base_to_reference { tf2::Transform::getIdentity() };

    /**
     * @brief Pose of the last keyframe scan in reference frame ( @see [1] to get familiar with the 
     *   'keyframe' concept)
     * @note [1] introduces 'keyframe' as a last validated, noise-proof transformation to the 'laser'
     *   reference frame (i.e. reference frame of incoming scans). The implementation, however, uses
     *   the keyframe being last validated, noise-proof transformation from the 'reference' frame 
     *   (usually 'odom' to the 'base' frame)
     * 
     * @see [1] http://wiki.ros.org/laser_scan_matcher
     */
    tf2::Transform keyframe_to_reference { tf2::Transform::getIdentity() };
    
};

/* ================================================================================================================================ */

} // End namespace scan_tools

#endif
