/* ============================================================================================================================ *//**
 * @file       point_cloud_assembler.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 23rd March 2022 10:09:09 pm
 * @modified   Wednesday, 25th May 2022 11:07:04 pm
 * @project    scan-tools
 * @brief      Declaration of the ROS2 node class assembling incoming PCL points clouds into a single points cloud defined in the
 *             common reference frame
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SCAN_TOOLS_POINT_CLOUD_ASSEMBLER_H__
#define __SCAN_TOOLS_POINT_CLOUD_ASSEMBLER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <optional>
#include <unordered_map>
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// Message includes
#include "sensor_msgs/msg/point_cloud2.hpp"
// TF includes
#include "tf2_ros/transform_listener.h"
// PCL includes
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
// Common includes
#include "node_common/parameters.hpp"
#include "node_common/communication.hpp"
// Private includes
#include "scan_tools/point_cloud_assembler/traits.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* ============================================================= Node ============================================================= */

/**
 * @brief ROS2 node class assembling incoming PCL points clouds into a single points 
 *    cloud defined in the common reference frame
 * 
 * @details The node's aim is to listen on the set of preconfigrued topics broadcasting
 *    messages of type @ref sensor_msgs::msg::PointCloud2. Incoming clouds are stored in
 *    the local memory of the node. When the 'triggering' cloud arrives (the 'triggering'
 *    condition depends on the configruation of node's params), stored clouds are concatenated
 *    and flushed to the output topic.
 * 
 *    The node support dynamic choice of the type of points stored by the processed clouds.
 *    Set of supported types if subset of point types supported by the PCL library.
 */
class RCLCPP_PUBLIC PointCloudAssembler: public rclcpp::Node {

public: /* -------------------------------------------------- Node's parameters --------------------------------------------------- */
    
    /// Size of the default values' array of the parameter defining input topics
    static constexpr std::size_t INPUT_TOPICS_PARAM_DEFAULT_ARRAY_SIZE = 1;
    /// Description of the parameter defining input topics
    static constexpr node_common::parameters::ParamDescriptor<
        std::vector<std::string>,
        INPUT_TOPICS_PARAM_DEFAULT_ARRAY_SIZE
    > INPUT_TOPICS_PARAM_DESCRIPTOR {
        .name           = "input_topics",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = std::array{ "cloud_in" },
        .description    = "Array of names of input topics."
    };
    
    /// Description of the parameter defining reference frame for the resulting cloud
    static constexpr node_common::parameters::ParamDescriptor<std::string> REFERENCE_FRAME_PARAM_DESCRIPTOR {
        .name           = "reference_frame",
        .read_only      = true,
        .dynamic_typing = false,
        .description    = "Reference frame of the assembled cloud"
    };
    
    /// Description of the parameter defining reference frames of clouds that should be included in the assembled cloud
    static constexpr node_common::parameters::ParamDescriptor<std::vector<std::string>> FRAMES_PARAM_DESCRIPTOR {
        .name           = "frames",
        .read_only      = true,
        .dynamic_typing = false,
        .description    = "Array of names of reference frames indicating those from incoming points clouds "
                          "that should be included in the assembled result. Empty or unset array indicates "
                          "that all incoming frames should be taken into account."
    };
    
    /// Description of the parameter defining assembly mode for the node
    static constexpr node_common::parameters::ParamDescriptor<std::string> COMBINING_MODE_PARAM_DESCRIPTOR {
        .name                   = "combining_mode",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "continuous",
        .description            = "Assembly mode for the node combining points clouds incoming from LIDAR scaners. 'batch' "
                                  "mode means, that the combined cloud will be published after at least one fresh update of "
                                  "measurement from each LIDAR has been received since the last publication. If 'continuous' "
                                  "mode is set, the combined cloud will be published at each of measurements arrival",
        .additional_constraints = "Either 'batch' or 'continuous'. 'batch' cannot be set if the 'frames' parameter " 
                                  "has been set empty" 
    };

    /// Description of the parameter defining decaying speed of incomping clouds' validity
    static constexpr node_common::parameters::ParamDescriptor<double> VALIDITY_DECAYING_TIME_PARAM_DESCRIPTOR {
        .name           = "validity_decaying_time",
        .read_only      = true,
        .dynamic_typing = false,
        .description    = "Decaying time of incomping clouds' validity in [s]. After the configured time "
                          "the last refresh of the given cloud (i.e. cloud associated with the given reference "
                          "frame) will be considered invalid and won't be taken into account when publishing "
                          "the next cloud assembly. Leaving this parameter unset will result in no validity timeout."
    };

    /// Description of the parameter defining type of points in the assembled clouds
    static constexpr node_common::parameters::ParamDescriptor<std::string> POINT_TYPE_PARAM_DESCRIPTOR {
        .name                   = "point_type",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "XYZ",
        .description            = "Type of the points stored in the processed clouds",
        .additional_constraints = "Either 'XYZ' or 'XYZL' or 'XYZI' (names correspond to PCL point types)"
    };

public: /* ------------------------------------------------- Topics's parameters -------------------------------------------------- */

    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 50;

    /// Name of the output point cloud topic 
    static constexpr auto CLOUD_PUB_TOPIC_NAME = "cloud_out";

private: /* ------------------------------------------------- Point clouds typing ------------------------------------------------- */

    /// Valid type of cloud points
    static constexpr auto VALID_POINT_TYPES = std::array{
        "XYZ",
        "XYZL",
        "XYZI"
    };
    
    /// Point types used when interacting with PCL library
    using SupportedPointTypes = std::tuple<
        pcl::PointXYZ,
        pcl::PointXYZL,
        pcl::PointXYZI 
    >;

    /**
     * @brief Helper constants indexing types of valid points
     */
    enum PointTypeId : std::size_t {
        XYZ  = traits::index_of<pcl::PointXYZ,  SupportedPointTypes>::value,
        XYZL = traits::index_of<pcl::PointXYZL, SupportedPointTypes>::value,
        XYZI = traits::index_of<pcl::PointXYZI, SupportedPointTypes>::value
    };

    /// Type of internal point cloud representation
    template<typename PointType>
    using CloudType = pcl::PointCloud<PointType>;

public: /* ------------------------------------------------- Public ctors & dtors ------------------------------------------------- */

    /**
     * @brief Construct a new Laser Scan Assembler object
     * 
     * @param options 
     *    configuration of the node
     */
    PointCloudAssembler(const rclcpp::NodeOptions & options);

    /**
     * @brief Destroy the Laser Scan Assembler object logging goodbye message 
     *    to the rosout
     */
    ~PointCloudAssembler();

private: /* ------------------------------------------------------ Callbacks ------------------------------------------------------ */

    /**
     * @brief Callback for the incoming point cloud messages
     * @param cloud_msg 
     *    incoming message
     */
    void cloud_callback(const sensor_msgs::msg::PointCloud2 &cloud_msg);
    
private: /* -------------------------------------------------- Auxiliary methods -------------------------------------------------- */

    /**
     * @brief Resets current storage object to store clouds corresponding to the new
     *    point type
     * @param new_point_type
     *    target point type
     */
    void reset_cloud_storage(PointTypeId new_point_type);

    /**
     * @brief Stores incoming point cloud in the local memory depending on the node's
     *    configuration and cloud's reference frame
     * 
     * @tparam PointType 
     *    type of the point stored by the incoming cloud
     * 
     * @param now 
     *    time of cloud's arrival
     * @param cloud_msg 
     *    pointer to the incoming cloud
     * 
     * @retval true
     *    if @p cloud_msg has been stored
     * @retval false
     *    if @p cloud_msg has been discarded
     * 
     * @todo Extend @ref CloudDescriptor and discard incomming cloud if it's timestamp
     *    (i.e. timestamp given by the publisher) is earlier than the timestamp of the 
     *    previous cloud received (which is saved in the extended filed). For now, the
     *    system is assummed to be run on a single machine and in consequence assumes
     *    cohesion between order of publishing and recining clouds. In general scenario
     *    it may be not the case
     */
    template<typename PointType>
    inline bool store_incoming_cloud(
        const rclcpp::Time &now,
        const sensor_msgs::msg::PointCloud2 &cloud_msg
    );

    /**
     * @brief Searches set of the stored clouds and gathers clouds that should be included in the
     *    compiled cloud
     * 
     * @tparam PointType 
     *    type of the point stored by the incoming cloud
     * 
     * @param now 
     *    time of the last cloud's arrival
     * @returns 
     *    set of pointers to clouds that should be included in the compiled cloud
     * 
     * @note In the 'batch' mode, if clouds are ready for being compiled, the method resets
     *    'freshness' flags of all registered clodus
     */
    template<typename PointType>
    inline std::vector<const CloudType<PointType>*> select_clouds_to_compile(const rclcpp::Time &now);

    /**
     * @brief Compiles the given set of clouds into a single points cloud and publishes it to the output topic
     * 
     * @tparam PointType 
     *    type of the point stored by the incoming cloud
     * 
     * @param now 
     *    time of the last cloud's arrival
     * @param cloud_components
     *     reference to the list of component clouds to be compiled
     */
    template<typename PointType>
    inline void compile_clouds(
        const rclcpp::Time &now,
        const std::vector<const CloudType<PointType>*> &cloud_components
    );

private: /* --------------------------------------------------- Auxiliary types --------------------------------------------------- */
    
    /**
     * @brief Numerical representation of the combining mode
     */
    enum class CombiningMode {

        /**
         * In the 'Continuous' mode the incoming clouds are identified by their @a frame_id 
         * parameter and stored in the assosiative container with the time of arrival kept.
         * After each cloud being received, the node iterates over saved clouds and combines
         * those which has been received in time no longer that the 'validity_decaying_time'
         * parameter.
         */
        Continuous,

        /**
         * In the 'SelectiveContinuous' mode (i.e. 'continuous' combining mode with the non-empty
         * 'frames' array configured) the incoming clouds are identified by their @a frame_id.
         * Clouds associated with the frames registered in the 'frame' parameter are saved in the
         * memory along with the time of arrival kept. After each registered cloud being received, 
         * the node iterates over saved clouds and combines those which has been received in time 
         * no longer that the 'validity_decaying_time' parameter.
         */
        SelectiveContinuous,

        /**
         * In the 'Batch' mode the incoming clouds are identified by their @a frame_id.
         * Clouds associated with the frames registered in the 'frame' parameter are saved in the
         * memory and their 'freshness' flag is set. After each registered cloud being received, 
         * the node iterates over saved clouds and checks whether all clouds has been 'refreshed'
         * since the last publication of the assembled cloud and withing the 'fresshness' decay
         * timerange. If so, the combined cloud is calculated and pulished and 'freshness' flags
         * of all registered clouds are reset.
         */
        Batch
    };

    /**
     * @brief Helper structure storing updated version of the incoming point cloud
     * @note Negative value of the @a refresh_stamp attribute is used to indicate that
     *    the cloud associated with the given reference frame has not been received yet.
     */
    template<typename PointType>
    struct CloudDescriptor {

        /// Type of the point for which the structure has been specialized for
        using PointT = PointType;

        /// Boolean flag indicating whether the cloud has been refreshed since some point in time
        bool refreshed { false };
        /// Timestamp of the last refresh
        rclcpp::Time refresh_stamp;
        /// Boolean flag indicating whether the cloud has been refreshed since some point in time
        CloudType<PointType> cloud { };
        
        /**
         * @brief Construct a new Cloud Descriptor object with the give initial refresh timestamp
         * 
         * @param refresh_timestamp 
         *    initial refresh timestamp of the descriptor
         */
        CloudDescriptor(const rclcpp::Time &refresh_timestamp);

    };

    /// Type used to store collection of clouds descriptors
    template<typename PointType>
    using CloudsCollection = std::unordered_map<std::string, CloudDescriptor<PointType>>;

    /**
     * @brief Helper template defining a specialization of std::variant storing on of
     *    specialization of @p Template template. These specializations are resolved
     *    for subsequent types of supported point types
     */
    template<template<typename> typename Template>
    using VariantBySupportedPointTypes = 
        typename traits::resolve_variant_by_tuple<Template, SupportedPointTypes>::type;

    /// Type used to actually store incoming clouds
    using CloudStorage =
        VariantBySupportedPointTypes<CloudsCollection>;

private: /* ----------------------------------------- Auxiliary traits and meta-functions ----------------------------------------- */

    /// Auxiliary meta-function returning type of the supported point by it's enumerated index
    template <PointTypeId Id>
    using point_by_id_t =
        typename std::tuple_element<static_cast<std::size_t>(Id), SupportedPointTypes>::type;

    /// Auxiliary meta-function returning cloud-collection corresponding to the type of the supported point by it's enumerated index
    template <PointTypeId Id>
    using cloud_collection_by_point_id_t =
        CloudsCollection<point_by_id_t<Id>>;

    /// Auxiliary meta-function returning type of the descriptor of the cloud point corresponding to the cloud-collection
    template <typename CloudsCollectionType>
    using cloud_descriptor_by_cloud_collection_t =
        typename std::remove_reference_t<std::remove_cv_t<CloudsCollectionType>>::mapped_type;

    /// Auxiliary meta-function returning type of the cloud point corresponding to the cloud-collection
    template <typename CloudsCollectionType>
    using point_by_cloud_collection_t =
        typename std::remove_reference_t<std::remove_cv_t<CloudsCollectionType>>::mapped_type::PointT;
    
private: /* ------------------------------------------------- Auxiliary functions ------------------------------------------------- */

    /**
     * @returns 
     *    point type id of the corresponding to the given string representation
     * 
     * @note Method assumes that @p point_type_string represents a supported type
     */
    static inline PointTypeId point_type_id_from_string(std::string point_type_string) {
        return static_cast<PointTypeId>(
            std::distance(VALID_POINT_TYPES.begin(), std::find(VALID_POINT_TYPES.begin(), VALID_POINT_TYPES.end(), point_type_string))
        );
    }

    /**
     * @brief Alias for PCL message-to-cloud conversion function
     */
    template<typename PointType>
    static inline void message_to_cloud(const sensor_msgs::msg::PointCloud2 &cloud_msg, CloudType<PointType> &cloud) {
        pcl::fromROSMsg(cloud_msg, cloud);
    }

    /**
     * @brief Alias for PCL cloud-to-message conversion function
     */
    template<typename PointType>
    static inline void cloud_to_message(const CloudType<PointType> &cloud, sensor_msgs::msg::PointCloud2 &cloud_msg) {
        pcl::toROSMsg(cloud, cloud_msg);
    }
    
    /**
     * @brief Auxiliary template function returning reference to the collection of stored
     *    point clouds by the currently used point type
     * 
     * @tparam Id 
     *    Id of the currently used point type
     * @returns 
     *    reference to the corresponding collection of clouds
     */
    template<typename PointType> 
    CloudsCollection<PointType> &get_input_clouds() {
        return std::get<CloudsCollection<PointType>>( input_clouds );
    }

private: /* ---------------------------------------------------- ROS interfaces --------------------------------------------------- */

	/// Subscriber interfaces for incoming clouds
	std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subs;
	/// Publisher interface used to provide output point cloud
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
    
private: /* ----------------------------------------------------- Node's state ---------------------------------------------------- */

    /// Reference frame of the resulting cloud
    std::string reference_frame;
    /// Preconfigured mode of the clouds-combining cycle
    CombiningMode combining_mode;
    /// Preconfigured 'fresshness' decay time
    std::optional<rclcpp::Duration> validity_decaying_time;

    /// Map of incoming clouds
    CloudStorage input_clouds;
    
    /// TF2 buffer utilitized by the @a transform_listener to buffer incoming frames
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    /// TF2 interface listening for current position frames of the mimiced turtle
    std::shared_ptr<tf2_ros::TransformListener> transform_listener{ nullptr };

};

/* ================================================================================================================================ */

} // End namespace scan_tools

#endif
