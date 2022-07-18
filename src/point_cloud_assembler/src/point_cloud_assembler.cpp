/* ============================================================================================================================ *//**
 * @file       point_cloud_assembler.h
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 23rd March 2022 10:09:09 pm
 * @modified   Wednesday, 25th May 2022 11:07:29 pm
 * @project    scan-tools
 * @brief      Definitions of the ROS2 node class assembling incoming PCL points clouds into a single points cloud defined in the
 *             common reference frame
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <algorithm>
#include <limits>
#include <exception>
// Common includes
#include "node_common/node.hpp"
// Private includes
#include "scan_tools/point_cloud_assembler.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {

/* ========================================================== Inner types ========================================================= */

template<typename PointType>
PointCloudAssembler::CloudDescriptor<PointType>::CloudDescriptor(const rclcpp::Time &refresh_timestamp) :
    refresh_stamp{ refresh_timestamp }    
{ }

/* ========================================================= Ctors & dtors ======================================================== */

PointCloudAssembler::PointCloudAssembler(const rclcpp::NodeOptions & options) : 
    rclcpp::Node("point_cloud_assembler", options)
{
    /* ------------------------ Register parameters ------------------------ */

    // Declare parameter defining input topics
    auto input_topics = node_common::parameters::declare_parameter_and_get(*this, INPUT_TOPICS_PARAM_DESCRIPTOR);
    // Declare parameter defining reference frame for the resulting cloud
    auto reference_frame = node_common::parameters::declare_parameter_and_get(*this, REFERENCE_FRAME_PARAM_DESCRIPTOR);
    // Declare parameter defining reference frames of clouds that should be included in the assembled cloud
    auto frames = node_common::parameters::declare_parameter_and_get(*this, FRAMES_PARAM_DESCRIPTOR);
    // Declare parameter defining assembly mode for the node
    auto combining_mode = node_common::parameters::declare_parameter_and_get(*this, COMBINING_MODE_PARAM_DESCRIPTOR);
    // Declare parameter defining decaying speed of incomping clouds' validity
    auto validity_decaying_time = declare_parameter_and_get(*this, VALIDITY_DECAYING_TIME_PARAM_DESCRIPTOR);
    // Declare parameter defining type of points in the assembled clouds
    auto point_type = node_common::parameters::declare_parameter_and_get(*this, POINT_TYPE_PARAM_DESCRIPTOR);

    /* ------------------------- Validate parameters ----------------------- */

    // Check if input topics has been given
    if(input_topics->empty())
        rclcpp::exceptions::InvalidParametersException("No input topic has been registered");

    // Check if 'reference_frame' is set to a valid value
    if(not reference_frame.has_value())
        rclcpp::exceptions::InvalidParametersException("'reference_frame' has not been set");
    if(reference_frame->empty())
        rclcpp::exceptions::InvalidParametersException("'reference_frame' cannot be an empty string");

    // Check if valid combining mode has been given
    if(combining_mode != "batch" and combining_mode != "continuous")
        rclcpp::exceptions::InvalidParametersException("Invalid 'combining_mode' mode has been given");

    // Check if 'batch' combining mode has been given without list of frames to combine
    if(combining_mode == "batch" and (not frames.has_value() or frames->empty()))
        rclcpp::exceptions::InvalidParametersException("No reference frames has been listed for 'batch' combining mode");

    // Check if 'validity_decaying_time' is non-positive
    if(validity_decaying_time.has_value() and *validity_decaying_time <= 0.0)
        rclcpp::exceptions::InvalidParametersException("'validity_decaying_time' cannot be non-positive");

    // Check if 'point_type' is one of supported values
    if(std::find(VALID_POINT_TYPES.begin(), VALID_POINT_TYPES.end(), *point_type) == VALID_POINT_TYPES.end())
        rclcpp::exceptions::InvalidParametersException("'point_type' has invalid value");

    /* ----------------- Register communciation interfaces ----------------- */

    // Prepare memory for input topics' handles
    cloud_subs.resize(input_topics->size()) ;
    // Initialize input topics 
    for(unsigned i = 0; i < input_topics->size(); ++i) {
        *node_common::communication::make_subscriber_builder(cloud_subs[i])
            .node(*this)
            .name(input_topics->at(i))
            .qos(TOPIC_QUEUE_SIZE)
            .callback(*this, &PointCloudAssembler::cloud_callback);
    }

    // Initialize publisher used to provide output point cloud
    *node_common::communication::make_publisher_builder(cloud_pub)
        .node(*this)
        .name(CLOUD_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);

    /* --------------------- Initialize internal state --------------------- */

    // Keep reference frame
    this->reference_frame = *reference_frame;
    
    // Keep the combining mode
    if(combining_mode == "batch")
        this->combining_mode = CombiningMode::Batch;
    else if(not frames.has_value() or frames->empty())
        this->combining_mode = CombiningMode::Continuous;
    else
        this->combining_mode = CombiningMode::SelectiveContinuous;
    
    // Initialize the clouds' storage
    reset_cloud_storage(point_type_id_from_string(*point_type));

    // Keep the decay time
    if(validity_decaying_time.has_value()) {

        // Convert floating-point seconds to duration
        auto validity_decaying_time_s = 
            std::chrono::duration<double, std::ratio<1>>{ *validity_decaying_time };
        // Convert to integer-based nanoseconds representation
        auto validity_decaying_time_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(validity_decaying_time_s);
        // Keep the timeout value as ROS-specific structure
        this->validity_decaying_time.emplace(validity_decaying_time_ns);

    }

    // If selective mode is configured, preinitialize clouds' storage
    if(this->combining_mode == CombiningMode::Batch or this->combining_mode == CombiningMode::SelectiveContinuous) {

        std::visit([&frames](auto &clouds) {

            using CloudDescriptorType = cloud_descriptor_by_cloud_collection_t<decltype(clouds)>;
        
            // Reserve memory for cloud's descriptors
            clouds.reserve(frames->size());
            // Construct clouds descriptors
            for(auto &frame_name : *frames) {
                clouds.emplace(std::make_pair(
                    frame_name,
                    CloudDescriptorType {
                        rclcpp::Time(std::numeric_limits<int64_t>::min()) 
                    }
                ));
            }
            
        }, input_clouds);

    }

    // Prepare TF2 buffer for the TF2 transform-listener interface
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // Create TF2 transform-listener interface with the given buffer
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
    /* --------------------------------------------------------------------- */

    node_common::node::print_hello(*this);
}

PointCloudAssembler::~PointCloudAssembler() {
    node_common::node::print_goodbye(*this);
}

/* =========================================================== Callbacks ========================================================== */

void PointCloudAssembler::cloud_callback(const sensor_msgs::msg::PointCloud2 &cloud_msg) {
    std::visit([this, &cloud_msg](auto &clouds){

        using PointType = point_by_cloud_collection_t<decltype(clouds)>;

        // Get current time
        auto now = this->get_clock()->now();
        
        // Try to store incoming cloud; if it has been accepted process current set of component clouds
        if(store_incoming_cloud<PointType>(now, cloud_msg)) {

            // Gather clouds to be compiled
            auto cloud_components = select_clouds_to_compile<PointType>(now);
            // Compile and output clouds
            compile_clouds<PointType>(now, cloud_components);

        }
    

    }, input_clouds);
}

/* ==================================================== Private helper methods ==================================================== */

void PointCloudAssembler::reset_cloud_storage(PointTypeId new_point_type) {
    switch(new_point_type) {
        case PointTypeId::XYZ:  input_clouds.emplace<cloud_collection_by_point_id_t<PointTypeId::XYZ>>();  return;
        case PointTypeId::XYZL: input_clouds.emplace<cloud_collection_by_point_id_t<PointTypeId::XYZL>>(); return;
        case PointTypeId::XYZI: input_clouds.emplace<cloud_collection_by_point_id_t<PointTypeId::XYZI>>(); return;
    };
}

template<typename PointType>
bool PointCloudAssembler::store_incoming_cloud(
    const rclcpp::Time &now,
    const sensor_msgs::msg::PointCloud2 &cloud_msg
) {

    // Get reference to the collection of stored clouds
    auto &input_clouds = get_input_clouds<PointType>();

    // Get reference frame of the cloud
    auto &reference_frame = cloud_msg.header.frame_id;

    // Ignore clouds associated with no reference frame
    if(reference_frame.empty())
        return false;

    // Try to store the cloud
    try {
        
        // If continuous mode is preconfigured keep the incoming cloud
        if(this->combining_mode == CombiningMode::Continuous) {

            // Check if cloud associated with the given reference frame has been already received
            if(input_clouds.find(reference_frame) != input_clouds.end()) {

                // Get reference to the cloud's descriptor
                auto &cloud_descriptor = input_clouds.at(reference_frame);
                // Update cloud's freshsness ('refreshed' flag is not used in the 'Continuous' mode)
                cloud_descriptor.refresh_stamp = now;
                // Copy cloud's content
                message_to_cloud(cloud_msg, cloud_descriptor.cloud);

            // Else, if cloud associated with the given reference frame has NOT been received yet
            } else {

                // Insert the new cloud's descriptor
                auto insert_info = input_clouds.emplace(std::make_pair(
                    reference_frame,
                    CloudDescriptor<PointType> { now }
                ));
                // Copy cloud's content
                message_to_cloud(cloud_msg, insert_info.first->second.cloud);

            }

        // Else, if selective mode is preconfigured, check if one of accepted clouds has been received
        } else {

            // If cloud associated with the given reference is not accepted, return
            if(input_clouds.find(reference_frame) == input_clouds.end())
                return false;

            // Get reference to the cloud's descriptor
            auto &cloud_descriptor = input_clouds.at(reference_frame);
            // Update cloud's freshsness ('refreshed' flag is not used in the 'Continuous' mode)
            if(this->combining_mode == CombiningMode::Batch)
                cloud_descriptor.refreshed = true;
            cloud_descriptor.refresh_stamp = now;
            // Copy cloud's content
            message_to_cloud(cloud_msg, cloud_descriptor.cloud);
        }

    // On storing error
    } catch(std::exception &ex) {

        // Log warning
        RCLCPP_WARN_STREAM(this->get_logger(),"Failed to alocate incoming cloud for '" << reference_frame <<  "' reference frame");
        // Return failure status
        return false;

    }

    return true;
}

template<typename PointType>
std::vector<const PointCloudAssembler::CloudType<PointType>*> 
PointCloudAssembler::select_clouds_to_compile(const rclcpp::Time &now) {

    // Get reference to the collection of stored clouds
    auto &input_clouds = get_input_clouds<PointType>();

    std::optional<rclcpp::Time> fresh_limit_timestamp;

    // Get timestamp before which stored clouds are considered 'non-fresh'
    if(validity_decaying_time.has_value())
        fresh_limit_timestamp.emplace(now - *validity_decaying_time);

    // Prepare list of clouds to be concatenated in the resulting cloud
    std::vector<const CloudType<PointType>*> cloud_components;
        
    // Gather valid clouds depending on the mode
    switch(this->combining_mode) {
            
        // In continuous modes include all clouds that are 'fresh' enough
        case CombiningMode::Continuous:                
        case CombiningMode::SelectiveContinuous:

            // Iterate over clouds descriptors and include resh ones into the 'to combine' set
            for(const auto &[frame, descriptor] : input_clouds) {
                if( not fresh_limit_timestamp.has_value() or descriptor.refresh_stamp >= fresh_limit_timestamp )
                    cloud_components.push_back(&descriptor.cloud);
            }

            break;

        // In the batch mode check all clouds' freshness before processing
        case CombiningMode::Batch:

            /**
             * Check if all registered clouds has bee refreshed since the last 
             * compiled cloud published and before passing it's freshness timerange
             */
            bool batch_ready_to_compile = std::all_of(input_clouds.begin(), input_clouds.end(),
                [now, fresh_limit_timestamp](const auto &cloud_record) {
                    return 
                        // Check if cloud has been refreshed since last compilation
                        cloud_record.second.refreshed and
                        // Check if cloud has been refreshed in the acceptable freshness timerange
                        (not fresh_limit_timestamp.has_value() or cloud_record.second.refresh_stamp >= fresh_limit_timestamp);
                }
            );

            // If batch is not ready to compile, return
            if(not batch_ready_to_compile)
                break;

            // Otherwise push all clouds into the 'to combine' set
            for(auto &[frame, descriptor] : input_clouds) {

                // Pull cloud into the compilation set
                cloud_components.push_back(&descriptor.cloud);
                // Mark cloud as requiring refreshing before the next batch
                descriptor.refreshed = false;
                
            }

            break;
            
    }

    return cloud_components;
}


template<typename PointType>
void PointCloudAssembler::compile_clouds(
    const rclcpp::Time &now,
    const std::vector<const CloudType<PointType>*> &cloud_components
) {
    // If there are no clouds to be compiled, return
    if(cloud_components.empty())
        return;

    // Prepare the output cloud
    CloudType<PointType> compiled_cloud;

    // Iterate over component clouds
    for(auto &component : cloud_components) {

        // Prepare container for the component cloud transformed into the compiled cloud's reference frame
        CloudType<PointType> transformed_component;

        // Try to transform the cloud
        try {

            // Transform component into the compiled cloud's reference frame
            if(not pcl_ros::transformPointCloud(reference_frame, *component, transformed_component, *tf_buffer)) {

                // Log warning
                RCLCPP_WARN(this->get_logger(),
                    "Failed to transform component cloud from '%s' into the target reference frame '%s' reference frame. Skipping cloud...",
                    reference_frame.c_str(), component->header.frame_id.c_str()
                );

                // Skip the cloud
                continue;
                
            }

        // If transformation has not been accessible, skip it
        } catch(tf2::ConnectivityException &ex) {

            // Log warning
            RCLCPP_WARN(this->get_logger(),
                "Failed to transform component cloud from '%s' into the target reference frame '%s' reference frame (%s). Skipping cloud...",
                reference_frame.c_str(), component->header.frame_id.c_str(), ex.what()
            );

            // Skip the cloud
            continue;
                
        }

        // On success, add transformed cloud into the output result
        compiled_cloud += transformed_component;
    }

    sensor_msgs::msg::PointCloud2 compiled_cloud_msg;

    // Convert point cloud to the ROS message
    cloud_to_message(compiled_cloud, compiled_cloud_msg);
    // Fill output cloud's header
    compiled_cloud_msg.header.frame_id = reference_frame;
    compiled_cloud_msg.header.stamp = now;

    // Publish the frame
    cloud_pub->publish(compiled_cloud_msg);
}

/* ================================================================================================================================ */

} // End namespace scan_tools

/* ======================================================== Nodes' registry ======================================================= */

RCLCPP_COMPONENTS_REGISTER_NODE(scan_tools::PointCloudAssembler)

/* ================================================================================================================================ */

