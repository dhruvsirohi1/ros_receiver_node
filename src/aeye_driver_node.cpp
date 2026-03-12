#include "aeye_ros2_driver/aeye_driver_node.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstring>
#include <functional>

namespace aeye_ros2_driver {

using PointField = sensor_msgs::msg::PointField;

// ============================================================
//  Constructor — orchestrates startup in clear steps
// ============================================================
AeyeDriverNode::AeyeDriverNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("aeye_driver", options)
{
    declare_all_parameters();
    build_field_layout();
    setup_publisher();
    start_receiver();

    RCLCPP_INFO(this->get_logger(),
        "AeyeDriverNode started — listening on %s:%d, publishing to '%s'",
        sensor_ip_.c_str(), port_, topic_name_.c_str());
}

AeyeDriverNode::~AeyeDriverNode()
{
    if (receiver_) {
        receiver_->stop();
    }
}

// ============================================================
//  Step 1: Declare and read all parameters
// ============================================================
//
//  Every configurable value is declared here with a default.
//  ROS 2's parameter system means any of these can be
//  overridden in a launch file, YAML config, or command line
//  without recompiling.
//
void AeyeDriverNode::declare_all_parameters()
{
    // Network
    sensor_ip_ = this->declare_parameter<std::string>("sensor_ip", "0.0.0.0");
    port_ = static_cast<uint16_t>(
        this->declare_parameter<int>("port", 8080));
    recv_buffer_size_ = this->declare_parameter<int>(
        "recv_buffer_size", 4 * 1024 * 1024);

    // Publishing
    frame_id_ = this->declare_parameter<std::string>("frame_id", "aeye_lidar");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/aeye/points");

    // Filtering
    min_range_ = this->declare_parameter<double>("min_range", 0.1);
    max_range_ = this->declare_parameter<double>("max_range", 200.0);
    min_intensity_ = this->declare_parameter<double>("min_intensity", 0.0);

    // Field selection
    include_intensity_ = this->declare_parameter<bool>("include_intensity", true);
    include_timestamp_ = this->declare_parameter<bool>("include_timestamp", false);
    include_point_type_ = this->declare_parameter<bool>("include_point_type", false);

    // Diagnostics
    enable_diagnostics_ = this->declare_parameter<bool>("enable_diagnostics", true);
}

// ============================================================
//  Step 2: Build the PointCloud2 field layout
// ============================================================
//
//  This runs once at startup. We walk through the enabled
//  fields in order, tracking the byte offset as we go.
//  The result is:
//    - active_fields_: describes each field's name, offset, type
//    - point_step_: total bytes per point in the output message
//
//  This avoids recalculating layout on every frame publish.
//
void AeyeDriverNode::build_field_layout()
{
    active_fields_.clear();
    uint32_t offset = 0;

    // x, y, z — always present
    active_fields_.push_back({"x", offset, PointField::FLOAT32, 1});
    offset += sizeof(float);
    active_fields_.push_back({"y", offset, PointField::FLOAT32, 1});
    offset += sizeof(float);
    active_fields_.push_back({"z", offset, PointField::FLOAT32, 1});
    offset += sizeof(float);

    if (include_intensity_) {
        active_fields_.push_back({"intensity", offset, PointField::FLOAT32, 1});
        offset += sizeof(float);
    }

    if (include_timestamp_) {
        // FLOAT64 = 8 bytes, matches uint64_t timestamp_ns
        active_fields_.push_back({"timestamp", offset, PointField::FLOAT64, 1});
        offset += sizeof(double);
    }

    if (include_point_type_) {
        active_fields_.push_back({"point_type", offset, PointField::UINT32, 1});
        offset += sizeof(uint32_t);
    }

    point_step_ = offset;

    RCLCPP_INFO(this->get_logger(),
        "PointCloud2 layout: %zu fields, %zu bytes/point",
        active_fields_.size(), point_step_);
}

// ============================================================
//  Step 3: Create the publisher
// ============================================================
//
//  SensorDataQoS = best-effort reliability + small history depth.
//
//  Best-effort means: if a subscriber can't keep up, we drop
//  messages rather than blocking. For real-time sensor data
//  this is correct — a stale point cloud is worse than a
//  missed one. A SLAM node that falls behind should get the
//  latest scan, not a queue of old ones.
//
void AeyeDriverNode::setup_publisher()
{
    auto logger = this->get_logger();

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_name_,
        rclcpp::SensorDataQoS()
    );
}

// ============================================================
//  Step 4: Create and start the receiver
// ============================================================
//
//  We bind on_packet as the callback. The receiver's internal
//  thread will call this whenever a packet is deserialized.
//
//  std::bind gives us a member function pointer that captures
//  `this`, so the receiver doesn't need to know about ROS.
//
void AeyeDriverNode::start_receiver()
{
    receiver_ = std::make_unique<AeyeReceiver>(
        sensor_ip_,
        port_,
        std::bind(&AeyeDriverNode::on_packet, this, std::placeholders::_1)
    );
    receiver_->start();
}

// ============================================================
//  Packet callback — called from receiver thread
// ============================================================
//
//  This is the heart of the driver. For each packet:
//    1. Walk through every point
//    2. Check SOF/EOF flags to manage frame boundaries
//    3. Accumulate valid points
//    4. Publish on EOF
//
void AeyeDriverNode::on_packet(const AeyePointPacket& packet)
{
    for (uint32_t i = 0; i < packet.num_points; ++i) {
        const auto& point = packet.points[i];

        // --- Start of frame: reset accumulation ---
        if (point.is_sof()) {
            begin_new_frame();
        }

        // --- Accumulate if we're inside a frame ---
        if (frame_in_progress_) {
            accumulate_point(point);
        }

        // --- End of frame: publish what we've got ---
        if (point.is_eof()) {
            publish_frame();
        }
    }
}

// ============================================================
//  Frame management
// ============================================================

void AeyeDriverNode::begin_new_frame()
{
    frame_buffer_.clear();
    // Reserve a reasonable size to avoid repeated reallocation.
    // A typical LiDAR frame is 30k-300k points.
    frame_buffer_.reserve(100000);
    frame_in_progress_ = true;
}

void AeyeDriverNode::accumulate_point(const AeyeReturnPoint& point)
{
    if (passes_filter(point)) {
        frame_buffer_.push_back(point);
    }
}

// ============================================================
//  Point filtering
// ============================================================
//
//  Cheap per-point check. We compute range from xyz rather
//  than storing it separately — three multiplies and a sqrt
//  is negligible compared to the cost of publishing.
//
bool AeyeDriverNode::passes_filter(const AeyeReturnPoint& point) const
{
    // Range filter
    const double range = std::sqrt(
        static_cast<double>(point.x) * point.x +
        static_cast<double>(point.y) * point.y +
        static_cast<double>(point.z) * point.z
    );

    if (range < min_range_ || range > max_range_) {
        return false;
    }

    // Intensity filter
    if (static_cast<double>(point.intensity) < min_intensity_) {
        return false;
    }

    return true;
}

// ============================================================
//  Publish: convert accumulated points → PointCloud2
// ============================================================
//
//  This is where the frame buffer gets serialized into the
//  PointCloud2 binary format. We do this once per frame
//  (not per point) to keep the allocation and copy costs
//  concentrated in one place.
//
void AeyeDriverNode::publish_frame()
{
    if (!frame_in_progress_ || frame_buffer_.empty()) {
        frame_in_progress_ = false;
        return;
    }

    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

    // --- Header ---
    msg->header.stamp = this->now();
    msg->header.frame_id = frame_id_;

    // --- Layout metadata ---
    msg->height = 1;  // unorganized point cloud (single row)
    msg->width = static_cast<uint32_t>(frame_buffer_.size());
    msg->point_step = static_cast<uint32_t>(point_step_);
    msg->row_step = msg->point_step * msg->width;
    msg->is_dense = true;   // we've already filtered out bad points
    msg->is_bigendian = false;

    // --- Field descriptors ---
    msg->fields.resize(active_fields_.size());
    for (size_t i = 0; i < active_fields_.size(); ++i) {
        msg->fields[i].name = active_fields_[i].name;
        msg->fields[i].offset = active_fields_[i].offset;
        msg->fields[i].datatype = active_fields_[i].datatype;
        msg->fields[i].count = active_fields_[i].count;
    }

    // --- Allocate data buffer and fill it ---
    msg->data.resize(msg->row_step);
    uint8_t* write_ptr = msg->data.data();

    for (const auto& point : frame_buffer_) {
        write_point_to_buffer(point, write_ptr);
        write_ptr += point_step_;
    }

    // --- Publish (move semantics — avoids copying the data buffer) ---
    publisher_->publish(std::move(msg));

    if (enable_diagnostics_) {
        RCLCPP_DEBUG(this->get_logger(),
            "Published frame: %zu points, %zu bytes",
            frame_buffer_.size(), msg->data.size());
    }

    frame_in_progress_ = false;
}

// ============================================================
//  Write one point into the PointCloud2 data buffer
// ============================================================
//
//  Copies only the active fields, in the order and offsets
//  defined by build_field_layout(). This is the only place
//  that needs to know the mapping from AeyeReturnPoint fields
//  to PointCloud2 byte positions.
//
//  We use memcpy for each field rather than casting to avoid
//  alignment issues — same principle as the deserializer.
//
void AeyeDriverNode::write_point_to_buffer(
    const AeyeReturnPoint& point, uint8_t* dest) const
{
    // xyz — always present, always first
    std::memcpy(dest + active_fields_[0].offset, &point.x, sizeof(float));
    std::memcpy(dest + active_fields_[1].offset, &point.y, sizeof(float));
    std::memcpy(dest + active_fields_[2].offset, &point.z, sizeof(float));

    size_t idx = 3;  // next field index after x, y, z

    if (include_intensity_) {
        std::memcpy(dest + active_fields_[idx].offset,
                    &point.intensity, sizeof(float));
        ++idx;
    }

    if (include_timestamp_) {
        // Convert uint64_t nanoseconds to double for FLOAT64 field
        const double ts = static_cast<double>(point.timestamp_ns);
        std::memcpy(dest + active_fields_[idx].offset, &ts, sizeof(double));
        ++idx;
    }

    if (include_point_type_) {
        std::memcpy(dest + active_fields_[idx].offset,
                    &point.point_type, sizeof(uint32_t));
        ++idx;
    }
}

}  // namespace aeye_ros2_driver
