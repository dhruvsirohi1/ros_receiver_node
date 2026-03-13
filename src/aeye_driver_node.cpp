#include "aeye_ros2_driver/aeye_driver_node.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstring>
#include <functional>

namespace aeye_ros2_driver {

using PointField = sensor_msgs::msg::PointField;

// ============================================================
//  Constructor
// ============================================================
AeyeDriverNode::AeyeDriverNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("aeye_driver", options)
{
    declare_all_parameters();
    build_field_layout();
    setup_publisher();
    start_receiver();

    RCLCPP_INFO(get_logger(),
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
//  TODO 1: Declare and read all parameters
// ============================================================
//
//  ROS 2 parameters let users change behavior without recompiling.
//
//  Hint: declare_parameter<T>(name, default_value) does two things:
//    1. Registers the parameter so it can be set from YAML or CLI
//    2. Returns the value (from YAML if provided, else the default)
//
//  Look at the member variables in the header to see which
//  parameters need to be declared. Think about what type each
//  one should be (string, int, double, bool).
//
//  Watch out: port_ is uint16_t but ROS 2 parameters don't
//  support uint16_t natively. What's the workaround?
//
void AeyeDriverNode::declare_all_parameters()
{
    // Network params
    this->declare_parameter<std::string>("sensor_ip", "0.0.0.0");
    this->declare_parameter<uint32_t>("port", 8080);
    this->declare_parameter<uint32_t>("recv_buffer_size", 4194304);

    // Publishing params
    this->declare_parameter<std::string>("frame_id", "aeye_lidar");
    this->declare_parameter<std::string>("topic_name", "/aeye/points");

    // Filtering params
    this->declare_parameter<float>("min_range", 0.1f);
    this->declare_parameter<float>("max_range", 200.0f);
    this->declare_parameter<float>("min_intensity", 0.0f);

    // Field selection params
    this->declare_parameter<bool>("include_timestamp", true);
    this->declare_parameter<bool>("include_timestamp", false);
    this->declare_parameter<bool>("include_point_type", false);

    // Diagnostics params
    this->declare_parameter<bool>("enable_diagnostics", false);

    // Fill in the parameters
    this->get_parameter("sensor_ip", sensor_ip_);
    this->get_parameter("port", port_);
    this->get_parameter("recv_buffer_size", recv_buffer_size_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("topic_name", topic_name_);
    double min_range, max_range;
    this->get_parameter("min_range", min_range);
    min_range_sq_ = min_range * min_range;
    this->get_parameter("max_range", max_range);
    max_range_sq_ = max_range * max_range;
    this->get_parameter("min_intensity", min_intensity_);
    this->get_parameter("include_timestamp", include_timestamp_);
    this->get_parameter("include_timestamp", include_timestamp_);
    this->get_parameter("include_point_type", include_point_type_);
    this->get_parameter("enable_diagnostics", enable_diagnostics_);



}

// ============================================================
//  TODO 2: Build the PointCloud2 field layout
// ============================================================
//
//  A PointCloud2 message is a flat byte array with a descriptor
//  that says "bytes 0-3 are float x, bytes 4-7 are float y..."
//
//  This function runs once at startup and builds that descriptor
//  based on which fields are enabled (include_intensity_, etc).
//
//  Steps:
//    a) Clear active_fields_ and start offset at 0
//    b) Always add x, y, z (FLOAT32, 4 bytes each)
//    c) Conditionally add intensity, timestamp, point_type
//       - What PointField datatype constant matches each one?
//       - What size does each add to the offset?
//       - Hint: timestamp is 8 bytes. What PointField type is 8 bytes?
//    d) Store the final offset as point_step_
//
//  Think about: why do we build this once rather than every frame?
//
void AeyeDriverNode::build_field_layout()
{
    active_fields_.clear();

    FieldConfig x{};
    x.name = "x";
    x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    x.offset = 4;
    x.count = 1;
    active_fields_.push_back(x);

    FieldConfig y{};
    x.name = "y";
    x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    x.offset = 4;
    x.count = 1;
    active_fields_.push_back(y);

    FieldConfig z{};
    x.name = "z";
    x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    x.offset = 4;
    x.count = 1;
    active_fields_.push_back(z);

    if (include_intensity_) {
        FieldConfig intensity{};
        x.name = "intensity";
        x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        x.offset = 4;
        x.count = 1;
        active_fields_.push_back(intensity);
    }

    if (include_timestamp_) {
        FieldConfig timestamp{};
        x.name = "timestamp";
        x.datatype = sensor_msgs::msg::PointField::FLOAT64;
        x.offset = 8;
        x.count = 1;
        active_fields_.push_back(timestamp);
    }

    if (include_point_type_) {
        FieldConfig point_type{};
        x.name = "point_type";
        x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        x.offset = 4;
        x.count = 1;
        active_fields_.push_back(point_type);
    }
}

// ============================================================
//  TODO 3: Create the publisher
// ============================================================
//
//  Hint: Use create_publisher<MessageType>(topic, qos).
//
//  For QoS, think about what happens with sensor data:
//    - Should a slow subscriber cause the publisher to wait?
//    - Is it better to drop old data or queue it up?
//    - ROS 2 has a built-in QoS profile designed for exactly
//      this use case. Look at rclcpp::SensorDataQoS.
//
void AeyeDriverNode::setup_publisher()
{

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("aeye_lidar",
        rclcpp::SensorDataQoS());

    RCLCPP_DEBUG(rclcpp::get_logger("aeye_driver_node"), "setup_publisher");
}

// ============================================================
//  TODO 4: Create and start the receiver
// ============================================================
//
//  The receiver needs three things from us:
//    1. IP address to bind to
//    2. Port to listen on
//    3. A callback function to invoke per packet
//
//  Hint: The callback must be a function that takes a
//  const AeyePointPacket&. How do you create a callable
//  from a member function? (look at std::bind or lambdas)
//
void AeyeDriverNode::start_receiver()
{
    auto cb = [this](const AeyePointPacket& packet) {
        on_packet(packet);
    };
    receiver_ = std::make_unique<AeyeReceiver>(AeyeReceiver(sensor_ip_, port_,
        cb));
    receiver_->start();
}

// ============================================================
//  TODO 5: Packet callback — called from receiver thread
// ============================================================
//
//  This is the core data flow. The receiver calls this once
//  per deserialized packet. A packet contains many points,
//  and each point might have SOF or EOF flags set.
//
//  Think about the state machine:
//    - SOF means: a new frame is starting. What should you do
//      with any existing accumulation?
//    - Between SOF and EOF: accumulate points
//    - EOF means: the frame is complete. Time to publish.
//    - What if we start listening mid-frame (no SOF seen yet)?
//      Should we accumulate those points or ignore them?
//
//  Hint: Walk through packet.points[0..num_points-1] and
//  check each point's flags. The SOF and EOF checks should
//  be independent (not else-if). Why?
//
void AeyeDriverNode::on_packet(const AeyePointPacket& packet)
{
    if (packet.num_points <= 0) return;
    if (packet.num_points > MAX_POINTS_PER_PACKET) return;

    for (const auto& point : packet.points) {
        if (point.is_sof()) {
            begin_new_frame();
            frame_in_progress_ = true;
        }
        accumulate_point(point);
        if (point.is_eof()) {
            frame_in_progress_ = false;
            publish_frame();
        }
    }
}

// ============================================================
//  TODO 6: Frame management
// ============================================================

// Start a fresh frame. Think about:
//   - What does clear() vs reserve() do?
//   - How many points might a frame have? What's a good
//     reserve size to avoid repeated reallocation?
void AeyeDriverNode::begin_new_frame()
{
    frame_buffer_.clear();
}

// Add a single point to the current frame.
// Should every point go in, or should we filter first?
void AeyeDriverNode::accumulate_point(const AeyeReturnPoint& point)
{
    if (frame_in_progress_) {
        if (passes_filter(point)) frame_buffer_.push_back(point);
    }
}

// ============================================================
//  TODO 7: Point filtering
// ============================================================
//
//  Given a point, decide if it should be included in the output.
//
//  Checks to implement:
//    a) Range filter — compute distance from origin using x,y,z.
//       Compare against min_range_ and max_range_.
//       Hint: you need sqrt(x² + y² + z²)
//
//    b) Intensity filter — compare against min_intensity_.
//
//  Think about: why filter at the driver level instead of
//  letting downstream nodes handle it?
//
bool AeyeDriverNode::passes_filter(const AeyeReturnPoint& point) const
{
    auto range_sq = point.x * point.x +
                         point.y * point.y +
                         point.z * point.z;

    return point.intensity > min_intensity_ &&
        (range_sq >= min_range_sq_ && range_sq <= max_range_sq_);
}

// ============================================================
//  TODO 8: Publish a completed frame as PointCloud2
// ============================================================
//
//  This is the most involved function. You're converting from
//  your internal buffer (vector of AeyeReturnPoint) into the
//  ROS 2 standard message format.
//
//  Steps:
//    a) Guard: if no frame in progress or buffer is empty,
//       reset state and return.
//
//    b) Create the message. Hint: use std::make_unique<>()
//       so we can move it into publish() later. Why does
//       moving matter for a multi-megabyte message?
//
//    c) Fill the header:
//       - stamp: current ROS time (how do you get this from a Node?)
//       - frame_id: from our parameter
//
//    d) Fill the layout metadata:
//       - height = 1 (unorganized point cloud — what does this mean?)
//       - width = number of points
//       - point_step = bytes per point (we computed this at startup)
//       - row_step = point_step × width
//       - is_dense = true (why can we claim this?)
//       - is_bigendian = false (why?)
//
//    e) Fill the field descriptors from active_fields_.
//       Each entry maps to a sensor_msgs::msg::PointField.
//
//    f) Allocate msg->data (row_step bytes) and fill it by
//       calling write_point_to_buffer() for each accumulated point.
//
//    g) Publish using std::move(msg)
//
//    h) Reset frame_in_progress_
//
void AeyeDriverNode::publish_frame()
{

}

// ============================================================
//  TODO 9: Write one point into the PointCloud2 data buffer
// ============================================================
//
//  dest points to the start of this point's slot in the
//  output byte array.
//
//  Copy each active field into dest at the correct offset.
//  Use active_fields_[i].offset to know where each field goes.
//
//  Hints:
//    - x, y, z are always present and always first (indices 0,1,2)
//    - After that, the index depends on which optionals are enabled
//    - Use a running index counter starting at 3
//    - Use memcpy to write each field. Why not just cast and assign?
//    - timestamp needs a type conversion: uint64_t → double
//
void AeyeDriverNode::write_point_to_buffer(
    const AeyeReturnPoint& point, uint8_t* dest) const
{

}

}  // namespace aeye_ros2_driver
