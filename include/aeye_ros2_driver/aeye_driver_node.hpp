#ifndef AEYE_ROS2_DRIVER__AEYE_DRIVER_NODE_HPP_
#define AEYE_ROS2_DRIVER__AEYE_DRIVER_NODE_HPP_

#include "aeye_ros2_driver/aeye_types.hpp"
#include "aeye_ros2_driver/aeye_receiver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <string>
#include <vector>
#include <mutex>

namespace aeye_ros2_driver {

// Describes one field in our PointCloud2 layout
struct FieldConfig {
    std::string name;
    uint32_t offset;       // byte offset within a single point
    uint8_t datatype;      // sensor_msgs::msg::PointField constant
    uint32_t count;        // number of elements (1 for scalar)
};

class AeyeDriverNode : public rclcpp::Node {
public:
    explicit AeyeDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AeyeDriverNode() override;

private:
    // --- Initialization helpers (called once in constructor) ---
    void declare_all_parameters();
    void build_field_layout();
    void setup_publisher();
    void start_receiver();

    // --- Called by AeyeReceiver on its thread, once per packet ---
    void on_packet(const AeyePointPacket& packet);

    // --- Frame assembly ---
    void begin_new_frame();
    void accumulate_point(const AeyeReturnPoint& point);
    void publish_frame();

    // --- PointCloud2 construction ---
    bool passes_filter(const AeyeReturnPoint& point) const;
    void write_point_to_buffer(const AeyeReturnPoint& point,
                               uint8_t* dest) const;

    // === Parameters ===
    std::string sensor_ip_;
    uint16_t port_;
    std::string frame_id_;
    std::string topic_name_;
    int recv_buffer_size_;
    double min_range_;
    double max_range_;
    double min_intensity_;
    bool include_intensity_;
    bool include_timestamp_;
    bool include_point_type_;
    bool enable_diagnostics_;

    // === PointCloud2 field layout (built once at startup) ===
    std::vector<FieldConfig> active_fields_;
    size_t point_step_;  // bytes per point in the output message

    // === Publisher ===
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    // === Receiver ===
    std::unique_ptr<AeyeReceiver> receiver_;

    // === Frame accumulation buffer ===
    // Points are accumulated here between SOF and EOF,
    // then bulk-converted to PointCloud2 on EOF.
    std::vector<AeyeReturnPoint> frame_buffer_;
    bool frame_in_progress_{false};
};

}  // namespace aeye_ros2_driver

#endif  // AEYE_ROS2_DRIVER__AEYE_DRIVER_NODE_HPP_
