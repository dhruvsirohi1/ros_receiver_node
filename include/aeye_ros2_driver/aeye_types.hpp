#ifndef AEYE_ROS2_DRIVER__AEYE_TYPES_HPP_
#define AEYE_ROS2_DRIVER__AEYE_TYPES_HPP_

#include <cstdint>
#include <cstddef>

namespace aeye_ros2_driver {

// --- Point type bitmask flags ---
constexpr uint32_t POINT_TYPE_SOF = 0x10;  // bit 4: start of frame
constexpr uint32_t POINT_TYPE_EOF = 0x20;  // bit 5: end of frame

// --- Packet limits ---
constexpr size_t MAX_POINTS_PER_PACKET = 1024;

// --- On-wire structures (must match FPGA/aeye_service layout) ---

struct __attribute__((packed)) AeyeReturnPoint {
    float x;                  // meters
    float y;
    float z;
    float intensity;
    uint64_t timestamp_ns;    // sensor timestamp
    uint32_t point_type;      // bitmask: SOF, EOF, return type, etc.
    uint8_t reserved[20];     // pad to 48 bytes total

    bool is_sof() const { return (point_type & POINT_TYPE_SOF) != 0; }
    bool is_eof() const { return (point_type & POINT_TYPE_EOF) != 0; }
};

static_assert(sizeof(AeyeReturnPoint) == 48, "AeyeReturnPoint must be 48 bytes");

struct __attribute__((packed)) AeyePointPacket {
    uint32_t sequence_number;
    uint32_t num_points;       // actual number of valid points in this packet
    AeyeReturnPoint points[MAX_POINTS_PER_PACKET];
};

// Header size: the bytes before the points array
constexpr size_t PACKET_HEADER_SIZE = sizeof(uint32_t) + sizeof(uint32_t);  // 8 bytes

}  // namespace aeye_ros2_driver

#endif  // AEYE_ROS2_DRIVER__AEYE_TYPES_HPP_
