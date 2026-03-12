#ifndef AEYE_ROS2_DRIVER__AEYE_RECEIVER_HPP_
#define AEYE_ROS2_DRIVER__AEYE_RECEIVER_HPP_

#include "aeye_ros2_driver/aeye_types.hpp"

#include <string>
#include <vector>
#include <functional>
#include <atomic>
#include <thread>

namespace aeye_ros2_driver {

/**
 * AeyeReceiver
 *
 * Opens a UDP socket, receives raw datagrams from aeye_service,
 * deserializes them into AeyePointPacket structs, and invokes
 * a user-supplied callback with the parsed packet.
 *
 * Runs its own receive thread — call start() / stop().
 */
class AeyeReceiver {
public:
    // Callback signature: called once per successfully deserialized packet
    using PacketCallback = std::function<void(const AeyePointPacket& packet)>;

    AeyeReceiver(const std::string& bind_ip, uint16_t port, PacketCallback callback);
    ~AeyeReceiver();

    // Non-copyable
    AeyeReceiver(const AeyeReceiver&) = delete;
    AeyeReceiver& operator=(const AeyeReceiver&) = delete;

    void start();
    void stop();

private:
    void receive_loop();

    /**
     * Deserialize raw UDP datagram bytes into an AeyePointPacket.
     * Returns true on success, false if the datagram is malformed.
     */
    bool deserialize(const uint8_t* raw, size_t length, AeyePointPacket& out);

    std::string bind_ip_;
    uint16_t port_;
    PacketCallback callback_;

    int socket_fd_{-1};
    std::atomic<bool> running_{false};
    std::thread recv_thread_;

    // Raw receive buffer — sized for the largest possible packet
    std::vector<uint8_t> recv_buf_;
};

}  // namespace aeye_ros2_driver

#endif  // AEYE_ROS2_DRIVER__AEYE_RECEIVER_HPP_
