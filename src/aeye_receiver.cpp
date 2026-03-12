#include "aeye_ros2_driver/aeye_receiver.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <rclcpp/logging.hpp>

namespace aeye_ros2_driver {

AeyeReceiver::AeyeReceiver(
    const std::string& bind_ip, uint16_t port, PacketCallback callback)
    : bind_ip_(bind_ip),
      port_(port),
      callback_(std::move(callback)),
      recv_buf_(sizeof(AeyePointPacket))
{
    RCLCPP_DEBUG(rclcpp::get_logger("aeye_receiver"),
        "AeyeReceiver::AeyeReceiver() created...");
}

AeyeReceiver::~AeyeReceiver()
{
    RCLCPP_DEBUG(rclcpp::get_logger("aeye_receiver"),
        "AeyeReceiver::~AeyeReceiver() destructor called. Stopping...");
    stop();
}

void AeyeReceiver::start()
{
    // --------------------------------------------------------
    // TODO 1: Create a UDP socket
    // --------------------------------------------------------
    // Hint: Which SOCK_ type is connectionless?
    // Store the file descriptor in socket_fd_.
    // If it fails, throw a std::runtime_error.
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }
    // --------------------------------------------------------
    // TODO 2: Set socket options
    // --------------------------------------------------------
    // Two options matter here:
    //
    //   a) Address reuse — why would this help during development
    //      when you're frequently restarting the node?
    //
    //   b) Receive buffer size — the default kernel buffer is
    //      small (usually ~200KB). We're receiving bursty,
    //      large datagrams. What happens to incoming data
    //      when the buffer is full and recv() hasn't been
    //      called yet?
    int yes = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

    int requested = 8 * 1024 * 1024;  // 8 MB request
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &requested, sizeof(requested)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("aeye_receiver"),
            "setsockopt falied while requesting buffer size");
        perror("setsockopt");
        close(socket_fd_);
        return;
    }

    int actual = 0;
    socklen_t optlen = sizeof(actual);
    getsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &actual, &optlen);

    RCLCPP_DEBUG(rclcpp::get_logger("aeye_receiver"),"Requested: %d bytes\n", requested);
    RCLCPP_DEBUG(rclcpp::get_logger("aeye_receiver"),"Actual: %d bytes\n", actual);

    // --------------------------------------------------------
    // TODO 3: Bind the socket
    // --------------------------------------------------------
    // Hint: You need a sockaddr_in struct. Think about:
    //   - What does sin_family need to be?
    //   - Why do we call htons() on the port?
    //   - What does binding to "0.0.0.0" mean vs a specific IP?
    //
    // If bind fails, clean up the socket before throwing.
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);          // local port to listen on
    if (inet_pton(AF_INET, bind_ip_.c_str(), &addr.sin_addr) != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("aeye_receiver"), "Invalid bind ip.");
        close(socket_fd_);
        return;
    }

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("aeye_receiver"), "Failed to bind to addr.");
        close(socket_fd_);
        return;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("aeye_receiver"),
        "Bound to %s:%d.", bind_ip_.c_str(), port_);
    // --------------------------------------------------------
    // TODO 4: Set a receive timeout
    // --------------------------------------------------------
    // Hint: Without a timeout, recv() blocks forever. How would
    // the receive loop ever check running_ to know it should
    // stop? What's a reasonable timeout — 10ms? 100ms? 1s?
    // Think about the tradeoff between shutdown responsiveness
    // and CPU usage.
    timeval tv{};
    tv.tv_sec = 0;      // 0 second
    tv.tv_usec = 100000; // 100ms or 0.1s

    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        RCLCPP_WARN(rclcpp::get_logger("aeye_receiver"),
            "setsockopt SO_RCVTIMEO");
    }

    // --------------------------------------------------------
    // TODO 5: Launch the receive thread
    // --------------------------------------------------------
    // Hint: Set running_ to true BEFORE launching the thread.
    // Why does the order matter?
    running_ = true;
    recv_thread_ = std::thread(&AeyeReceiver::receive_loop, this);

    std::cout << "[AeyeReceiver] Listening on "
              << bind_ip_ << ":" << port_ << std::endl;
}

void AeyeReceiver::stop()
{
    // --------------------------------------------------------
    // TODO 6: Shut down cleanly
    // --------------------------------------------------------
    // Think about:
    //   - What order should you do things? (signal, join, close)
    //   - What happens if you close the socket before the
    //     thread has finished its recv() call?
    //   - What if stop() is called twice?
    running_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); // make sure recv loop stops
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
    close(socket_fd_);
}

void AeyeReceiver::receive_loop()
{
    // --------------------------------------------------------
    // TODO 7: Implement the receive loop
    // --------------------------------------------------------
    // This runs on its own thread. The structure is:
    //
    //   while (running_) {
    //       1. Call recv() into recv_buf_
    //       2. Handle errors:
    //          - Timeout is not a real error (which errno values?)
    //          - Zero bytes means nothing useful arrived
    //          - Actual errors should be logged but not fatal
    //       3. On success: deserialize and invoke callback_
    //   }
    //
    // Hint: recv() returns ssize_t. What does a negative
    // return value mean vs zero vs positive?
    while (running_) {
        ssize_t n_bytes = recv(socket_fd_, recv_buf_.data(), sizeof(AeyePointPacket), 0);

        if (n_bytes < 0) {
            RCLCPP_DEBUG(rclcpp::get_logger("aeye_receive_loop"), "n_bytes < 0");
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // timeout or nonblocking no-data case
                continue;
            }
            if (errno == EINTR) {
                // interrupted by signal, retry
                continue;
            }
            std::cerr << "recv failed: " << std::strerror(errno) << "\n";
            break;
        }

        AeyePointPacket out{};
        if (!deserialize(recv_buf_.data(), n_bytes, out)) {
            RCLCPP_DEBUG(rclcpp::get_logger("aeye_receive_loop"), "Deserialize failed");
            continue;
        }
        callback_(out);
    }
}

bool AeyeReceiver::deserialize(
    const uint8_t* raw, size_t length, AeyePointPacket& out)
{
    // --------------------------------------------------------
    // TODO 8: Deserialize raw UDP bytes into AeyePointPacket
    // --------------------------------------------------------
    //
    // The on-wire layout is:
    //
    //   [0..3]   uint32_t  sequence_number
    //   [4..7]   uint32_t  num_points
    //   [8.. ]   AeyeReturnPoint[num_points]   (each 48 bytes)
    //
    // Steps:
    //   a) Validate the datagram is at least as big as the header.
    //      What is the header size? (look at PACKET_HEADER_SIZE)
    //
    //   b) Extract sequence_number and num_points from the raw bytes.
    //      Hint: Why is memcpy preferred over reinterpret_cast here?
    //      Think about strict aliasing and alignment.
    //
    //   c) Sanity check num_points. What should the upper bound be?
    //      What's the right thing to do if it's too large?
    //
    //   d) Calculate the expected total size using num_points.
    //      Verify the datagram is big enough. Why might it be
    //      smaller than expected? (Think network conditions)
    //
    //   e) Copy the point data into out.points.
    //      Hint: This can be done in a single memcpy because
    //      the on-wire layout matches our packed struct layout.
    //      Why does this only work on little-endian machines?
    //
    // Return true on success, false if the datagram is malformed.
    if (length < PACKET_HEADER_SIZE) {
        RCLCPP_ERROR(rclcpp::get_logger("aeye_receive_loop"),
            "Invalid length of recv packet [-1]");
        return false;
    }

    std::memcpy(&out.sequence_number, raw, sizeof(out.sequence_number));
    std::memcpy(&out.num_points, raw + sizeof(out.sequence_number), sizeof(out.num_points));

    if (out.num_points == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("aeye_receive_deserialize"),
            "No points in datagram");
        return false;
    }
    if (out.num_points > MAX_POINTS_PER_PACKET) {
        RCLCPP_DEBUG(rclcpp::get_logger("aeye_receive_deserialize"),
            "Too many points in the packet.");
        return false;
    }

    if (length != PACKET_HEADER_SIZE + out.num_points * sizeof(AeyeReturnPoint)) {
        RCLCPP_DEBUG(rclcpp::get_logger("aeye_receive_deserialize"),
            "Datagram length does not match expected length.");
        return false;
    }

    std::memcpy(&out.points, raw + PACKET_HEADER_SIZE, out.num_points * sizeof(AeyeReturnPoint));
    return true;
}

}  // namespace aeye_ros2_driver
