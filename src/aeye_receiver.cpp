#include "aeye_ros2_driver/aeye_receiver.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <iostream>

namespace aeye_ros2_driver {

AeyeReceiver::AeyeReceiver(
    const std::string& bind_ip, uint16_t port, PacketCallback callback)
    : bind_ip_(bind_ip),
      port_(port),
      callback_(std::move(callback)),
      recv_buf_(sizeof(AeyePointPacket))  // max possible datagram size
{
}

AeyeReceiver::~AeyeReceiver()
{
    stop();
}

void AeyeReceiver::start()
{
    // --- Create UDP socket ---
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        throw std::runtime_error("Failed to create UDP socket: "
                                 + std::string(strerror(errno)));
    }

    // Allow address reuse (helpful during development / restarts)
    int optval = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    // Increase receive buffer size — we're getting large, bursty datagrams
    int rcvbuf_size = 4 * 1024 * 1024;  // 4 MB
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size));

    // --- Bind to address ---
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    inet_pton(AF_INET, bind_ip_.c_str(), &addr.sin_addr);

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("Failed to bind UDP socket to "
                                 + bind_ip_ + ":" + std::to_string(port_)
                                 + " — " + std::string(strerror(errno)));
    }

    // --- Set a receive timeout so the loop can check running_ periodically ---
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100 ms
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // --- Launch receive thread ---
    running_ = true;
    recv_thread_ = std::thread(&AeyeReceiver::receive_loop, this);

    std::cout << "[AeyeReceiver] Listening on "
              << bind_ip_ << ":" << port_ << std::endl;
}

void AeyeReceiver::stop()
{
    running_ = false;

    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

// ============================================================
//  Receive loop — runs on its own thread
// ============================================================
void AeyeReceiver::receive_loop()
{
    while (running_) {
        // Blocking recv with timeout (set in start())
        ssize_t bytes_read = recv(
            socket_fd_,
            recv_buf_.data(),
            recv_buf_.size(),
            0  // flags
        );

        if (bytes_read < 0) {
            // Timeout or interrupted — just loop and check running_
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            std::cerr << "[AeyeReceiver] recv error: "
                      << strerror(errno) << std::endl;
            continue;
        }

        if (bytes_read == 0) {
            continue;
        }

        // --- Deserialize and dispatch ---
        AeyePointPacket packet;
        if (deserialize(recv_buf_.data(),
                        static_cast<size_t>(bytes_read),
                        packet))
        {
            callback_(packet);
        }
    }
}

// ============================================================
//  Deserialization: raw UDP bytes → AeyePointPacket
// ============================================================
//
//  On-wire layout (packed, little-endian — matches x86 and the FPGA):
//
//    [0..3]   uint32_t  sequence_number
//    [4..7]   uint32_t  num_points
//    [8.. ]   AeyeReturnPoint[num_points]   (each 48 bytes)
//
//  The FPGA writes points directly into DMA memory in this packed
//  layout. aeye_service reads them out and forwards over UDP with
//  no re-encoding — so on an x86 host we can memcpy straight into
//  our matching packed structs.
//
bool AeyeReceiver::deserialize(
    const uint8_t* raw, size_t length, AeyePointPacket& out)
{
    // --- Validate minimum size: need at least the header ---
    if (length < PACKET_HEADER_SIZE) {
        std::cerr << "[AeyeReceiver] Datagram too short for header: "
                  << length << " bytes" << std::endl;
        return false;
    }

    // --- Read header fields ---
    std::memcpy(&out.sequence_number, raw, sizeof(uint32_t));
    std::memcpy(&out.num_points, raw + sizeof(uint32_t), sizeof(uint32_t));

    // --- Sanity checks ---
    if (out.num_points > MAX_POINTS_PER_PACKET) {
        std::cerr << "[AeyeReceiver] num_points (" << out.num_points
                  << ") exceeds MAX_POINTS_PER_PACKET" << std::endl;
        return false;
    }

    const size_t expected_size =
        PACKET_HEADER_SIZE + (out.num_points * sizeof(AeyeReturnPoint));

    if (length < expected_size) {
        std::cerr << "[AeyeReceiver] Datagram truncated: got "
                  << length << " bytes, expected " << expected_size
                  << " for " << out.num_points << " points" << std::endl;
        return false;
    }

    // --- Copy point data in one shot ---
    //
    // This works because:
    //   1. Both FPGA and x86 are little-endian
    //   2. Structs are __attribute__((packed)) — no padding surprises
    //   3. The on-wire layout matches our in-memory layout exactly
    //
    // If the sensor were big-endian, we'd need to byte-swap each field here.
    //
    std::memcpy(out.points,
                raw + PACKET_HEADER_SIZE,
                out.num_points * sizeof(AeyeReturnPoint));

    return true;
}

}  // namespace aeye_ros2_driver
