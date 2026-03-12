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
      recv_buf_(sizeof(AeyePointPacket))
{
}

AeyeReceiver::~AeyeReceiver()
{
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


    // --------------------------------------------------------
    // TODO 3: Bind the socket
    // --------------------------------------------------------
    // Hint: You need a sockaddr_in struct. Think about:
    //   - What does sin_family need to be?
    //   - Why do we call htons() on the port?
    //   - What does binding to "0.0.0.0" mean vs a specific IP?
    //
    // If bind fails, clean up the socket before throwing.


    // --------------------------------------------------------
    // TODO 4: Set a receive timeout
    // --------------------------------------------------------
    // Hint: Without a timeout, recv() blocks forever. How would
    // the receive loop ever check running_ to know it should
    // stop? What's a reasonable timeout — 10ms? 100ms? 1s?
    // Think about the tradeoff between shutdown responsiveness
    // and CPU usage.


    // --------------------------------------------------------
    // TODO 5: Launch the receive thread
    // --------------------------------------------------------
    // Hint: Set running_ to true BEFORE launching the thread.
    // Why does the order matter?


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

    return false;
}

}  // namespace aeye_ros2_driver
