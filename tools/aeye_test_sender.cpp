#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <random>
#include <vector>
#include <csignal>
#include <atomic>
#include <getopt.h>

// ============================================================
//  Types — must match aeye_types.hpp exactly
// ============================================================

constexpr uint32_t POINT_TYPE_SOF = 0x10;
constexpr uint32_t POINT_TYPE_EOF = 0x20;
constexpr size_t MAX_POINTS_PER_PACKET = 1024;

struct __attribute__((packed)) AeyeReturnPoint {
    float x, y, z;
    float intensity;
    uint64_t timestamp_ns;
    uint32_t point_type;
    uint8_t reserved[20];
};

static_assert(sizeof(AeyeReturnPoint) == 48, "AeyeReturnPoint must be 48 bytes");

struct __attribute__((packed)) AeyePointPacket {
    uint32_t sequence_number;
    uint32_t num_points;
    AeyeReturnPoint points[MAX_POINTS_PER_PACKET];
};

constexpr size_t PACKET_HEADER_SIZE = sizeof(uint32_t) + sizeof(uint32_t);

// ============================================================
//  Signal handling for clean shutdown
// ============================================================

static std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

// ============================================================
//  Scan pattern generators
// ============================================================

enum class ScanPattern {
    HEMISPHERE,   // dome / half-sphere — default, looks good in RViz
    FLAT_PLANE,   // ground plane with noise — tests range filtering
    CUBE,         // points on cube surfaces — tests sharp edges
    ROTATING_LINE // single vertical line rotating — tests frame timing
};

ScanPattern parse_pattern(const std::string& s) {
    if (s == "hemisphere")     return ScanPattern::HEMISPHERE;
    if (s == "flat_plane")     return ScanPattern::FLAT_PLANE;
    if (s == "cube")           return ScanPattern::CUBE;
    if (s == "rotating_line")  return ScanPattern::ROTATING_LINE;
    std::cerr << "Unknown pattern '" << s << "', using hemisphere\n";
    return ScanPattern::HEMISPHERE;
}

// Generates one full frame of points for the given pattern.
// frame_index drives animation (rotation etc).
std::vector<AeyeReturnPoint> generate_frame(
    ScanPattern pattern,
    int points_per_frame,
    int frame_index,
    std::mt19937& rng)
{
    std::vector<AeyeReturnPoint> points;
    points.reserve(points_per_frame);

    std::normal_distribution<float> noise(0.0f, 0.02f);
    std::uniform_real_distribution<float> intensity_dist(0.1f, 1.0f);

    const float rotation = static_cast<float>(frame_index) * 0.05f;  // slow rotation
    const auto now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count()
    );

    switch (pattern) {
    case ScanPattern::HEMISPHERE: {
        // Points distributed over a hemisphere at ~10m radius
        for (int i = 0; i < points_per_frame; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(points_per_frame);
            // Fibonacci sphere for even distribution
            float phi = std::acos(1.0f - t);  // 0 to pi/2 for hemisphere
            float theta = 2.4f * static_cast<float>(i) + rotation;

            float radius = 10.0f + noise(rng);
            float x = radius * std::sin(phi) * std::cos(theta);
            float y = radius * std::sin(phi) * std::sin(theta);
            float z = radius * std::cos(phi);

            // Intensity falls off with elevation angle
            float intensity = intensity_dist(rng) * (1.0f - t * 0.5f);

            AeyeReturnPoint pt{};
            pt.x = x;
            pt.y = y;
            pt.z = z;
            pt.intensity = intensity;
            pt.timestamp_ns = now_ns + static_cast<uint64_t>(i) * 1000;
            pt.point_type = 0;
            points.push_back(pt);
        }
        break;
    }

    case ScanPattern::FLAT_PLANE: {
        // Ground plane at z = -1.5m (typical LiDAR mount height)
        // with some objects sticking up
        for (int i = 0; i < points_per_frame; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(points_per_frame);
            float angle = t * 2.0f * M_PI + rotation;
            float range = 2.0f + 18.0f * t;  // 2 to 20 meters

            float x = range * std::cos(angle);
            float y = range * std::sin(angle);
            float z = -1.5f + noise(rng);

            // Add some "obstacles" — raised points in certain sectors
            if (range > 5.0f && range < 8.0f && std::fmod(angle, 1.0f) < 0.3f) {
                z += 1.0f + std::fabs(noise(rng)) * 5.0f;
            }

            AeyeReturnPoint pt{};
            pt.x = x;
            pt.y = y;
            pt.z = z;
            pt.intensity = intensity_dist(rng);
            pt.timestamp_ns = now_ns + static_cast<uint64_t>(i) * 1000;
            pt.point_type = 0;
            points.push_back(pt);
        }
        break;
    }

    case ScanPattern::CUBE: {
        // Points on the surfaces of a 5m cube centered at origin
        std::uniform_real_distribution<float> face(-2.5f, 2.5f);
        std::uniform_int_distribution<int> face_sel(0, 5);

        for (int i = 0; i < points_per_frame; ++i) {
            float a = face(rng);
            float b = face(rng);
            float x, y, z;

            // Pick a random face
            switch (face_sel(rng)) {
                case 0: x =  2.5f; y = a; z = b; break;
                case 1: x = -2.5f; y = a; z = b; break;
                case 2: x = a; y =  2.5f; z = b; break;
                case 3: x = a; y = -2.5f; z = b; break;
                case 4: x = a; y = b; z =  2.5f; break;
                default: x = a; y = b; z = -2.5f; break;
            }

            // Apply rotation around Z axis
            float rx = x * std::cos(rotation) - y * std::sin(rotation);
            float ry = x * std::sin(rotation) + y * std::cos(rotation);

            // Push out to ~10m range
            AeyeReturnPoint pt{};
            pt.x = rx + 10.0f;
            pt.y = ry;
            pt.z = z;
            pt.intensity = intensity_dist(rng);
            pt.timestamp_ns = now_ns + static_cast<uint64_t>(i) * 1000;
            pt.point_type = 0;
            points.push_back(pt);
        }
        break;
    }

    case ScanPattern::ROTATING_LINE: {
        // Single vertical line that sweeps around — easy to see frame timing
        float angle = rotation * 2.0f;
        float range = 10.0f;

        for (int i = 0; i < points_per_frame; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(points_per_frame);
            float z = -2.0f + 4.0f * t;  // -2m to +2m vertical

            AeyeReturnPoint pt{};
            pt.x = range * std::cos(angle) + noise(rng);
            pt.y = range * std::sin(angle) + noise(rng);
            pt.z = z;
            pt.intensity = 0.5f + 0.5f * t;  // brighter at top
            pt.timestamp_ns = now_ns + static_cast<uint64_t>(i) * 1000;
            pt.point_type = 0;
            points.push_back(pt);
        }
        break;
    }
    }

    // --- Set SOF/EOF flags ---
    if (!points.empty()) {
        points.front().point_type |= POINT_TYPE_SOF;
        points.back().point_type |= POINT_TYPE_EOF;
    }

    return points;
}

// ============================================================
//  Packetize and send one frame
// ============================================================

// Splits a frame into packets of at most max_points_per_pkt
// and sends each over UDP. Returns number of packets sent.
int send_frame(
    int sock_fd,
    const sockaddr_in& dest,
    const std::vector<AeyeReturnPoint>& frame,
    uint32_t& sequence_number,
    size_t max_points_per_pkt,
    double drop_rate,
    std::mt19937& rng)
{
    std::uniform_real_distribution<double> drop_dist(0.0, 1.0);

    int packets_sent = 0;
    size_t offset = 0;

    while (offset < frame.size()) {
        size_t count = std::min(max_points_per_pkt, frame.size() - offset);

        // Build packet
        // Only allocate what we need: header + count points
        size_t packet_bytes = PACKET_HEADER_SIZE + count * sizeof(AeyeReturnPoint);
        std::vector<uint8_t> buf(packet_bytes);

        uint32_t seq = sequence_number++;
        auto num = static_cast<uint32_t>(count);

        std::memcpy(buf.data(), &seq, sizeof(uint32_t));
        std::memcpy(buf.data() + sizeof(uint32_t), &num, sizeof(uint32_t));
        std::memcpy(buf.data() + PACKET_HEADER_SIZE,
                    &frame[offset],
                    count * sizeof(AeyeReturnPoint));

        // Simulate packet drops
        if (drop_rate > 0.0 && drop_dist(rng) < drop_rate) {
            // Silently skip this packet
            offset += count;
            continue;
        }

        ssize_t sent = sendto(
            sock_fd,
            buf.data(),
            packet_bytes,
            0,
            reinterpret_cast<const sockaddr*>(&dest),
            sizeof(dest)
        );

        if (sent < 0) {
            std::cerr << "[sender] sendto failed: " << strerror(errno) << "\n";
        } else {
            ++packets_sent;
        }

        offset += count;
    }

    return packets_sent;
}

// ============================================================
//  Usage / help
// ============================================================

void print_usage(const char* prog) {
    std::cout <<
        "Usage: " << prog << " [OPTIONS]\n"
        "\n"
        "Simulates aeye_service by generating LiDAR scan patterns\n"
        "and sending them as UDP point packets.\n"
        "\n"
        "Options:\n"
        "  --dest-ip IP          Destination IP address     [127.0.0.1]\n"
        "  --port PORT           Destination UDP port       [8080]\n"
        "  --fps FPS             Frames per second          [10]\n"
        "  --points N            Points per frame           [50000]\n"
        "  --pkt-size N          Max points per packet      [500]\n"
        "  --pattern NAME        Scan pattern:              [hemisphere]\n"
        "                          hemisphere, flat_plane,\n"
        "                          cube, rotating_line\n"
        "  --drop-rate RATE      Packet drop probability    [0.0]\n"
        "                          0.0 = no drops, 0.1 = 10% drops\n"
        "  --frames N            Total frames (0=infinite)  [0]\n"
        "  --verbose             Print per-frame stats\n"
        "  --help                Show this message\n"
        "\n"
        "Examples:\n"
        "  # Basic: 10 FPS hemisphere to localhost:8080\n"
        "  " << prog << "\n"
        "\n"
        "  # Fast rate, small frames, rotating line\n"
        "  " << prog << " --fps 30 --points 10000 --pattern rotating_line\n"
        "\n"
        "  # Simulate 5%% packet loss\n"
        "  " << prog << " --drop-rate 0.05\n"
        "\n"
        "  # Send exactly 100 frames then exit\n"
        "  " << prog << " --frames 100 --verbose\n";
}

// ============================================================
//  Main
// ============================================================

int main(int argc, char* argv[])
{
    // --- Defaults ---
    std::string dest_ip     = "127.0.0.1";
    uint16_t port           = 8080;
    double fps              = 10.0;
    int points_per_frame    = 50000;
    size_t max_pkt_points   = 500;
    std::string pattern_str = "hemisphere";
    double drop_rate        = 0.0;
    int total_frames        = 0;  // 0 = infinite
    bool verbose            = false;

    // --- Parse command line ---
    static struct option long_opts[] = {
        {"dest-ip",    required_argument, nullptr, 'i'},
        {"port",       required_argument, nullptr, 'p'},
        {"fps",        required_argument, nullptr, 'f'},
        {"points",     required_argument, nullptr, 'n'},
        {"pkt-size",   required_argument, nullptr, 's'},
        {"pattern",    required_argument, nullptr, 'a'},
        {"drop-rate",  required_argument, nullptr, 'd'},
        {"frames",     required_argument, nullptr, 'c'},
        {"verbose",    no_argument,       nullptr, 'v'},
        {"help",       no_argument,       nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "i:p:f:n:s:a:d:c:vh", long_opts, nullptr)) != -1) {
        switch (opt) {
            case 'i': dest_ip = optarg; break;
            case 'p': port = static_cast<uint16_t>(std::stoi(optarg)); break;
            case 'f': fps = std::stod(optarg); break;
            case 'n': points_per_frame = std::stoi(optarg); break;
            case 's': max_pkt_points = static_cast<size_t>(std::stoi(optarg)); break;
            case 'a': pattern_str = optarg; break;
            case 'd': drop_rate = std::stod(optarg); break;
            case 'c': total_frames = std::stoi(optarg); break;
            case 'v': verbose = true; break;
            case 'h': print_usage(argv[0]); return 0;
            default:  print_usage(argv[0]); return 1;
        }
    }

    // Clamp
    if (max_pkt_points > MAX_POINTS_PER_PACKET) {
        max_pkt_points = MAX_POINTS_PER_PACKET;
    }

    ScanPattern pattern = parse_pattern(pattern_str);

    // --- Signal handling ---
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // --- Create UDP socket ---
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        std::cerr << "Failed to create socket: " << strerror(errno) << "\n";
        return 1;
    }

    // Increase send buffer to handle bursts
    int sndbuf = 4 * 1024 * 1024;
    setsockopt(sock_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    inet_pton(AF_INET, dest_ip.c_str(), &dest_addr.sin_addr);

    // --- RNG ---
    std::mt19937 rng(42);  // fixed seed for reproducibility

    // --- Timing ---
    const auto frame_interval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / fps)
    );

    uint32_t sequence_number = 0;
    int frame_index = 0;

    std::cout << "=== Aeye Test Sender ===\n"
              << "  Destination: " << dest_ip << ":" << port << "\n"
              << "  FPS:         " << fps << "\n"
              << "  Points/frame:" << points_per_frame << "\n"
              << "  Points/pkt:  " << max_pkt_points << "\n"
              << "  Pattern:     " << pattern_str << "\n"
              << "  Drop rate:   " << (drop_rate * 100.0) << "%\n"
              << "  Frames:      " << (total_frames == 0 ? "infinite" : std::to_string(total_frames)) << "\n"
              << "  Ctrl+C to stop\n"
              << "========================\n";

    auto next_frame_time = std::chrono::steady_clock::now();

    while (g_running) {
        // Check frame limit
        if (total_frames > 0 && frame_index >= total_frames) {
            break;
        }

        // Generate frame
        auto frame = generate_frame(pattern, points_per_frame, frame_index, rng);

        // Send frame as packets
        int pkts = send_frame(
            sock_fd, dest_addr, frame,
            sequence_number, max_pkt_points,
            drop_rate, rng
        );

        if (verbose) {
            std::cout << "Frame " << frame_index
                      << " | " << frame.size() << " points"
                      << " | " << pkts << " packets"
                      << " | seq " << sequence_number
                      << "\n";
        }

        ++frame_index;

        // Sleep until next frame
        next_frame_time += frame_interval;
        auto now = std::chrono::steady_clock::now();
        if (next_frame_time > now) {
            std::this_thread::sleep_until(next_frame_time);
        } else {
            // We're behind — reset to avoid burst
            if (verbose) {
                std::cerr << "[sender] Frame took too long, resetting timer\n";
            }
            next_frame_time = now;
        }
    }

    close(sock_fd);

    std::cout << "\nSent " << frame_index << " frames, "
              << sequence_number << " packets total.\n";

    return 0;
}
