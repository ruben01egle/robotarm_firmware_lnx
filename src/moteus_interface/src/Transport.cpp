#include "moteus_interface/Transport.hpp"

#include "rclcpp/logging.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <cerrno>
#include <cstring>

constexpr size_t kMaxWireFrames = moteus_interface::transport::Transport::MAX_FRAMES;
constexpr size_t kMaxTxPayload = sizeof(UdpRequestHeader) + kMaxWireFrames * sizeof(MoteusCanFrame);
constexpr size_t kMaxRxPayload = kMaxWireFrames * sizeof(MoteusCanFrame);

moteus_interface::transport::Transport::Transport(): socket_fd_(-1), logger_(rclcpp::get_logger("MoteusTransport"))
{
}

moteus_interface::transport::Transport::~Transport()
{
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool moteus_interface::transport::Transport::initialize(
    const std::string gateway_ip,
    const uint16_t gateway_port,
    rclcpp::Logger logger)
{
    logger_ = logger;

      if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }

      in_addr addr = {};
    if (::inet_pton(AF_INET, gateway_ip.c_str(), &addr) != 1) {
    RCLCPP_ERROR(logger_, "Transport: invalid bridge IP address '%s'", gateway_ip.c_str());
        initialized_ = false;
        return false;
    }
    gateway_addr_ = addr.s_addr;
    gateway_port_ = gateway_port;

    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Transport: failed to create UDP socket: %s",
                    std::strerror(errno));
        return false;
    }

    int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0) {
        RCLCPP_ERROR(logger_, "Transport: failed to get socket flags: %s", std::strerror(errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    if (::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        RCLCPP_ERROR(logger_, "Transport: failed to set O_NONBLOCK: %s", std::strerror(errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

     RCLCPP_INFO(logger_, "Transport: initialized, gateway=%s:%u",
              gateway_ip.c_str(), static_cast<unsigned>(gateway_port));
    initialized_ = true;
    return true;
}

bool moteus_interface::transport::Transport::write(
        const mjbots::moteus::CanFdFrame *frames,
        size_t size,
        uint32_t timeout_us) 
{
    if (!initialized_ || socket_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Transport: write() called before initialize()");
        return false;
    }
    if (size == 0) { return true; }

    if (size > kMaxWireFrames) {
        RCLCPP_ERROR(logger_,
                    "Transport: write() with %zu frames exceeds MAX_FRAMES (%zu)",
                    size, kMaxWireFrames);
        return false;
    }

    uint32_t expected_replies = 0;
    for (size_t i = 0; i < size; ++i) {
        if (frames[i].reply_required) { ++expected_replies; }
    }

    uint8_t tx_buf[kMaxTxPayload];
    auto* header = reinterpret_cast<UdpRequestHeader*>(tx_buf);

    header->timeoutUs = static_cast<uint32_t>(timeout_us);
    header->expectedReplies = expected_replies;
    header->frameCount = static_cast<uint32_t>(size);

    auto* wire_frames = reinterpret_cast<MoteusCanFrame*>(tx_buf + sizeof(UdpRequestHeader));
    for (size_t i = 0; i < size; ++i) {
        wire_frames[i] = encode_frame(frames[i]);
    }

    const size_t tx_size = sizeof(UdpRequestHeader) + size * sizeof(MoteusCanFrame);

    sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(gateway_port_);
    dest.sin_addr.s_addr = gateway_addr_;

    const ssize_t sent = ::sendto(
        socket_fd_, tx_buf, tx_size, 0,
        reinterpret_cast<const sockaddr*>(&dest), sizeof(dest));
    if (sent < 0 || static_cast<size_t>(sent) != tx_size) {
        RCLCPP_ERROR(logger_, "Transport: sendto failed: %s", std::strerror(errno));
        return false;
    }
    
    return true;
}

bool moteus_interface::transport::Transport::read(
    std::vector<mjbots::moteus::CanFdFrame> & replies,
    uint32_t timeout_us)
{
    if (timeout_us > 0) {
        struct pollfd pfd = {};
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;

        struct timespec timeout_ts = {};
        timeout_ts.tv_sec = timeout_us / 1000000;
        timeout_ts.tv_nsec = (timeout_us % 1000000) * 1000;

        int poll_result = ::ppoll(&pfd, 1, &timeout_ts, nullptr);

        if (poll_result < 0)
        {
            RCLCPP_ERROR(logger_, "Read failed: System error during ppoll. Error: %s", ::strerror(errno));
            return false;
        }
        else if (poll_result == 0)
        {
            RCLCPP_ERROR(logger_, "Read timeout: Total budget of %d us expired.", timeout_us);
            return false;
        }
    }

    uint8_t rx_buf[kMaxRxPayload];
    const ssize_t received = ::recvfrom(
        socket_fd_, rx_buf, sizeof(rx_buf), 0, nullptr, nullptr);
    if (received < 0) {
        RCLCPP_ERROR(logger_, "Transport: recvfrom failed: %s", std::strerror(errno));
        return false;
    }
    else if (received == 0) return true;

    if (static_cast<size_t>(received) % sizeof(MoteusCanFrame) != 0) {
        RCLCPP_ERROR(logger_,
                    "Transport: malformed response (%zd bytes, not a multiple "
                    "of %zu)",
                    received, sizeof(MoteusCanFrame));
        return false;
    }

    const size_t reply_count = static_cast<size_t>(received) / sizeof(MoteusCanFrame);
    const auto* wire_replies = reinterpret_cast<const MoteusCanFrame*>(rx_buf);

    for (size_t i = 0; i < reply_count; ++i) {
        replies.push_back(decode_frame(wire_replies[i]));
    }

    return true;
}

bool moteus_interface::transport::Transport::cycle(
    const mjbots::moteus::CanFdFrame *frames,
    size_t size, std::vector<mjbots::moteus::CanFdFrame> &replies,
    uint32_t timeout_us)
{
    char dummy;
    while (::recvfrom(socket_fd_, &dummy, 1, MSG_DONTWAIT | MSG_TRUNC, nullptr, nullptr) >= 0) {
        // clear stale messages
    }
    replies.clear();

    static constexpr uint32_t timeout_network_us = 500; 
    uint32_t timeout_bus_us;
    if (timeout_us <= timeout_network_us) timeout_bus_us = 0;
    else timeout_bus_us = timeout_us - timeout_network_us;

    if (!write(frames, size, timeout_bus_us)) return false;

    if (!read(replies, timeout_us)) return false;

    return true;
}

MoteusCanFrame moteus_interface::transport::Transport::encode_frame(const mjbots::moteus::CanFdFrame &frame)
{
    MoteusCanFrame wire = {};
    wire.id = frame.arbitration_id;
    
    uint8_t flags = FLAG_IS_EXTID;
    
    const bool want_brs =
        (frame.brs == mjbots::moteus::CanFdFrame::kDefault ? !kDefaultDisableBrs :
        frame.brs == mjbots::moteus::CanFdFrame::kForceOn ? true : false);
    
    const bool want_fd =
        (frame.fdcan_frame == mjbots::moteus::CanFdFrame::kDefault ||
        frame.fdcan_frame == mjbots::moteus::CanFdFrame::kForceOn);
    
    if (want_fd) {
        flags |= FLAG_IS_CANFD;
        if (want_brs) { flags |= FLAG_USE_BRS; }
    }
    wire.flags = flags;
    
    const size_t payload_size = frame.size;
    ::memcpy(wire.data, frame.data, payload_size);
    
    if (want_fd) {
        const size_t padded_size = round_up_dlc(payload_size);
        for (size_t i = payload_size; i < padded_size; ++i) {
            wire.data[i] = 0x50;
            }
        wire.len = static_cast<uint8_t>(padded_size);
    } else {
        wire.len = static_cast<uint8_t>(payload_size);
    }
    
    return wire;
}

mjbots::moteus::CanFdFrame moteus_interface::transport::Transport::decode_frame(const MoteusCanFrame &wire)
{
    using namespace mjbots::moteus;
    CanFdFrame frame;
    frame.arbitration_id = wire.id;
    frame.destination = static_cast<int8_t>(wire.id & 0x7f);
    frame.source = static_cast<int8_t>((wire.id >> 8) & 0x7f);
    frame.can_prefix = static_cast<uint16_t>(wire.id >> 16);
    frame.bus = 0;
    
    frame.size = wire.len;
    ::memcpy(frame.data, wire.data, wire.len);
    
    if (wire.flags & FLAG_IS_CANFD) {
        frame.fdcan_frame = CanFdFrame::kForceOn;
        frame.brs = (wire.flags & FLAG_USE_BRS)
                        ? CanFdFrame::kForceOn
                        : CanFdFrame::kForceOff;
    } else {
        frame.fdcan_frame = CanFdFrame::kForceOff;
        frame.brs = CanFdFrame::kForceOff;
    }
    
    return frame;
}

size_t moteus_interface::transport::Transport::round_up_dlc(size_t size)
{
    if (size <= 8) { return size; }
    if (size <= 12) { return 12; }
    if (size <= 16) { return 16; }
    if (size <= 20) { return 20; }
    if (size <= 24) { return 24; }
    if (size <= 32) { return 32; }
    if (size <= 48) { return 48; }
    if (size <= 64) { return 64; }
    return size;
}
