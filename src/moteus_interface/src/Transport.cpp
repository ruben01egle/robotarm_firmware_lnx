#include "moteus_interface/Transport.hpp"

#include "rclcpp/logging.hpp"
#include <poll.h>

moteus_interface::transport::Transport::Transport(): socket_fd_(-1), logger_(rclcpp::get_logger("MoteusTransport"))
{
}

moteus_interface::transport::Transport::~Transport()
{
    if (socket_fd_ >= 0)
    {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool moteus_interface::transport::Transport::initialize(
    const std::string &interface_name,
    rclcpp::Logger logger)
{
    logger_ = logger;

    if (socket_fd_ >= 0)
    {
        RCLCPP_ERROR(logger_, "Socket is already open (FD: %d). Close it before re-initializing.", socket_fd_);
        return false;
    }

    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
        RCLCPP_FATAL(logger_, "Failed to create SocketCAN raw socket! Error: %s", ::strerror(errno));
        return false;
    }

    int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        RCLCPP_FATAL(logger_, "Failed to set O_NONBLOCK on socket! Error: %s", ::strerror(errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    int enable_canfd = 1;
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) != 0)
    {
        RCLCPP_FATAL(logger_, "Failed to enable CAN-FD on socket! Error: %s", ::strerror(errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct ifreq ifr = {};
    std::strncpy(ifr.ifr_name, interface_name.c_str(), sizeof(ifr.ifr_name) - 1);
    
    if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_FATAL(logger_, "Failed to find CAN interface '%s'! Is the interface up? Error: %s", 
                     interface_name.c_str(), ::strerror(errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_FATAL(logger_, "Failed to bind socket to interface '%s' (Index: %d)! Error: %s", 
                     interface_name.c_str(), ifr.ifr_ifindex, ::strerror(errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully initialized native SocketCAN transport layer on interface '%s'.", 
                interface_name.c_str());
    return true;
}

bool moteus_interface::transport::Transport::write(const mjbots::moteus::CanFdFrame *frames, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        struct canfd_frame send_frame = {};
        
        send_frame.can_id = frames[i].arbitration_id;
        if (send_frame.can_id >= 0x7ff) {
            send_frame.can_id |= (1 << 31);
        }
        
        send_frame.len = round_up_dlc(frames[i].size);
        std::memcpy(send_frame.data, frames[i].data, frames[i].size);
        if (send_frame.len != frames[i].size) {
            std::memset(&send_frame.data[frames[i].size], 0x50, send_frame.len - frames[i].size);
        }
        
        send_frame.flags = CANFD_FDF | CANFD_BRS;
        
        if (::write(socket_fd_, &send_frame, sizeof(send_frame)) < 0) {
            RCLCPP_ERROR(logger_, "Failed to write frame for ID 0x%X! Error: %s", 
                         frames[i].arbitration_id, ::strerror(errno));
            return false;
        }
    }
    return true;
}

bool moteus_interface::transport::Transport::read(std::vector<mjbots::moteus::CanFdFrame> & replies)
{
    struct canfd_frame raw_frames[MAX_FRAMES] = {};
    struct iovec iov[MAX_FRAMES] = {};
    struct mmsghdr msgs[MAX_FRAMES] = {};

    for (size_t i = 0; i < MAX_FRAMES; ++i)
    {
        iov[i].iov_base = &raw_frames[i];
        iov[i].iov_len = sizeof(struct canfd_frame);
        
        msgs[i].msg_hdr.msg_iov = &iov[i];
        msgs[i].msg_hdr.msg_iovlen = 1;
    }

    int frames_received = ::recvmmsg(socket_fd_, msgs, MAX_FRAMES, MSG_DONTWAIT, nullptr);


    if (frames_received < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return true;
        }

        RCLCPP_ERROR(logger_, "Error reading from SocketCAN! Error: %s", ::strerror(errno));
        return false;
    }

    for (int i = 0; i < frames_received; ++i)
    {
        if (msgs[i].msg_len < sizeof(struct canfd_frame)) {
            RCLCPP_WARN(logger_, "Recieved incomplete can frame");
            continue;
        }
        mjbots::moteus::CanFdFrame this_frame;
         struct canfd_frame& recv_frame = raw_frames[i]; 
        
        this_frame.arbitration_id = recv_frame.can_id & 0x1fffffff;
        this_frame.destination = this_frame.arbitration_id & 0x7f;
        this_frame.source = (this_frame.arbitration_id >> 8) & 0x7f;
        this_frame.can_prefix = (this_frame.arbitration_id >> 16);

        using F = mjbots::moteus::CanFdFrame;
        this_frame.brs = (recv_frame.flags & CANFD_BRS) ? F::kForceOn : F::kForceOff;
        this_frame.fdcan_frame = (recv_frame.flags & CANFD_FDF) ? F::kForceOn : F::kForceOff;

        this_frame.size = recv_frame.len;
        if (this_frame.size > 64) this_frame.size = 64;
        std::memcpy(this_frame.data, recv_frame.data, this_frame.size);

        replies.push_back(this_frame);
    }

    return true;
}

bool moteus_interface::transport::Transport::cycle(
    const mjbots::moteus::CanFdFrame *frames,
    size_t size, std::vector<mjbots::moteus::CanFdFrame> &replies,
    int timeout_us)
{
    std::vector<mjbots::moteus::CanFdFrame> stale_replies;
    read(stale_replies);
    replies.clear();

    if (!write(frames, size)) return false;

    size_t expected_replies = 0;
    for (size_t i = 0; i < size; ++i) {
        if (frames[i].reply_required) { expected_replies++; }
    }
    if (expected_replies == 0) return true;

    struct pollfd pfd = {};
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;

    auto start_time = std::chrono::steady_clock::now();
    int remaining_us = timeout_us;

    while (replies.size() < expected_replies)
    {
        if (remaining_us <= 0)
        {
            RCLCPP_ERROR(logger_, "Cycle timeout: Total budget of %d us depleted. Got %zu/%zu replies.",
                         timeout_us, replies.size(), expected_replies);
            return false;
        }

        struct timespec timeout_ts = {};
        timeout_ts.tv_sec = remaining_us / 1000000;
        timeout_ts.tv_nsec = (remaining_us % 1000000) * 1000;

        int poll_result = ::ppoll(&pfd, 1, &timeout_ts, nullptr);

        if (poll_result < 0)
        {
            if (errno == EINTR) continue; 
            RCLCPP_ERROR(logger_, "Cycle failed: System error during ppoll. Error: %s", ::strerror(errno));
            return false;
        }
        else if (poll_result == 0)
        {
            RCLCPP_ERROR(logger_, "Cycle timeout: Total budget of %d us expired. Got %zu/%zu replies.",
                         timeout_us, replies.size(), expected_replies);
            return false;
        }

        if (pfd.revents & POLLIN)
        {
            if (!read(replies)) return false;
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
        remaining_us = timeout_us - static_cast<int>(elapsed_us);
    }

    return true;
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
