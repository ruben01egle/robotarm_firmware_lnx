#include "moteus_interface/TransportUSB.hpp"

#include "rclcpp/logging.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <linux/serial.h>
#include <time.h>
#include <algorithm>

inline uint64_t dt_us(const struct timespec& start, const struct timespec& end) {
    return ((end.tv_sec - start.tv_sec) * 1000000ULL) + 
           ((end.tv_nsec - start.tv_nsec) / 1000ULL);
}

moteus_interface::transport::TransportUSB::TransportUSB(const std::string device):
    Transport(rclcpp::get_logger("MoteusTransport")),
    initialized_(false),
    fd_(-1),
    device_(device)
{
}

moteus_interface::transport::TransportUSB::~TransportUSB()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool moteus_interface::transport::TransportUSB::initialize()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }

    if (device_.empty() || device_ == "auto") {
        RCLCPP_ERROR(logger_, "Transport: Device empty '%s'", 
                    device_.c_str());
        return false;
    }

    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        RCLCPP_ERROR(logger_, "Transport: Failed to open serial device '%s': %s. "
                             "Is the device plugged in and does the udev rule exist?", 
                    device_.c_str(), std::strerror(errno));
        initialized_ = false;
        return false;
    }

    struct serial_struct serial;
    if (::ioctl(fd_, TIOCGSERIAL, &serial) < 0) {
        RCLCPP_ERROR(logger_, "Transport: ioctl(TIOCGSERIAL) failed: %s", std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    serial.flags |= ASYNC_LOW_LATENCY;
    
    if (::ioctl(fd_, TIOCSSERIAL, &serial) < 0) {
        RCLCPP_ERROR(logger_, "Transport: ioctl(TIOCSSERIAL) failed: %s", std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    struct termios toptions;
    if (::tcgetattr(fd_, &toptions) < 0) {
        RCLCPP_ERROR(logger_, "Transport: tcgetattr failed: %s", std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    toptions.c_oflag &= ~OPOST;
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
    toptions.c_cflag |= (CLOCAL | CREAD);
    toptions.c_cflag &= ~CRTSCTS;

    if (::tcsetattr(fd_, TCSANOW, &toptions) < 0) {
        RCLCPP_ERROR(logger_, "Transport: tcsetattr(TCSANOW) failed: %s", std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    if (::tcsetattr(fd_, TCSAFLUSH, &toptions) < 0) {
        RCLCPP_ERROR(logger_, "Transport: tcsetattr(TCSAFLUSH) failed: %s", std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    RCLCPP_INFO(logger_, "Transport: Successfully opened and configured '%s'", device_.c_str());
    initialized_ = true;
    return true;
}

bool moteus_interface::transport::TransportUSB::write(const mjbots::moteus::CanFdFrame *frames, size_t size, uint32_t /*bus_timeout_us*/)
{
    if (!initialized_ || fd_ < 0) {
        RCLCPP_ERROR(logger_, "Transport: write() called but device is not initialized!");
        return false;
    }
    if (size == 0) { return true; }

    flush();
    tx_buffer_pos_ = 0;

    for (size_t i = 0; i < size; ++i) {
        const auto& frame = frames[i];

        if (sizeof(tx_buffer_) - tx_buffer_pos_ < 200) { 
            RCLCPP_ERROR(logger_, "Transport: Tx buffer overflow prevention triggered!");
            return false;
        }

        tx_buffer_pos_ += std::snprintf(tx_buffer_ + tx_buffer_pos_, sizeof(tx_buffer_) - tx_buffer_pos_, 
                                "can send %04x ", frame.arbitration_id);

        const size_t dlc = round_up_dlc(frame.size);
        for (size_t j = 0; j < frame.size; ++j) {
            tx_buffer_pos_ += std::snprintf(tx_buffer_ + tx_buffer_pos_, sizeof(tx_buffer_) - tx_buffer_pos_, "%02x", frame.data[j]);
        }
        for (size_t j = frame.size; j < dlc; ++j) {
            tx_buffer_pos_ += std::snprintf(tx_buffer_ + tx_buffer_pos_, sizeof(tx_buffer_) - tx_buffer_pos_, "50");
        }

        if (frame.brs == mjbots::moteus::CanFdFrame::kForceOn) {
            tx_buffer_pos_ += std::snprintf(tx_buffer_ + tx_buffer_pos_, sizeof(tx_buffer_) - tx_buffer_pos_, " B");
        } else if (frame.brs == mjbots::moteus::CanFdFrame::kForceOff) {
            tx_buffer_pos_ += std::snprintf(tx_buffer_ + tx_buffer_pos_, sizeof(tx_buffer_) - tx_buffer_pos_, " b");
        }

        if (frame.fdcan_frame == mjbots::moteus::CanFdFrame::kForceOn) {
            tx_buffer_pos_ += std::snprintf(tx_buffer_ + tx_buffer_pos_, sizeof(tx_buffer_) - tx_buffer_pos_, " F");
        } else if (frame.fdcan_frame == mjbots::moteus::CanFdFrame::kForceOff) {
            tx_buffer_pos_ += std::snprintf(tx_buffer_ + tx_buffer_pos_, sizeof(tx_buffer_) - tx_buffer_pos_, " f");
        }

        tx_buffer_pos_ += std::snprintf(tx_buffer_ + tx_buffer_pos_, sizeof(tx_buffer_) - tx_buffer_pos_, "\n");
    }

    size_t written = 0;
    while (written < static_cast<size_t>(tx_buffer_pos_)) {
        int ret = ::write(fd_, tx_buffer_ + written, tx_buffer_pos_ - written);
        if (ret < 0) {
            if (errno == EINTR || errno == EAGAIN) { continue; }
            RCLCPP_ERROR(logger_, "Transport: Fatal error writing all frames to serial port: %s", std::strerror(errno));
            return false;
        }
        written += ret;
    }

    return true;
}

bool moteus_interface::transport::TransportUSB::read(std::vector<mjbots::moteus::CanFdFrame> &replies, uint32_t expected_replies, uint32_t timeout_us)
{
    if (!initialized_ || fd_ < 0) { return false; }
    if (expected_replies == 0) { return true; }

    struct timespec start_time;
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);

    struct pollfd pfd = {};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    size_t receieved_replies = 0;

    do {
        const size_t to_read = sizeof(rx_buffer_) - rx_buffer_pos_;
        if (to_read == 0) {
            RCLCPP_WARN(logger_, "Transport: Line buffer overflow, resetting buffer.");
            rx_buffer_pos_ = 0;
            continue; 
        }

        int read_ret = ::read(fd_, &rx_buffer_[rx_buffer_pos_], to_read);
        
        if (read_ret > 0) {
            rx_buffer_pos_ += read_ret;

            size_t scan_pos = 0;
            while (scan_pos < rx_buffer_pos_) {
                if (rx_buffer_[scan_pos] == '\r' || rx_buffer_[scan_pos] == '\n') {
                    if (scan_pos > 0) { 
                        std::string_view line(&rx_buffer_[0], scan_pos);
                        if (parse_line(line, replies)) {
                            receieved_replies++;
                        }
                    }
                    std::memmove(&rx_buffer_[0], &rx_buffer_[scan_pos + 1], rx_buffer_pos_ - (scan_pos + 1));
                    rx_buffer_pos_ -= (scan_pos + 1);
                    scan_pos = 0; 
                } else {
                    scan_pos++;
                }
            }
        }
        else if (read_ret < 0) {
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(logger_, "Transport: Fatal read error: %s", std::strerror(errno));
                return false;
            }
        }
        else if (read_ret == 0) {
            RCLCPP_ERROR(logger_, "Transport: Device unplugged / connection closed.");
            initialized_ = false;
            return false;
        }
        
        if (receieved_replies >= expected_replies) {
            return true;
        }

        if (timeout_us == 0) {
            return true; 
        }

        struct timespec current_time;
        ::clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);
        uint32_t elapsed_us = static_cast<uint32_t>(dt_us(start_time, current_time));
        
        if (elapsed_us >= timeout_us) {
            RCLCPP_WARN(logger_,
                "Transport: Read timeout. Got %zu of %zu replies.", receieved_replies, expected_replies);
            return true;
        }

        uint32_t remaining_us = timeout_us - elapsed_us;
        struct timespec timeout_ts = {};
        timeout_ts.tv_sec = remaining_us / 1000000;
        timeout_ts.tv_nsec = (remaining_us % 1000000) * 1000;

        int poll_result = ::ppoll(&pfd, 1, &timeout_ts, nullptr);
        if (poll_result < 0) {
            if (errno == EINTR) { continue; }
            RCLCPP_ERROR(logger_, "Read fatal system error during ppoll: %s", ::strerror(errno));
            return false; 
        }
        else if (poll_result == 0) {
            return true;
        }

    } while (true);

    return true;
}

bool moteus_interface::transport::TransportUSB::cycle(
            const mjbots::moteus::CanFdFrame *frames,
            size_t size,
            std::vector<mjbots::moteus::CanFdFrame> &replies,
            uint32_t expected_replies,
            uint32_t timeout_us)
{
    flush();
    if (!write(frames, size, 0)) return false;

    if (!read(replies, expected_replies, timeout_us)) return false;

    return true;
}

void moteus_interface::transport::TransportUSB::flush()
{
    if (!initialized_ || fd_ < 0) { return; }

    ::tcflush(fd_, TCIFLUSH);

    rx_buffer_pos_ = 0;
}

bool moteus_interface::transport::TransportUSB::parse_line(
    std::string_view line, 
    std::vector<mjbots::moteus::CanFdFrame>& replies)
{
    if (line.size() < 10 || line.substr(0, 4) != "rcv ") {
        return false; 
    }

    size_t first_space = 3;
    size_t second_space = line.find(' ', first_space + 1);
    
    if (second_space == std::string_view::npos) {
        return false;
    }

    size_t third_space = line.find(' ', second_space + 1);

    std::string_view addr_str = line.substr(first_space + 1, second_space - (first_space + 1));
    std::string_view data_str = (third_space == std::string_view::npos) ? 
                                 line.substr(second_space + 1) : 
                                 line.substr(second_space + 1, third_space - (second_space + 1));

    if (addr_str.empty() || data_str.empty()) {
        return false;
    }

    // stoul uses string -> generally speaking not RT loop safe
    // but string is very short -> sso (optimization to avoid heap for small strings)
    //uint32_t arbitration_id = 0;
    //arbitration_id = std::stoul(std::string(addr_str), nullptr, 16);
    // TODO: validate
    uint32_t arbitration_id = 0;
    for (char c : addr_str) {
        int n = parse_hex_nybble(c);
        if (n < 0) return false;
        arbitration_id = (arbitration_id << 4) | static_cast<uint32_t>(n);
    }

    mjbots::moteus::CanFdFrame frame;
    frame.arbitration_id = arbitration_id;
    frame.destination = arbitration_id & 0x7f;
    frame.source = (arbitration_id >> 8) & 0x7f;
    frame.can_prefix = (arbitration_id >> 16);

    size_t to_read = std::min<size_t>(64 * 2, data_str.size());
    for (size_t i = 0; i < to_read; i += 2) {
        frame.data[i / 2] = parse_hex_byte(&data_str[i]);
    }
    frame.size = to_read / 2;

    if (third_space != std::string_view::npos) {
        std::string_view flags = line.substr(third_space + 1);
        for (char c : flags) {
            if (c == 'b') frame.brs = mjbots::moteus::CanFdFrame::kForceOff;
            if (c == 'B') frame.brs = mjbots::moteus::CanFdFrame::kForceOn;
            if (c == 'f') frame.fdcan_frame = mjbots::moteus::CanFdFrame::kForceOff;
            if (c == 'F') frame.fdcan_frame = mjbots::moteus::CanFdFrame::kForceOn;
        }
    }

    replies.push_back(frame);
    return true;
}

size_t moteus_interface::transport::TransportUSB::round_up_dlc(size_t size)
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

int moteus_interface::transport::TransportUSB::parse_hex_nybble(char c) {
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    return -1;
}

int moteus_interface::transport::TransportUSB::parse_hex_byte(const char* value) {
    const int high = parse_hex_nybble(value[0]);
    if (high < 0) { return high; }
    const int low = parse_hex_nybble(value[1]);
    if (low < 0) { return low; }
    return (high << 4) | low;
}
