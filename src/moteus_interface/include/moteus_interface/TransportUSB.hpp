#ifndef MOTEUS_INTERFACE_TRANSPORTUSB_HPP
#define MOTEUS_INTERFACE_TRANSPORTUSB_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <string_view>

#include "moteus.h"
#include "rclcpp/logger.hpp"

#include "Transport.hpp"

namespace moteus_interface::transport
{

class TransportUSB : public Transport
{
public:
    explicit TransportUSB(const std::string device="/dev/fdcanusb");
    virtual ~TransportUSB();
    bool initialize() override;

    bool write(const mjbots::moteus::CanFdFrame *frames, size_t size, uint32_t bus_timeout_us) override;
    bool read(std::vector<mjbots::moteus::CanFdFrame> & replies, uint32_t expected_replies, uint32_t timeout_us=0) override;

    bool cycle(const mjbots::moteus::CanFdFrame *frames, size_t size,
               std::vector<mjbots::moteus::CanFdFrame> & replies,
               uint32_t expected_replies,
               uint32_t timeout_us) override;

private:
    void flush();
    bool parse_line(
        std::string_view line, 
        std::vector<mjbots::moteus::CanFdFrame>& replies);
    size_t round_up_dlc(size_t size);
    int parse_hex_nybble(char c);
    int parse_hex_byte(const char* value);

private:
    bool initialized_;
    int fd_;
    std::string device_;

    char rx_buffer_[4096] = {};
    size_t rx_buffer_pos_ = 0;

    char tx_buffer_[4096] = {};
    size_t tx_buffer_pos_ = 0;
};

}

#endif