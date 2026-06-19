#ifndef MOTEUS_INTERFACE_TRANSPORT_HPP
#define MOTEUS_INTERFACE_TRANSPORT_HPP

#include <memory>
#include <vector>

#include "moteus.h"
#include "rclcpp/logger.hpp"

namespace moteus_interface::transport
{

class Transport
{
public:
    Transport();
    ~Transport();
    bool initialize(const std::string& interface_name = "can0",
                   rclcpp::Logger logger = rclcpp::get_logger("MoteusTransport"));

    bool write(const mjbots::moteus::CanFdFrame *frames, size_t size);
    bool read(std::vector<mjbots::moteus::CanFdFrame> & replies);

    bool cycle(const mjbots::moteus::CanFdFrame *frames, size_t size,
               std::vector<mjbots::moteus::CanFdFrame> & replies,
               int timeout_us);

private:
    size_t round_up_dlc(size_t size);

private:
    int socket_fd_;
    rclcpp::Logger logger_;

    static constexpr size_t MAX_FRAMES = 32;
};

}

#endif