#ifndef MOTEUS_INTERFACE_TRANSPORT_HPP
#define MOTEUS_INTERFACE_TRANSPORT_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "moteus.h"
#include "rclcpp/logger.hpp"

#include "CanProtocolTypes.hpp"

namespace moteus_interface::transport
{

class Transport
{
public:
    Transport(rclcpp::Logger logger):logger_(logger) {};
    virtual ~Transport() = default;

    virtual bool initialize() = 0;

    virtual bool write(const mjbots::moteus::CanFdFrame *frames, size_t size, uint32_t bus_timeout_us) = 0;
    virtual bool read(std::vector<mjbots::moteus::CanFdFrame> & replies, uint32_t expected_replies, uint32_t timeout_us=0) = 0;

    virtual bool cycle(const mjbots::moteus::CanFdFrame *frames, size_t size,
               std::vector<mjbots::moteus::CanFdFrame> & replies,
               uint32_t expected_replies,
               uint32_t timeout_us) = 0;

    protected:
        rclcpp::Logger logger_;

};

}

#endif