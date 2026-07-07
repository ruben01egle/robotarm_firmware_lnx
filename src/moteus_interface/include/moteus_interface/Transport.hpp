#ifndef MOTEUS_INTERFACE_TRANSPORT_HPP
#define MOTEUS_INTERFACE_TRANSPORT_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "moteus.h"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/logger.hpp"

#include "CanProtocolTypes.hpp"
#include "moteus_interface/TransportTiming.hpp"

namespace moteus_interface::transport
{

class Transport
{
public:
    Transport(rclcpp::Logger logger): logger_(logger), timing_(logger) {};
    virtual ~Transport() = default;

    virtual bool declare_and_read_parameters(
        const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& params) = 0;
    virtual bool initialize() = 0;

    virtual bool write(const mjbots::moteus::CanFdFrame *frames, size_t size, uint32_t bus_timeout_us) = 0;
    virtual bool read(std::vector<mjbots::moteus::CanFdFrame> & replies, uint32_t expected_replies, uint32_t timeout_us=0) = 0;

    virtual bool cycle(const mjbots::moteus::CanFdFrame *frames, size_t size,
               std::vector<mjbots::moteus::CanFdFrame> & replies,
               uint32_t expected_replies,
               uint32_t timeout_us) = 0;
               
    void set_timing_enabled(bool enabled, size_t capacity = 20000) { timing_.set_enabled(enabled, capacity); }
    bool timing_enabled() const { return timing_.enabled(); }
    void reset_timing_log() { timing_.reset(); }
    bool dump_timing_log(const std::string& path) const { return timing_.dump(path); }

protected:
    rclcpp::Logger logger_;
    TransportTiming timing_;
};

}

#endif