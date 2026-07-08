#ifndef MOTEUS_INTERFACE_TRANSPORTUDP_HPP
#define MOTEUS_INTERFACE_TRANSPORTUDP_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "moteus.h"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/logger.hpp"

#include "CanProtocolTypes.hpp"
#include "Transport.hpp"

namespace moteus_interface::transport
{

class TransportUDP : public Transport
{
public:
    TransportUDP();
    virtual ~TransportUDP();

    bool declare_and_read_parameters(
        const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& params) override;
    bool initialize() override;

    bool write(const mjbots::moteus::CanFdFrame *frames, size_t size, uint32_t timeout_us) override;
    bool read(std::vector<mjbots::moteus::CanFdFrame> & replies, uint32_t expected_replies, uint32_t timeout_us=0) override;

    bool cycle(const mjbots::moteus::CanFdFrame *frames, size_t size,
               std::vector<mjbots::moteus::CanFdFrame> & replies,
               uint32_t expected_replies,
               uint32_t timeout_us) override;

public:
    static constexpr size_t MAX_FRAMES = 12;
    static constexpr bool kDefaultDisableBrs = false;

private:
    MoteusCanFrame encode_frame(const mjbots::moteus::CanFdFrame& frame);
    mjbots::moteus::CanFdFrame decode_frame(const MoteusCanFrame& wire);
    size_t round_up_dlc(size_t size);

private:
    int socket_fd_ = -1;
    uint32_t network_timeout_us_ = 0;
    std::string gateway_ip_; 
    uint32_t gateway_addr_ = 0;
    uint16_t gateway_port_ = 0;
    bool initialized_ = false;

    uint32_t last_sequence_recieved_ = 0;
};

}

#endif