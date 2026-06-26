#ifndef MOTEUS_INTERFACE_TRANSPORTUDP_HPP
#define MOTEUS_INTERFACE_TRANSPORTUDP_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "moteus.h"
#include "rclcpp/logger.hpp"

#include "CanProtocolTypes.hpp"
#include "Transport.hpp"

namespace moteus_interface::transport
{

class TransportUDP : public Transport
{
public:
    TransportUDP();
    ~TransportUDP();
    bool initialize(const std::string gateway_ip,
                    const uint16_t gateway_port,
                    rclcpp::Logger logger = rclcpp::get_logger("MoteusTransport")) override;

    bool write(const mjbots::moteus::CanFdFrame *frames, size_t size, uint32_t bus_timeout_us) override;
    bool read(std::vector<mjbots::moteus::CanFdFrame> & replies, uint32_t timeout_us=0) override;

    bool cycle(const mjbots::moteus::CanFdFrame *frames, size_t size,
               std::vector<mjbots::moteus::CanFdFrame> & replies,
               uint32_t timeout_us) override;

public:
    static constexpr size_t MAX_FRAMES = 32;
    static constexpr bool kDefaultDisableBrs = false;

private:
    MoteusCanFrame encode_frame(const mjbots::moteus::CanFdFrame& frame);
    mjbots::moteus::CanFdFrame decode_frame(const MoteusCanFrame& wire);
    size_t round_up_dlc(size_t size);

private:
    int socket_fd_ = -1;
    uint32_t gateway_addr_ = 0;
    uint16_t gateway_port_ = 0;
    bool initialized_ = false;

    rclcpp::Logger logger_;

};

}

#endif