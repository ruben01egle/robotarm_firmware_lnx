#pragma once
#include <cstdint>

#pragma pack(push, 1)

enum MoteusFrameFlags : uint8_t {
    FLAG_NONE       = 0x00,
    FLAG_IS_CANFD   = 0x01,  // 1 = CAN-FD, 0 = CAN
    FLAG_USE_BRS    = 0x02,  // 1 = Bit-Rate-Switch on, 0 = off
    FLAG_IS_EXTID   = 0x04   // 1 = Extended ID (29 Bit), 0 = Standard ID (11 Bit)
};

struct MoteusCanFrame {
    uint32_t id;
    uint8_t  len;
    uint8_t  flags;
    uint8_t  data[64];
};

struct UdpRequestHeader {
    uint32_t timeoutUs;
    uint32_t expectedReplies;
    uint32_t frameCount;
};

#pragma pack(pop)