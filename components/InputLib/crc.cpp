#include "crc.hpp"
#include "cppcrc/cppcrc.h"


namespace ww {
namespace crc {

    uint16_t Encode(const uint8_t* data, size_t sz) {
        return CRC16::CCITT_FALSE::calc(data, sz);
    }

    bool Verify(const uint8_t* data, size_t sz, uint16_t crc_code) {
        return Encode(data, sz) == crc_code;
    }

} // namespace crc
} // namespace ww
