#ifndef _INPUT_LIB_WW_CRC_HPP_
#define _INPUT_LIB_WW_CRC_HPP_

#include <cstdint>
#include <cstdlib>

namespace ww {
namespace crc {

    uint16_t Encode(const uint8_t* data, size_t sz);
    bool Verify(const uint8_t* data, size_t sz, uint16_t crc_code);

} // namespace crc
} // namespace ww

#endif
