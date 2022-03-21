#ifndef GPS_BASE_RTCM3_HPP
#define GPS_BASE_RTCM3_HPP

#include <cstdint>

namespace gps_base {
    /** Implementation of RTCM packet extraction
     *
     * RTCM is the de-factor standard encapsulation for correction messages
     */
    namespace rtcm3 {
        static const std::uint8_t PREAMBLE = 0xD3;
        static const int PREAMBLE_SIZE = 1;
        static const int HEADER_SIZE = 3;
        static const int CRC_SIZE = 3;
        static const int MIN_PACKET_SIZE = HEADER_SIZE + CRC_SIZE;

        bool isPreamble(std::uint8_t const* buffer, std::size_t size);
        std::uint16_t getLength(std::uint8_t const* buffer, std::size_t size);
        std::uint32_t crc(std::uint8_t const* packetStart, std::size_t size);
        int extractPacket(std::uint8_t const* buffer, std::size_t size);
    }
}

#endif
