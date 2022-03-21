#include <boost/test/unit_test.hpp>
#include <gps_base/rtcm3.hpp>

#include <fstream>

using namespace gps_base;
using namespace std;

BOOST_AUTO_TEST_CASE(isPreamble_returns_true_if_the_first_buffer_byte_is_a_RTCM3_preamble) {
    uint8_t preamble = 0xD3;
    BOOST_TEST(rtcm3::isPreamble(&preamble, 1));
}

BOOST_AUTO_TEST_CASE(isPreamble_returns_false_if_the_first_buffer_byte_is_a_RTCM3_preamble) {
    uint8_t preamble = 0x42;
    BOOST_TEST(!rtcm3::isPreamble(&preamble, 1));
}

BOOST_AUTO_TEST_CASE(isPreamble_throws_if_the_buffer_is_empty) {
    uint8_t* buffer = nullptr;
    BOOST_REQUIRE_THROW(rtcm3::isPreamble(buffer, 0), std::length_error);
}

BOOST_AUTO_TEST_CASE(getLength_returns_the_payload_length) {
    std::vector<uint8_t> buffer = { 0xD3, 0b00000010, 0x13 };
    BOOST_TEST(rtcm3::getLength(buffer.data(), 3) == 531) ;

}

BOOST_AUTO_TEST_CASE(getLength_throws_if_the_buffer_is_smaller_than_3) {
    std::vector<uint8_t> buffer = { 0xD3, 0b00000010 };
    BOOST_REQUIRE_THROW(rtcm3::getLength(buffer.data(), 2), std::length_error) ;
}

BOOST_AUTO_TEST_CASE(crc_computes_the_RTCM3_crc) {
    const std::vector<uint8_t> buffer =
    {
        0xd3, 0x00, 0x13, 0x3e, 0xd0, 0x00, 0x02, 0x36,
        0xfd, 0xb8, 0x0d, 0xde, 0x08, 0x00, 0x5b, 0x2b,
        0xc1, 0x08, 0xa7, 0xb9, 0x8d, 0x3d
    };
    BOOST_TEST(rtcm3::crc(buffer.data(), 22) == 0xD8AB37);
}

BOOST_AUTO_TEST_CASE(extractPacket_returns_0_if_the_buffer_is_empty) {
    uint8_t* buffer = nullptr;
    BOOST_TEST(rtcm3::extractPacket(buffer, 0) == 0);
}

BOOST_AUTO_TEST_CASE(extractPacket_returns_0_if_the_first_buffer_byte_is_a_RTCM3_preamble_and_the_buffer_is_smaller_than_the_smallest_possible_packet) {
    // There is no reason to access the rest of the buffer here
    // Use a std::vector to let valgrind and asan tell us about out-of-memory accesses
    std::vector<uint8_t> buffer = { 0xD3 };
    BOOST_TEST(rtcm3::extractPacket(buffer.data(), 5) == 0);
}

BOOST_AUTO_TEST_CASE(extractPacket_returns_0_if_the_first_buffer_byte_is_a_RTCM3_preamble_and_the_buffer_is_smaller_than_the_packet_length) {
    // There is no reason to access the rest of the buffer here
    // Use a std::vector to let valgrind and asan tell us about out-of-memory accesses
    std::vector<uint8_t> buffer = { 0xD3, 0b00000010, 0x13 };
    BOOST_TEST(rtcm3::extractPacket(buffer.data(), 10) == 0) ;
}

BOOST_AUTO_TEST_CASE(extractPacket_returns_minus_one_if_the_crc_fails) {
    const std::vector<uint8_t> buffer =
    {
        0xd3, 0x00, 0x13, 0x3e, 0xd0, 0x00, 0x02, 0x36,
        0xfd, 0xb8, 0x0d, 0xde, 0x08, 0x00, 0x5b, 0x2b,
        0xc1, 0x08, 0xa7, 0xb9, 0x8d, 0x3d, 0xd7, 0xab, 0x37
    };
    BOOST_TEST(rtcm3::extractPacket(buffer.data(), 25) == -1);
}

BOOST_AUTO_TEST_CASE(extractPacket_returns_the_full_packet_length_if_the_crc_passes) {
    const std::vector<uint8_t> buffer =
    {
        0xd3, 0x00, 0x13, 0x3e, 0xd0, 0x00, 0x02, 0x36,
        0xfd, 0xb8, 0x0d, 0xde, 0x08, 0x00, 0x5b, 0x2b,
        0xc1, 0x08, 0xa7, 0xb9, 0x8d, 0x3d, 0xd8, 0xab, 0x37
    };
    BOOST_TEST(rtcm3::extractPacket(buffer.data(), 25) == 25);
}