#include <boost/test/unit_test.hpp>
#include <gps_base/RTCMReassembly.hpp>

#include <fstream>

using namespace gps_base;
using namespace std;

const std::vector<uint8_t> VALID_RTCM =
{
    0xd3, 0x00, 0x13, 0x3e, 0xd0, 0x00, 0x02, 0x36,
    0xfd, 0xb8, 0x0d, 0xde, 0x08, 0x00, 0x5b, 0x2b,
    0xc1, 0x08, 0xa7, 0xb9, 0x8d, 0x3d, 0xd8, 0xab, 0x37
};

BOOST_AUTO_TEST_CASE(pull_extracts_a_RTCM_packet_from_the_pushed_data) {
    RTCMReassembly reassembly;
    reassembly.push(VALID_RTCM);
    BOOST_TEST(reassembly.pull() == VALID_RTCM);
    BOOST_TEST(reassembly.pull().empty() == true);
}

BOOST_AUTO_TEST_CASE(it_handles_getting_more_data_than_the_internal_driver_buffer) {
    std::vector<uint8_t> in_data;
    in_data.resize(4096);
    in_data.insert(in_data.end(), VALID_RTCM.begin(), VALID_RTCM.end());

    RTCMReassembly reassembly;
    reassembly.push(in_data);
    BOOST_TEST(reassembly.pull() == VALID_RTCM);
    BOOST_TEST(reassembly.pull().empty() == true);
}