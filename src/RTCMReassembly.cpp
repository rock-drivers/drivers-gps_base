#include <gps_base/RTCMReassembly.hpp>

#include <gps_base/rtcm3.hpp>
#include <iodrivers_base/Driver.hpp>
#include <iodrivers_base/TestStream.hpp>

using namespace gps_base;
using namespace std;

struct RTCMReassembly::ReassemblyDriver : public iodrivers_base::Driver {
    int extractPacket(uint8_t const* bytes, size_t size) const {
        return rtcm3::extractPacket(bytes, size);
    }

    ReassemblyDriver()
        : iodrivers_base::Driver(BUFFER_SIZE) {}
};

RTCMReassembly::RTCMReassembly()
    : mDriver(new RTCMReassembly::ReassemblyDriver()) {
    mDriver->openURI("test://");
}

RTCMReassembly::~RTCMReassembly() {
    delete mDriver;
}

/** Push data to be processed */
void RTCMReassembly::push(std::vector<uint8_t> const& in_data) {
    auto& stream = dynamic_cast<iodrivers_base::TestStream&>(*mDriver->getMainStream());
    stream.pushDataToDriver(in_data);
}

std::vector<uint8_t> RTCMReassembly::pull() {
    uint8_t buffer[BUFFER_SIZE];
    try {
        size_t size = mDriver->readPacket(buffer, BUFFER_SIZE, base::Time());
        return vector<uint8_t>(buffer, buffer + size);
    }
    catch(iodrivers_base::TimeoutError&) {
        return vector<uint8_t>();
    }
}
