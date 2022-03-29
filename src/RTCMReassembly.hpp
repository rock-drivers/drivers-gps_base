#ifndef GPS_BASE_RTCMREASSEMBLY_HPP
#define GPS_BASE_RTCMREASSEMBLY_HPP

#include <cstdint>
#include <memory>
#include <vector>

namespace gps_base {
    /** Reassembly of RTCM packets from a raw byte stream
     */
    class RTCMReassembly {
        static constexpr int BUFFER_SIZE = 4096;

        struct ReassemblyDriver;
        ReassemblyDriver* mDriver = nullptr;

    public:
        RTCMReassembly();
        RTCMReassembly(RTCMReassembly&) = delete;
        ~RTCMReassembly();

        /** Push data to be processed */
        void push(std::vector<uint8_t> const& in_data);

        /** Extract a single frame
         *
         * @return the frame's data, or an empty vector if there is no full
         *   frame available
         */
        std::vector<uint8_t> pull();
    };
}

#endif