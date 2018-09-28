#ifndef _GPS_BASE_BASETYPES_HPP_
#define _GPS_BASE_BASETYPES_HPP_

#include <vector>
#include <base/Time.hpp>

namespace gps_base
{
    enum GPS_SOLUTION_TYPES
    {
        NO_SOLUTION  = 0,
        AUTONOMOUS_2D  = 6, // is 6 for historical reasons
        AUTONOMOUS   = 1,
        DIFFERENTIAL = 2,
        INVALID      = 3,
        RTK_FIXED    = 4,
        RTK_FLOAT    = 5,
        OMNISTAR     = 7
    };

    struct Time {
        base::Time cpu_time;
        base::Time gps_time;
        double processing_latency;
    };

    struct Solution {
        base::Time time;
        double latitude;
        double longitude;
        GPS_SOLUTION_TYPES positionType;
        int noOfSatellites;
        double altitude;
        double geoidalSeparation;
        double ageOfDifferentialCorrections;

        double deviationLatitude;
        double deviationLongitude;
        double deviationAltitude;
        Solution()
	        : positionType(INVALID) {}
    };

    struct Position {
        base::Time time;
        double latitude;
        double longitude;
        GPS_SOLUTION_TYPES positionType;
        int noOfSatellites;
        double altitude;
        double geoidalSeparation;
        double ageOfDifferentialCorrections;
	    Position()
	        : positionType(INVALID) {}
    };

    struct Errors {
        base::Time time;
        double deviationLatitude;
        double deviationLongitude;
        double deviationAltitude;
    };

    struct SolutionQuality {
        base::Time time;
        std::vector<int> usedSatellites;
        double pdop;
        double hdop;
        double vdop;
    };

    enum CONSTELLATIONS {
        CONSTELLATION_GPS,
        CONSTELLATION_SBAS,
        CONSTELLATION_GLONASS
    };
    struct Satellite {
        int PRN;
        int elevation;
        int azimuth;
        double SNR;

        static CONSTELLATIONS getConstellationFromPRN(int prn)
        {
            if (prn < 33)
                return CONSTELLATION_GPS;
            else if (prn < 65)
                return CONSTELLATION_SBAS;
            else
                return CONSTELLATION_GLONASS;
        }

        CONSTELLATIONS getConstellation() const
        {
	        return getConstellationFromPRN(PRN);
        }
    };

    struct SatelliteInfo {
        base::Time time;
        std::vector < gps_base::Satellite> knownSatellites;
    };

    struct UserDynamics {
        int hSpeed;
        int hAccel;
        int vSpeed;
        int vAccel;
        UserDynamics()
            : hSpeed(0), hAccel(0), vSpeed(0), vAccel(0) {}
    };

    struct ConstellationInfo {
        gps_base::SolutionQuality quality;
        gps_base::SatelliteInfo  satellites;
    };
}

#endif // _GPS_BASE_BASETYPES_HPP_
