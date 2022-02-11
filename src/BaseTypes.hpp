#ifndef _GPS_BASE_BASETYPES_HPP_
#define _GPS_BASE_BASETYPES_HPP_

#include <vector>
#include <base/Time.hpp>
#include <base/Pose.hpp>

namespace gps_base
{
    enum GPS_SOLUTION_TYPES
    {
        NO_SOLUTION    = 0, //! No GPS solution found
        AUTONOMOUS_2D  = 6, //! Like AUTONOMOUS, but solution doesn't provide height information. Is 6 for historical reasons
        AUTONOMOUS     = 1, //! Solution not using differential corrections
        DIFFERENTIAL   = 2, //! Atmospheric correction is used e.g. as provided by a Satellite or Ground Based Augmentation System
        INVALID        = 3, //! Correction result is invalid
        RTK_FIXED      = 4, //! RTK solution
        RTK_FLOAT      = 5  //! RTK solution with undetermined phase shift
    };

    /** @deprecated do not use in new code */
    struct Time {
        base::Time cpu_time;
        base::Time gps_time;
        double processing_latency;
    };

    /** Representation of a position returned by a GNSS device */
    struct Solution {
        base::Time time;
        /** Latitude in degrees
         *
         * It is in degrees for historical reasons
         */
        double latitude;
        /** Longitude in degrees
         *
         * It is in degrees for historical reasons
         */
        double longitude;
        /** Type of solution */
        GPS_SOLUTION_TYPES positionType;
        /** How many satellites are used in this solution */
        int noOfSatellites;
        /** Altitude above mean sea level */
        double altitude;
        /** Geoidal separation at this location */
        double geoidalSeparation;
        /** Age of differential corrections used in this solution if any
         *
         * Set to base::unknown<double> if no differential corrections are available
         */
        double ageOfDifferentialCorrections;

        /** Error on the east-west axis, in meters
         */
        double deviationLatitude;
        /** Error on the north-south axis, in meters
         */
        double deviationLongitude;
        /** Error in altitude, in meters
         */
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
        CONSTELLATION_GLONASS,
        CONSTELLATION_GALILEO,
        CONSTELLATION_BEIDOU,
        CONSTELLATION_QZSS
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
            else if ((prn < 65) || ((prn >= 152) && (prn <= 158)))
                return CONSTELLATION_SBAS;
            else if ((prn >= 301) && (prn <= 336))
                return CONSTELLATION_GALILEO;
            else if ((prn >= 401) && (prn <= 437))
                return CONSTELLATION_BEIDOU;
            else if ((prn >= 193) && (prn <= 202))
                return CONSTELLATION_QZSS;
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

    /** @deprecated do not use in new code */
    struct UserDynamics {
        int hSpeed;
        int hAccel;
        int vSpeed;
        int vAccel;
        UserDynamics()
            : hSpeed(0), hAccel(0), vSpeed(0), vAccel(0) {}
    };

    /** Full quality information about the solution */
    struct ConstellationInfo {
        gps_base::SolutionQuality quality;
        gps_base::SatelliteInfo  satellites;
    };

    /**
     * Set of parameters used to setup GPS-to-local cartesian coordinates
     * conversion
     */
    struct UTMConversionParameters {
        /** The local origin for UTM-to-NWU conversion
         */
        base::Position nwu_origin = base::Position::Zero();
        /** UTM zone number
         */
        int utm_zone = 1;
        /** North or south of the equator
         */
        bool utm_north = true;
    };

}

#endif // _GPS_BASE_BASETYPES_HPP_
