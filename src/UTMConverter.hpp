#ifndef _GPS_BASE_UTMCONVERTER_HPP_
#define _GPS_BASE_UTMCONVERTER_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <gps_base/BaseTypes.hpp>

class OGRCoordinateTransformation;

namespace gps_base
{
    class UTMConverter
    {
        private:
            int utm_zone;
            bool utm_north;
            base::Position origin;
            OGRCoordinateTransformation *utm2latlon, *latlon2utm;

            void createCoTransform();

        public:
            UTMConverter();
            UTMConverter(UTMConversionParameters const& parameters);
            UTMConverter(UTMConverter const& src);
            UTMConverter& operator=(UTMConverter const& src);
            ~UTMConverter();


            void setParameters(UTMConversionParameters const& parameters);

            UTMConversionParameters getParameters() const;

            /** Sets the UTM zone
             */
            void setUTMZone(int zone);

            /** Sets the UTM zone
             */
            void setUTMNorth(bool north);

            /** Get the UTM zone */
            int getUTMZone() const;

            /** Get whether we're north or south of the equator */
            bool getUTMNorth() const;

            /** Set a position that will be removed from the computed UTM
             * solution
             */
            void setNWUOrigin(base::Position origin);

            /** Returns the current origin position in UTM coordinates
             */
            base::Position getNWUOrigin() const;

            /** Convert a GPS solution into UTM coordinates
             *
             * The returned RBS will has all its fields invalidated (only the
             * timestamp updated) if there is no solution
             */
            base::samples::RigidBodyState convertToUTM(const gps_base::Solution &solution) const;

            /** Convert a UTM position into latitude/longitude with deviations
             * The Solutions' solution type field is not set by this function
             */
            gps_base::Solution convertUTMToGPS(const base::samples::RigidBodyState& position) const;

            /** Convert a GPS solution into NWU coordinates (Rock's convention)
             *
             * The returned RBS will has all its fields invalidated (only the
             * timestamp updated) if there is no solution
             */
            base::samples::RigidBodyState convertToNWU(const gps_base::Solution &solution) const;

            /** Convert NWU coordinates (Rock's convention) into GPS coordinates
             */
            gps_base::Solution convertNWUToGPS(const base::samples::RigidBodyState& nwu) const;

            /** Convert a UTM-converted GPS solution into NWU coordinates (Rock's convention)
             */
            base::samples::RigidBodyState convertToNWU(const base::samples::RigidBodyState &solution) const;

            /** Convert a NWU position into a UTM position
             */
            base::samples::RigidBodyState convertNWUToUTM(const base::samples::RigidBodyState &nwu) const;
    };

} // end namespace gps_base

#endif // _GPS_BASE_UTMCONVERTER_HPP_
