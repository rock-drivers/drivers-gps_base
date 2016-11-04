#ifndef _GPS_BASE_DRIVER_HPP_
#define _GPS_BASE_DRIVER_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <gps_base/BaseTypes.hpp>

class OGRCoordinateTransformation;

namespace gps_base
{
    class Driver
    {
        private:
            int utm_zone;
            bool utm_north;
            base::Position origin;
            OGRCoordinateTransformation *coTransform;

            void createCoTransform();

        public:
            Driver();

            void setUtmZone(int zone);
            void setUtmNorth(bool north);
            int getUtmZone();
            bool getUtmNorth();
            void setOrigin(base::Position origin);
            base::Position getOrigin();

            bool convertSolutionToRBS(const gps_base::Solution &solution, base::samples::RigidBodyState &position);
    };

} // end namespace gps_base

#endif // _GPS_BASE_DRIVER_HPP_
