#include "UTMConverter.hpp"
#include <iostream>
#include <ogr_spatialref.h>

using namespace std;
using namespace gps_base;

UTMConverter::UTMConverter()
{
    this->utm_zone = 32;
    this->utm_north = true;
    this->origin = base::Position::Zero();
    createCoTransform();
}

void UTMConverter::createCoTransform()
{
    OGRSpatialReference oSourceSRS;
    OGRSpatialReference oTargetSRS;

    oSourceSRS.SetWellKnownGeogCS("WGS84");
    oTargetSRS.SetWellKnownGeogCS("WGS84");
    oTargetSRS.SetUTM(this->utm_zone, this->utm_north);

    coTransform = OGRCreateCoordinateTransformation(&oSourceSRS, &oTargetSRS);

    if (coTransform == NULL)
    {
        throw runtime_error("Failed to initialize CoordinateTransform");
    }
}

void UTMConverter::setUtmZone(int zone)
{
    this->utm_zone = zone;
    createCoTransform();
}

void UTMConverter::setUtmNorth(bool north)
{
    this->utm_north = north;
    createCoTransform();
}

int UTMConverter::getUtmZone()
{
    return this->utm_zone;
}

bool UTMConverter::getUtmNorth()
{
    return this->utm_north;
}

base::Position UTMConverter::getOrigin()
{
    return this->origin;
}

void UTMConverter::setOrigin(base::Position origin)
{
    this->origin = origin;
}

bool UTMConverter::convertSolutionToRBS(const gps_base::Solution &solution, base::samples::RigidBodyState &position)
{
    // if there is a valid reading, then write it to position readings port
    if (solution.positionType != gps_base::NO_SOLUTION)
    {
        double la = solution.latitude;
        double lo = solution.longitude;
        double alt = solution.altitude;

        coTransform->Transform(1, &lo, &la, &alt);

        position.time = solution.time;
        position.position.x() = lo - origin.x();
        position.position.y() = la - origin.y();
        position.position.z() = alt - origin.z();
        position.cov_position(0, 0) = solution.deviationLongitude * solution.deviationLongitude;
        position.cov_position(1, 1) = solution.deviationLatitude * solution.deviationLatitude;
        position.cov_position(2, 2) = solution.deviationAltitude * solution.deviationAltitude;

        return true;
    }

    return false;
}
