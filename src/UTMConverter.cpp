#include "UTMConverter.hpp"
#include <iostream>
#include <ogr_spatialref.h>

using namespace std;
using namespace gps_base;

UTMConverter::UTMConverter()
    : utm_zone(32)
    , utm_north(true)
    , origin(base::Position::Zero())
    , coTransform(NULL)
{
    createCoTransform();
}

void UTMConverter::createCoTransform()
{
    OGRSpatialReference oSourceSRS;
    OGRSpatialReference oTargetSRS;

    oSourceSRS.SetWellKnownGeogCS("WGS84");
    oTargetSRS.SetWellKnownGeogCS("WGS84");
    oTargetSRS.SetUTM(this->utm_zone, this->utm_north);

    OGRCoordinateTransformation* newTransform =
        OGRCreateCoordinateTransformation(&oSourceSRS, &oTargetSRS);
    if (newTransform == NULL)
        throw runtime_error("Failed to initialize CoordinateTransform");

    delete coTransform;
    coTransform = newTransform;
}

void UTMConverter::setUTMZone(int zone)
{
    this->utm_zone = zone;
    createCoTransform();
}

void UTMConverter::setUTMNorth(bool north)
{
    this->utm_north = north;
    createCoTransform();
}

int UTMConverter::getUTMZone() const
{
    return this->utm_zone;
}

bool UTMConverter::getUTMNorth() const
{
    return this->utm_north;
}

base::Position UTMConverter::getNWUOrigin() const
{
    return this->origin;
}

void UTMConverter::setNWUOrigin(base::Position origin)
{
    this->origin = origin;
}

base::samples::RigidBodyState UTMConverter::convertToUTM(const gps_base::Solution &solution) const
{
    base::samples::RigidBodyState position;
    position.time = solution.time;

    if (solution.positionType == gps_base::NO_SOLUTION)
        return position;

    // if there is a valid reading, then write it to position readings port
    double northing = solution.latitude;
    double easting  = solution.longitude;
    double altitude = solution.altitude;

    coTransform->Transform(1, &easting, &northing, &altitude);

    position.time = solution.time;
    position.position.x() = easting;
    position.position.y() = northing;
    position.position.z() = altitude;
    position.cov_position = Eigen::Vector3d(
        solution.deviationLongitude * solution.deviationLongitude,
        solution.deviationLatitude * solution.deviationLatitude,
        solution.deviationAltitude * solution.deviationAltitude).asDiagonal();
    return position;
}

base::samples::RigidBodyState UTMConverter::convertToNWU(const gps_base::Solution &solution) const
{
    return convertToNWU(convertToUTM(solution));
}

base::samples::RigidBodyState UTMConverter::convertToNWU(const base::samples::RigidBodyState &utm) const
{
    base::samples::RigidBodyState position = utm;
    double easting  = position.position.x();
    double northing = position.position.y();
    position.position.x() = northing;
    position.position.y() = 1000000 - easting;
    position.position -= origin;
    std::swap(position.cov_position(0, 0), position.cov_position(1, 1));
    return position;
}
