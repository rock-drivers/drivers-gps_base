#include "UTMConverter.hpp"
#include <iostream>
#include <ogr_spatialref.h>

using namespace std;
using namespace gps_base;

UTMConverter::UTMConverter()
    : utm_zone(32)
    , utm_north(true)
    , origin(base::Position::Zero())
    , utm2latlon(nullptr)
    , latlon2utm(nullptr)
{
    createCoTransform();
}

UTMConverter::UTMConverter(UTMConversionParameters const& parameters)
    : utm_zone(parameters.utm_zone)
    , utm_north(parameters.utm_north)
    , origin(parameters.nwu_origin)
    , utm2latlon(nullptr)
    , latlon2utm(nullptr)
{
    createCoTransform();
}

UTMConverter::~UTMConverter()
{
    delete latlon2utm;
    delete utm2latlon;
}

void UTMConverter::setParameters(UTMConversionParameters const& parameters)
{
    utm_zone = parameters.utm_zone;
    utm_north = parameters.utm_north;
    origin = parameters.nwu_origin;
    createCoTransform();
}

UTMConversionParameters UTMConverter::getParameters() const
{
    UTMConversionParameters parameters;
    parameters.utm_zone = utm_zone;
    parameters.utm_north = utm_north;
    parameters.nwu_origin = origin;
    return parameters;
}

void UTMConverter::createCoTransform()
{
    OGRSpatialReference latlonSRS;
    latlonSRS.SetWellKnownGeogCS("WGS84");
#if GDAL_VERSION_MAJOR >= 3
    latlonSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
#endif

    OGRSpatialReference utmSRS;
    utmSRS.SetWellKnownGeogCS("WGS84");
    utmSRS.SetUTM(this->utm_zone, this->utm_north);

    OGRCoordinateTransformation* newToUTM =
        OGRCreateCoordinateTransformation(&latlonSRS, &utmSRS);
    if (newToUTM == NULL)
        throw runtime_error("failed to compute coordinate transform from lat/lon to UTM");

    OGRCoordinateTransformation* newToLatLon =
        OGRCreateCoordinateTransformation(&utmSRS, &latlonSRS);
    if (newToLatLon == NULL)
        throw runtime_error("failed to compute coordinate transform from UTM to lat/lon");

    delete latlon2utm;
    latlon2utm = newToUTM;
    delete utm2latlon;
    utm2latlon = newToLatLon;
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

    latlon2utm->Transform(1, &easting, &northing, &altitude);

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

gps_base::Solution UTMConverter::convertUTMToGPS(const base::samples::RigidBodyState& position) const
{
    // if there is a valid reading, then write it to position readings port
    double easting  = position.position.x();
    double northing = position.position.y();
    double altitude = position.position.z();

    utm2latlon->Transform(1, &easting, &northing, &altitude);

    gps_base::Solution solution;
    solution.time = position.time;
    solution.latitude = northing;
    solution.longitude = easting;
    solution.altitude = altitude;
    solution.deviationLongitude = sqrt(position.cov_position(0, 0));
    solution.deviationLatitude = sqrt(position.cov_position(1, 1));
    solution.deviationAltitude = sqrt(position.cov_position(2, 2));
    return solution;
}

base::samples::RigidBodyState UTMConverter::convertToNWU(const gps_base::Solution &solution) const
{
    return convertToNWU(convertToUTM(solution));
}

gps_base::Solution UTMConverter::convertNWUToGPS(const base::samples::RigidBodyState& nwu) const
{
    return convertUTMToGPS(convertNWUToUTM(nwu));
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

base::samples::RigidBodyState UTMConverter::convertNWUToUTM(const base::samples::RigidBodyState &nwu) const
{
    base::samples::RigidBodyState utm = nwu;
    auto position = nwu.position + origin;
    double northing  = position.x();
    double easting = 1000000 - position.y();
    utm.position.x() = easting;
    utm.position.y() = northing;
    std::swap(utm.cov_position(0, 0), utm.cov_position(1, 1));
    return utm;
}
