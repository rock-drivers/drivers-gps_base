#include <boost/test/unit_test.hpp>
#include <gps_base/UTMConverter.hpp>

using namespace gps_base;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_instantiated)
{
    gps_base::UTMConverter converter;
}

BOOST_AUTO_TEST_CASE(it_converts_the_position)
{
    gps_base::Solution solution;
    solution.positionType = gps_base::AUTONOMOUS;
    solution.latitude = -13.057361;
    solution.longitude = -38.649902;
    solution.altitude = 2.0;

    gps_base::UTMConverter converter;
    converter.setUTMZone(24);
    converter.setUTMNorth(false);

    base::samples::RigidBodyState pos;
    pos = converter.convertToUTM(solution);
    BOOST_REQUIRE_CLOSE(pos.position.x(), 537956.57943, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.y(), 8556494.7274, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.z(), 2, 0.0001);
    pos = converter.convertToNWU(solution);
    BOOST_REQUIRE_CLOSE(pos.position.x(), 8556494.7274, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.y(), 1000000 - 537956.57943, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.z(), 2, 0.0001);
}

BOOST_AUTO_TEST_CASE(it_applies_the_origin_only_in_NWU_coordinates)
{
    gps_base::Solution solution;
    solution.positionType = gps_base::AUTONOMOUS;
    solution.latitude = -13.057361;
    solution.longitude = -38.649902;
    solution.altitude = 2.0;

    gps_base::UTMConverter converter;
    converter.setUTMZone(24);
    converter.setUTMNorth(false);
    converter.setNWUOrigin(base::Position(8550000, 400000, 0));

    base::samples::RigidBodyState pos;
    pos = converter.convertToUTM(solution);
    BOOST_REQUIRE_CLOSE(pos.position.x(), 537956.57943, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.y(), 8556494.7274, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.z(), 2, 0.0001);
    pos = converter.convertToNWU(solution);
    BOOST_REQUIRE_CLOSE(pos.position.x(), 6494.7274, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.y(), 62043.420570012648, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.z(), 2, 0.0001);
}

BOOST_AUTO_TEST_CASE(it_propagates_the_deviations)
{
    gps_base::Solution solution;
    solution.positionType = gps_base::AUTONOMOUS;
    solution.deviationLatitude = 0.2;
    solution.deviationLongitude = 0.33;
    solution.deviationAltitude = 0.27;

    gps_base::UTMConverter converter;

    base::samples::RigidBodyState pos = converter.convertToUTM(solution);
    BOOST_REQUIRE_CLOSE(0.33*0.33, pos.cov_position(0, 0), 0.0001);
    BOOST_REQUIRE_CLOSE(0.2*0.2,   pos.cov_position(1, 1), 0.0001);
    BOOST_REQUIRE_CLOSE(0.27*0.27, pos.cov_position(2, 2), 0.0001);
    pos = converter.convertToNWU(solution);
    BOOST_REQUIRE_CLOSE(0.2*0.2,   pos.cov_position(0, 0), 0.0001);
    BOOST_REQUIRE_CLOSE(0.33*0.33, pos.cov_position(1, 1), 0.0001);
    BOOST_REQUIRE_CLOSE(0.27*0.27, pos.cov_position(2, 2), 0.0001);
}

BOOST_AUTO_TEST_CASE(it_initializes_non_diagonal_covariance_elements_to_zero)
{
    gps_base::Solution solution;
    solution.positionType = gps_base::AUTONOMOUS;

    gps_base::UTMConverter converter;

    base::samples::RigidBodyState pos = converter.convertToUTM(solution);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (i != j)
                BOOST_REQUIRE_EQUAL(0, pos.cov_position(i, j));
}

BOOST_AUTO_TEST_CASE(it_propagates_the_timestamp)
{
    gps_base::Solution solution;
    solution.time = base::Time::fromSeconds(100);
    solution.positionType = gps_base::AUTONOMOUS;

    gps_base::UTMConverter converter;

    base::samples::RigidBodyState pos;
    pos = converter.convertToUTM(solution);
    BOOST_REQUIRE_EQUAL(base::Time::fromSeconds(100), pos.time);
    pos = converter.convertToNWU(solution);
    BOOST_REQUIRE_EQUAL(base::Time::fromSeconds(100), pos.time);
}

BOOST_AUTO_TEST_CASE(covariance_should_be_unset_if_deviation_is_unset)
{
    gps_base::Solution solution;
    solution.positionType = gps_base::AUTONOMOUS;
    solution.deviationLatitude = base::unset<float>();
    solution.deviationLongitude = base::unset<float>();
    solution.deviationAltitude = base::unset<float>();

    gps_base::UTMConverter converter;

    base::samples::RigidBodyState pos;
    pos = converter.convertToUTM(solution);
    BOOST_REQUIRE(base::isUnset<float>(pos.cov_position(0, 0)));
    BOOST_REQUIRE(base::isUnset<float>(pos.cov_position(1, 1)));
    BOOST_REQUIRE(base::isUnset<float>(pos.cov_position(2, 2)));

    pos = converter.convertToNWU(solution);
    BOOST_REQUIRE(base::isUnset<float>(pos.cov_position(0, 0)));
    BOOST_REQUIRE(base::isUnset<float>(pos.cov_position(1, 1)));
    BOOST_REQUIRE(base::isUnset<float>(pos.cov_position(2, 2)));
}

BOOST_AUTO_TEST_CASE(the_convertion_methods_return_an_invalid_RBS_with_timestamp_set_if_there_is_no_solution)
{
    gps_base::Solution solution;
    solution.time = base::Time::now();
    solution.positionType = gps_base::NO_SOLUTION;

    gps_base::UTMConverter converter;

    base::samples::RigidBodyState pos;
    pos = converter.convertToUTM(solution);
    BOOST_REQUIRE_EQUAL(solution.time, pos.time);
    BOOST_REQUIRE(!pos.hasValidPosition());

    pos = converter.convertToNWU(solution);
    BOOST_REQUIRE_EQUAL(solution.time, pos.time);
    BOOST_REQUIRE(!pos.hasValidPosition());
}

