#include <boost/test/unit_test.hpp>
#include <gps_base/UTMConverter.hpp>

using namespace gps_base;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_instantiated)
{
    gps_base::UTMConverter converter;
}

BOOST_AUTO_TEST_CASE(it_should_output_utm_position)
{
    gps_base::UTMConverter converter;
    gps_base::Solution solution;
    base::samples::RigidBodyState pos;

    solution.latitude = -13.057361;
    solution.longitude = -38.649902;
    solution.altitude = 0.0;
    converter.setUTMZone(24);
    converter.setUTMNorth(false);
    solution.positionType = gps_base::AUTONOMOUS;
    solution.deviationLatitude = 0.2;
    solution.deviationLongitude = 0.33;
    solution.deviationAltitude = 0.27;

    BOOST_REQUIRE_EQUAL(true, converter.convertSolutionToRBS(solution, pos));

    BOOST_REQUIRE_CLOSE(pos.position.x(), 537956.57943, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.y(), 8556494.7274, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.z(), 0, 0.0001);
    BOOST_REQUIRE_CLOSE(0.33*0.33, pos.cov_position(0, 0), 0.0001);
    BOOST_REQUIRE_CLOSE(0.2*0.2, pos.cov_position(1, 1), 0.0001);
    BOOST_REQUIRE_CLOSE(0.27*0.27, pos.cov_position(2, 2), 0.0001);

    return;
}

BOOST_AUTO_TEST_CASE(covariance_should_be_unset_if_deviation_is_unset)
{
    gps_base::UTMConverter converter;
    gps_base::Solution solution;
    base::samples::RigidBodyState pos;

    solution.positionType = gps_base::AUTONOMOUS;
    solution.deviationLatitude = base::unset<float>();
    solution.deviationLongitude = base::unset<float>();
    solution.deviationAltitude = base::unset<float>();

    BOOST_REQUIRE_EQUAL(true, converter.convertSolutionToRBS(solution, pos));
    BOOST_REQUIRE_EQUAL(true, base::isUnset<float>(pos.cov_position(0, 0)));
    BOOST_REQUIRE_EQUAL(true, base::isUnset<float>(pos.cov_position(1, 1)));
    BOOST_REQUIRE_EQUAL(true, base::isUnset<float>(pos.cov_position(2, 2)));
}

BOOST_AUTO_TEST_CASE(it_should_return_false_if_solution_is_unset)
{
    gps_base::UTMConverter converter;
    gps_base::Solution solution;
    base::samples::RigidBodyState pos;

    solution.positionType = gps_base::NO_SOLUTION;
    BOOST_REQUIRE_EQUAL(false, converter.convertSolutionToRBS(solution, pos));

    return;
}

