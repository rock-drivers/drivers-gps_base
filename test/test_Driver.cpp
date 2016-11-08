#include <boost/test/unit_test.hpp>
#include <gps_base/Driver.hpp>

using namespace gps_base;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_instantiated)
{
    gps_base::Driver driver;
}

BOOST_AUTO_TEST_CASE(it_should_output_utm_position)
{
    gps_base::Driver driver;
    gps_base::Solution solution;
    base::samples::RigidBodyState pos;

    driver.setUtmZone(24);
    driver.setUtmNorth(false);

    solution.latitude = -13.057361;
    solution.longitude = -38.649902;
    solution.altitude = 0.0;
    solution.positionType = gps_base::AUTONOMOUS;

    driver.convertSolutionToRBS(solution, pos);

    BOOST_REQUIRE_CLOSE(pos.position.x(), 537956.57943, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.y(), 8556494.7274, 0.0001);
    BOOST_REQUIRE_CLOSE(pos.position.z(), 0, 0.0001);

    return;
}
