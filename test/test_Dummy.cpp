#include <boost/test/unit_test.hpp>
#include <gps_base/Dummy.hpp>

using namespace gps_base;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    gps_base::DummyClass dummy;
    dummy.welcome();
}
