
#define BOOST_TEST_DYN_LINK

#include <iostream>
#define BOOST_TEST_MODULE PlannerTests
#include <boost/test/unit_test.hpp>

#include "StateInformation.h"
#include "kinect_interface.h"
#include "planning.h"



BOOST_AUTO_TEST_SUITE( Planner )
BOOST_AUTO_TEST_CASE( test_can_move )
{
FrameDataPtr clear_frame=boost::shared_ptr<FrameData>();
FrameDataPtr bad_frame=boost::shared_ptr<FrameData>();
//TODO load frame data
MapperPathPlanner my_planner= MapperPathPlanner();
BOOST_CHECK (my_planner.canMove(clear_frame));
BOOST_CHECK (my_planner.canMove(bad_frame));
}

BOOST_AUTO_TEST_SUITE_END()
