/*
 * Copyright 2019,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <iostream>
#include <sot/core/debug.hh>

#ifndef WIN32
#include <unistd.h>
#endif

using namespace std;

#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>

#include <sot/core/control-pd.hh>
#include <sstream>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

#define BOOST_TEST_MODULE debug - control - pd

#include <boost/test/output_test_stream.hpp>
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(control_pd) {
  sot::ControlPD *aControlPD = new ControlPD("acontrol_pd");
  aControlPD->init(0.001);
  std::istringstream Kpiss("[5](10.0,20.0,30.0,40.0,50.0)");
  std::istringstream Kdiss("[5](0.10,0.20,0.30,0.40,0.50)");
  aControlPD->KpSIN.set(Kpiss);
  aControlPD->KdSIN.set(Kdiss);
  std::istringstream posiss("[5](1.0,1.0,1.0,1.0,1.0)");
  aControlPD->positionSIN.set(posiss);
  std::istringstream dposiss("[5](3.0,3.1,3.2,3.3,3.4)");
  aControlPD->desiredpositionSIN.set(dposiss);
  std::istringstream veliss("[5](0.0,0.0,0.0,0.0,0.0)");
  aControlPD->velocitySIN.set(veliss);
  std::istringstream dveliss("[5](1.5,1.4,1.3,1.2,1.1)");
  aControlPD->desiredvelocitySIN.set(dveliss);

  aControlPD->controlSOUT.recompute(0);
  aControlPD->positionErrorSOUT.recompute(0);
  aControlPD->velocityErrorSOUT.recompute(0);
  {
    boost::test_tools::output_test_stream output;
    aControlPD->controlSOUT.get(output);
    BOOST_CHECK(output.is_equal("20.15 42.28 66.39 92.48 120.55"));
  }
  {
    boost::test_tools::output_test_stream output;
    aControlPD->positionErrorSOUT.get(output);
    BOOST_CHECK(output.is_equal("2 2.1 2.2 2.3 2.4"));
  }
  {
    boost::test_tools::output_test_stream output;
    aControlPD->velocityErrorSOUT.get(output);
    BOOST_CHECK(output.is_equal("1.5 1.4 1.3 1.2 1.1"));
  }
}
