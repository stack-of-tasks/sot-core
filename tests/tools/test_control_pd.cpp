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

BOOST_AUTO_TEST_CASE(control_pd) {
  sot::ControlPD *aControlPD = ControlPD("acontrol_pd");
  aControlPD->init(0.001);
  aControlPD->setsize(5);
  std::istringstream Kpiss("[5](10.0,20.0,30.0,40.0,50.0)");
  std::istringstream Kdiss("[5](0.10,0.20,0.30,0.40,0.50)");
  aControlPD->KpSIN.set(Kpiss);
  aControlPD->KdSIN.set(Kdiss);
  std::istringstream posiss("[5](1.0,1.0,1.0,1.0,1.0)");
  aControlPD->positionSIN.set(posiss);
  std::istringstream dposiss("[5](2.0,2.0,2.0,2.0,2.0)");
  aControlPD->desiredpositionSIN.set(dposiss);
  std::istringstream veliss("[5](0.0,0.0,0.0,0.0,0.0)");
  aControlPD->velocitySIN.set(veliss);
  std::istringstream dveliss("[5](1.0,1.0,1.0,1.0,1.0)");
  aControlPD->desiredvelocitySIN.set(dveliss);

  aControlPD->recompute(0);
  output_test_stream output;
  aControlPD->controlSOUT.get(output);
}
