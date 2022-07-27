/*
 * Copyright 2019,
 * Noëlie Ramuzat,
 *
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

#include <sot/core/admittance-control-op-point.hh>
#include <sstream>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

#define BOOST_TEST_MODULE debug - control - admittance

#include <boost/test/output_test_stream.hpp>
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(control_admittance) {
  sot::core::AdmittanceControlOpPoint *aControlAdm =
      new sot::core::AdmittanceControlOpPoint("acontrol_admittance");

  std::istringstream Kp("[6](10.0,10.0,10.0,10.0,10.0,10.0)");
  std::istringstream Kd("[6](0.0,0.0,0.0,0.0,0.0,0.0)");
  aControlAdm->m_KpSIN.set(Kp);
  aControlAdm->m_KdSIN.set(Kd);
  std::istringstream dqSaturation("[6](10.0,10.0,10.0,10.0,10.0,10.0)");
  aControlAdm->m_dqSaturationSIN.set(dqSaturation);
  std::istringstream w_forceDes("[6](100.0,0.0,0.0,0.0,0.0,0.0)");
  aControlAdm->m_w_forceDesSIN.set(w_forceDes);
  std::istringstream force("[6](10.0,0.0,10.0,0.0,0.0,0.0)");
  aControlAdm->m_forceSIN.set(force);
  MatrixHomogeneous opPose;
  opPose.translation() << 0.3, 0.0, 0.0;
  opPose.linear() << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  aControlAdm->m_opPoseSIN = opPose;
  MatrixHomogeneous sensorPose;
  sensorPose.translation() << 0.3, 0.0, 0.0;
  sensorPose.linear() << 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0;
  aControlAdm->m_sensorPoseSIN = sensorPose;
  aControlAdm->init(0.001);

  aControlAdm->m_dqSOUT.recompute(0);
  {
    dynamicgraph::Vector expected(6);
    expected << 1.1, 0.0, -0.109, 0.0, 0.03, 0.0;
    BOOST_CHECK(aControlAdm->m_dqSOUT(0).isApprox(expected));
  }
}
