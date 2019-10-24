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

#include <boost/test/unit_test.hpp>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <sot/core/device.hh>
#include <sstream>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

class TestDevice : public dg::sot::Device {
public:
  TestDevice(const std::string &RobotName) : Device(RobotName) {
    timestep_ = 0.001;
  }
  ~TestDevice() {}
};

BOOST_AUTO_TEST_CASE(test_device) {
  TestDevice aDevice(std::string("simple_humanoid"));

  /// Fix constant vector for the control entry in position
  dg::Vector aStateVector(38);
  dg::Vector aVelocityVector(38);
  dg::Vector aLowerVelBound(38), anUpperVelBound(38);
  dg::Vector anAccelerationVector(38);
  dg::Vector aControlVector(38);

  for (unsigned int i = 0; i < 38; i++) {
    aLowerVelBound[i] = -3.14;
    aStateVector[i] = -0.5;
    anUpperVelBound[i] = 3.14;
    aControlVector(i)=-0.1;
  }
  
  aDevice.setVelocitySize(38);
  aDevice.setVelocityBounds(aLowerVelBound, anUpperVelBound);
  aDevice.setVelocity(aStateVector);
  aDevice.setState(aStateVector); // entry signal in position
  aDevice.controlSIN.setConstant(aControlVector);

  for (unsigned int i = 0; i < 2000; i++) {
    aDevice.stateSOUT.recompute(i);
    if (i == 1)
      std::cout << " First integration " << aDevice.stateSOUT << std::endl;
  }

  aDevice.display(std::cout);
  aDevice.cmdDisplay();
  // const dg::Vector &aControl = aDevice.motorcontrolSOUT(2001);
  //  double diff = 0, ldiff;
}
