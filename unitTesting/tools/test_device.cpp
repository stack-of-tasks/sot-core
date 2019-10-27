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
  dg::Vector aLowerBound(38), anUpperBound(38);
  dg::Vector anAccelerationVector(38);
  dg::Vector aControlVector(38);

  for (unsigned int i = 0; i < 38; i++) {
    // Specify lower velocity bound
    aLowerVelBound[i] = -3.14;
    // Specify lower position bound
    aLowerBound[i]=-3.14;
    // Specify state vector
    aStateVector[i] = 0.1;
    // Specify upper velocity bound
    anUpperVelBound[i] = 3.14;
    // Specify upper position bound
    anUpperBound[i]=3.14;
    // Specify control vector
    aControlVector(i)= 0.1;
  }
  /// Specify state size
  aDevice.setStateSize(38);
  /// Specify state bounds
  aDevice.setPositionBounds(aLowerBound,anUpperBound);
  /// Specify velocity size
  aDevice.setVelocitySize(38);
  /// Specify velocity
  aDevice.setVelocity(aStateVector);
  /// Specify velocity bounds
  aDevice.setVelocityBounds(aLowerVelBound, anUpperVelBound);
  /// Specify current state value
  aDevice.setState(aStateVector); // entry signal in position
  /// Specify constant control value
  aDevice.controlSIN.setConstant(aControlVector);

  for (unsigned int i = 0; i < 2000; i++) {
    double dt=0.001;
    aDevice.increment(dt);
    if (i == 0)
    {
      aDevice.stateSOUT.get(std::cout);
      std::ostringstream anoss;
      aDevice.stateSOUT.get(anoss);
      for (unsigned int i = 0; i < 38; i++)
        aControlVector[i]= 0.5;
    }
    if (i == 1)
    {
      aDevice.stateSOUT.get(std::cout);
      std::ostringstream anoss;
      aDevice.stateSOUT.get(anoss);
    }
  }

  aDevice.display(std::cout);
  aDevice.cmdDisplay();
}
