/*
 * Copyright 2019,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <pinocchio/multibody/liegroup/special-euclidean.hpp>

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
namespace dg = dynamicgraph;

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
    aLowerBound[i] = -3.14;
    // Specify state vector
    aStateVector[i] = 0.1;
    // Specify upper velocity bound
    anUpperVelBound[i] = 3.14;
    // Specify upper position bound
    anUpperBound[i] = 3.14;
    // Specify control vector
    aControlVector(i) = 0.1;
  }

  dg::Vector expected = aStateVector; // backup initial state vector

  /// Specify state size
  aDevice.setStateSize(38);
  /// Specify state bounds
  aDevice.setPositionBounds(aLowerBound, anUpperBound);
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

  const double dt = 0.001;
  const unsigned int N = 2000;
  for (unsigned int i = 0; i < N; i++) {
    aDevice.increment(dt);
    if (i == 0) {
      aDevice.stateSOUT.get(std::cout);
      std::ostringstream anoss;
      aDevice.stateSOUT.get(anoss);
    }
    if (i == 1) {
      aDevice.stateSOUT.get(std::cout);
      std::ostringstream anoss;
      aDevice.stateSOUT.get(anoss);
    }
  }

  aDevice.display(std::cout);
  aDevice.cmdDisplay();

  // verify correct integration
  typedef pinocchio::SpecialEuclideanOperationTpl<3, double> SE3;
  Eigen::Matrix<double, 7, 1> qin, qout;
  qin.head<3>() = expected.head<3>();

  Eigen::QuaternionMapd quat(qin.tail<4>().data());
  quat = Eigen::AngleAxisd(expected(5), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(expected(4), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(expected(3), Eigen::Vector3d::UnitX());

  const double T = dt * N;
  Eigen::Matrix<double, 6, 1> control = aControlVector.head<6>() * T;
  SE3().integrate(qin, control, qout);

  // Manual integration
  expected.head<3>() = qout.head<3>();
  expected.segment<3>(3) = Eigen::QuaternionMapd(qout.tail<4>().data())
                               .toRotationMatrix()
                               .eulerAngles(2, 1, 0)
                               .reverse();
  for (int i = 6; i < expected.size(); i++)
    expected[i] = 0.3;

  std::cout << expected.transpose() << std::endl;
  std::cout << aDevice.stateSOUT(N).transpose() << std::endl;

  BOOST_CHECK(aDevice.stateSOUT(N).isApprox(expected));
}
