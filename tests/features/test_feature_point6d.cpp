/*
 * Copyright 2019,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <iostream>

#include <dynamic-graph/linear-algebra.h>
#include <sot/core/debug.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/feature-point6d.hh>
#include <sot/core/gain-adaptive.hh>
#include <sot/core/sot.hh>
#include <sot/core/task.hh>
using namespace std;
using namespace dynamicgraph::sot;

class TestPoint6d {
public:
  FeaturePoint6d feature_, featureDes_;
  Task task_;
  int time_;
  int dim_, robotDim_, featureDim_;
  dynamicgraph::Vector manual_;

  TestPoint6d(unsigned dim, std::string &name)
      : feature_("feature" + name), featureDes_("featureDes" + name),
        task_("task" + name), time_(0)

  {
    feature_.computationFrame("desired");
    feature_.setReference(&featureDes_);
    feature_.selectionSIN = Flags(true);

    task_.addFeature(feature_);
    task_.setWithDerivative(true);
    dim_ = dim;
    robotDim_ = featureDim_ = dim;

    dynamicgraph::Matrix Jq(dim, dim);
    Jq.setIdentity();
    feature_.articularJacobianSIN.setReference(&Jq);

    manual_.resize(dim);
  }

  void setInputs(MatrixHomogeneous &s, MatrixHomogeneous &sd,
                 dynamicgraph::Vector &vd, double gain) {
    feature_.positionSIN = s;
    feature_.positionSIN.access(time_);
    feature_.positionSIN.setReady();

    featureDes_.positionSIN = sd;
    featureDes_.positionSIN.access(time_);
    featureDes_.positionSIN.setReady();

    featureDes_.velocitySIN = vd;
    featureDes_.velocitySIN.access(time_);
    featureDes_.velocitySIN.setReady();

    task_.controlGainSIN = gain;
    task_.controlGainSIN.access(time_);
    task_.controlGainSIN.setReady();
  }

  void printInputs() {
    std::cout << "----- inputs -----" << std::endl;
    std::cout << "feature_.position: " << feature_.positionSIN(time_)
              << std::endl;
    std::cout << "featureDes_.position: " << featureDes_.positionSIN(time_)
              << std::endl;
    std::cout << "featureDes_.velocity: "
              << featureDes_.velocitySIN(time_).transpose() << std::endl;
    std::cout << "task.controlGain: " << task_.controlGainSIN(time_)
              << std::endl;
  }

  int recompute() {

    feature_.errorSOUT.recompute(time_);
    feature_.errordotSOUT.recompute(time_);
    task_.taskSOUT.recompute(time_);
    task_.errorSOUT.recompute(time_);
    task_.errorTimeDerivativeSOUT.recompute(time_);
    time_++;
    MatrixHomogeneous s = feature_.positionSIN;
    MatrixHomogeneous sd = featureDes_.positionSIN;
    dynamicgraph::Vector vd = featureDes_.velocitySIN;
    double gain = task_.controlGainSIN;
    dynamicgraph::Vector manual;
    const std::vector<dynamicgraph::sot::MultiBound> &taskTaskSOUT =
        task_.taskSOUT(time_);

    /// Verify the computation of the desired frame.
    /// -gain *(s-sd) - ([w]x (sd -s)-vd)
    dynamicgraph::Matrix aM;
    dynamicgraph::Vector aV;
    aM.resize(3, 3);
    aV.resize(3);
    aM(0, 0) = 0.0;
    aM(0, 1) = -vd(5);
    aM(0, 2) = vd(4);
    aM(1, 0) = vd(5);
    aM(1, 1) = 0.0;
    aM(1, 2) = -vd(3);
    aM(2, 0) = -vd(4);
    aM(2, 1) = vd(3);
    aM(2, 2) = 0.0;
    for (unsigned int i = 0; i < 3; i++)
      aV(i) = sd(i, 3) - s(i, 3);

    aV = aM * aV;

    /// Recompute error_th.
    for (unsigned int i = 0; i < 3; i++) {
      manual_[i] = -gain * (s(i, 3) - sd(i, 3)) - (aV(i) - vd(i));
      if (manual_[i] != taskTaskSOUT[i].getSingleBound())
        return -1;
    }
    return 0;
  }

  void printOutput() {
    std::cout << "----- output -----" << std::endl;
    std::cout << "time: " << time_ << std::endl;
    std::cout << "feature.errorSOUT: " << feature_.errorSOUT(time_).transpose()
              << std::endl;
    std::cout << "feature.errordotSOUT: "
              << feature_.errordotSOUT(time_).transpose() << std::endl;
    std::cout << "task.taskSOUT: " << task_.taskSOUT(time_) << std::endl;
    //    std::cout << "task.errorSOUT: " << task_.errorSOUT(time_)
    //<< std::endl;
    // std::cout << "task.errordtSOUT: " << task_.errorTimeDerivativeSOUT(time_)
    //<< std::endl;
    std::cout << "manual: " << manual_.transpose() << std::endl;
  }

  int runTest(MatrixHomogeneous &s, MatrixHomogeneous &sd,
              dynamicgraph::Vector &vd, double gain) {
    setInputs(s, sd, vd, gain);
    printInputs();
    int r = recompute();
    printOutput();
    return r;
  }
};

int main(void) {
  // Name of the robot
  std::string srobot("Robot");
  // Dimension of the robot.
  unsigned int dim = 6;
  // Feature and Desired Feature
  MatrixHomogeneous s, sd;
  // Desired velocity
  dynamicgraph::Vector vd(6);
  // Task gain.
  double gain;
  // Result of test
  int r;

  TestPoint6d testFeaturePoint6d(dim, srobot);

  std::cout << " ----- Test Velocity -----" << std::endl;
  s.setIdentity();
  sd.setIdentity();
  vd.setConstant(1.);
  gain = 0.0;

  if ((r = testFeaturePoint6d.runTest(s, sd, vd, gain)) < 0) {
    std::cerr << "Failure on 1st test." << std::endl;
    return r;
  }

  std::cout << " ----- Test Position -----" << std::endl;
  s.setIdentity();
  sd.setIdentity();
  sd.translation()[2] = 2.0;
  vd.setZero();
  gain = 1.0;

  if ((r = testFeaturePoint6d.runTest(s, sd, vd, gain)) < 0) {
    std::cerr << "Failure on 2nd test." << std::endl;
    return r;
  }

  std::cout << " ----- Test both -----" << std::endl;
  s.setIdentity();
  sd.setIdentity();
  sd.translation()[2] = 2.0;
  vd.setConstant(1.);
  gain = 3.0;

  if ((r = testFeaturePoint6d.runTest(s, sd, vd, gain)) < 0) {
    std::cerr << "Failure on 3th test." << std::endl;
    return r;
  }

  std::cout << " ----- Test both again -----" << std::endl;
  s.setIdentity();
  sd.setIdentity();
  sd.translation()[2] = 2.0;
  vd.setConstant(1.);
  gain = 3.0;

  if ((r = testFeaturePoint6d.runTest(s, sd, vd, gain)) < 0) {
    std::cerr << "Failure on 4th test." << std::endl;
    return r;
  }

  std::cout << "Test successfull !" << std::endl;
  return 0;
}
