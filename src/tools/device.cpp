/*
 * Copyright 2010,
 * Nicolas Mansard, Olivier Stasse, François Bleibel, Florent Lamiraux
 *
 * CNRS
 *
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#define ENABLE_RT_LOG

#include "sot/core/device.hh"
#include <iostream>
#include <sot/core/debug.hh>
using namespace std;

#include <Eigen/Geometry>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/real-time-logger.h>
#include <sot/core/matrix-geometry.hh>

#include <pinocchio/multibody/liegroup/special-euclidean.hpp>
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string Device::CLASS_NAME = "Device";

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void Device::integrateRollPitchYaw(Vector &state, const Vector &control,
                                   double dt) {
  using Eigen::AngleAxisd;
  using Eigen::QuaternionMapd;
  using Eigen::Vector3d;

  typedef pinocchio::SpecialEuclideanOperationTpl<3, double> SE3;
  Eigen::Matrix<double, 7, 1> qin, qout;
  qin.head<3>() = state.head<3>();

  QuaternionMapd quat(qin.tail<4>().data());
  quat = AngleAxisd(state(5), Vector3d::UnitZ()) *
         AngleAxisd(state(4), Vector3d::UnitY()) *
         AngleAxisd(state(3), Vector3d::UnitX());

  SE3().integrate(qin, control.head<6>() * dt, qout);

  // Update freeflyer pose
  ffPose_.translation() = qout.head<3>();
  state.head<3>() = qout.head<3>();

  ffPose_.linear() = QuaternionMapd(qout.tail<4>().data()).toRotationMatrix();
  state.segment<3>(3) = ffPose_.linear().eulerAngles(2, 1, 0).reverse();
}

const MatrixHomogeneous &Device::freeFlyerPose() const { return ffPose_; }

Device::~Device() {
  for (unsigned int i = 0; i < 4; ++i) {
    delete forcesSOUT[i];
  }
}

Device::Device(const std::string &n)
    : Entity(n), state_(6), sanityCheck_(true),
      controlInputType_(CONTROL_INPUT_ONE_INTEGRATION),
      controlSIN(NULL, "Device(" + n + ")::input(double)::control"),
      attitudeSIN(NULL, "Device(" + n + ")::input(vector3)::attitudeIN"),
      zmpSIN(NULL, "Device(" + n + ")::input(vector3)::zmp"),
      stateSOUT("Device(" + n + ")::output(vector)::state")
      //,attitudeSIN(NULL,"Device::input(matrixRot)::attitudeIN")
      ,
      velocitySOUT("Device(" + n + ")::output(vector)::velocity"),
      attitudeSOUT("Device(" + n + ")::output(matrixRot)::attitude"),
      motorcontrolSOUT("Device(" + n + ")::output(vector)::motorcontrol"),
      previousControlSOUT("Device(" + n + ")::output(vector)::previousControl"),
      ZMPPreviousControllerSOUT("Device(" + n +
                                ")::output(vector)::zmppreviouscontroller"),
      robotState_("Device(" + n + ")::output(vector)::robotState"),
      robotVelocity_("Device(" + n + ")::output(vector)::robotVelocity"),
      pseudoTorqueSOUT("Device(" + n + ")::output(vector)::ptorque")

      ,
      ffPose_(), forceZero6(6) {
  forceZero6.fill(0);
  /* --- SIGNALS --- */
  for (int i = 0; i < 4; ++i) {
    withForceSignals[i] = false;
  }
  forcesSOUT[0] =
      new Signal<Vector, int>("Device(" + n + ")::output(vector6)::forceRLEG");
  forcesSOUT[1] =
      new Signal<Vector, int>("Device(" + n + ")::output(vector6)::forceLLEG");
  forcesSOUT[2] =
      new Signal<Vector, int>("Device(" + n + ")::output(vector6)::forceRARM");
  forcesSOUT[3] =
      new Signal<Vector, int>("Device(" + n + ")::output(vector6)::forceLARM");

  signalRegistration(
      controlSIN << stateSOUT << robotState_ << robotVelocity_ << velocitySOUT
                 << attitudeSOUT << attitudeSIN << zmpSIN << *forcesSOUT[0]
                 << *forcesSOUT[1] << *forcesSOUT[2] << *forcesSOUT[3]
                 << previousControlSOUT << pseudoTorqueSOUT << motorcontrolSOUT
                 << ZMPPreviousControllerSOUT);
  state_.fill(.0);
  stateSOUT.setConstant(state_);

  velocity_.resize(state_.size());
  velocity_.setZero();
  velocitySOUT.setConstant(velocity_);

  /* --- Commands --- */
  {
    std::string docstring;
    /* Command setStateSize. */
    docstring = "\n"
                "    Set size of state vector\n"
                "\n";
    addCommand("resize", new command::Setter<Device, unsigned int>(
                             *this, &Device::setStateSize, docstring));
    docstring = "\n"
                "    Set state vector value\n"
                "\n";
    addCommand("set", new command::Setter<Device, Vector>(
                          *this, &Device::setState, docstring));

    docstring = "\n"
                "    Set velocity vector value\n"
                "\n";
    addCommand("setVelocity", new command::Setter<Device, Vector>(
                                  *this, &Device::setVelocity, docstring));

    void (Device::*setRootPtr)(const Matrix &) = &Device::setRoot;
    docstring = command::docCommandVoid1("Set the root position.",
                                         "matrix homogeneous");
    addCommand("setRoot",
               command::makeCommandVoid1(*this, setRootPtr, docstring));

    /* Second Order Integration set. */
    docstring = "\n"
                "    Set the position calculous starting from  \n"
                "    acceleration measure instead of velocity \n"
                "\n";

    addCommand("setSecondOrderIntegration",
               command::makeCommandVoid0(
                   *this, &Device::setSecondOrderIntegration, docstring));

    /* Display information */
    docstring = "\n"
                "    Display information on device  \n"
                "\n";
    addCommand("display", command::makeCommandVoid0(*this, &Device::cmdDisplay,
                                                    docstring));

    /* SET of control input type. */
    docstring = "\n"
                "    Set the type of control input which can be  \n"
                "    acceleration, velocity, or position\n"
                "\n";

    addCommand("setControlInputType",
               new command::Setter<Device, string>(
                   *this, &Device::setControlInputType, docstring));

    docstring = "\n"
                "    Enable/Disable sanity checks\n"
                "\n";
    addCommand("setSanityCheck",
               new command::Setter<Device, bool>(*this, &Device::setSanityCheck,
                                                 docstring));

    addCommand("setPositionBounds",
               command::makeCommandVoid2(
                   *this, &Device::setPositionBounds,
                   command::docCommandVoid2("Set robot position bounds",
                                            "vector: lower bounds",
                                            "vector: upper bounds")));

    addCommand("setVelocityBounds",
               command::makeCommandVoid2(
                   *this, &Device::setVelocityBounds,
                   command::docCommandVoid2("Set robot velocity bounds",
                                            "vector: lower bounds",
                                            "vector: upper bounds")));

    addCommand("setTorqueBounds",
               command::makeCommandVoid2(
                   *this, &Device::setTorqueBounds,
                   command::docCommandVoid2("Set robot torque bounds",
                                            "vector: lower bounds",
                                            "vector: upper bounds")));

    addCommand("getTimeStep",
               command::makeDirectGetter(
                   *this, &this->timestep_,
                   command::docDirectGetter("Time step", "double")));

    // Handle commands and signals called in a synchronous way.
    periodicCallBefore_.addSpecificCommands(*this, commandMap, "before.");
    periodicCallAfter_.addSpecificCommands(*this, commandMap, "after.");
  }
}

void Device::setStateSize(const unsigned int &size) {
  state_.resize(size);
  state_.fill(.0);
  stateSOUT.setConstant(state_);
  previousControlSOUT.setConstant(state_);
  pseudoTorqueSOUT.setConstant(state_);
  motorcontrolSOUT.setConstant(state_);

  Device::setVelocitySize(size);

  Vector zmp(3);
  zmp.fill(.0);
  ZMPPreviousControllerSOUT.setConstant(zmp);
}

void Device::setVelocitySize(const unsigned int &size) {
  velocity_.resize(size);
  velocity_.fill(.0);
  velocitySOUT.setConstant(velocity_);
}

void Device::setState(const Vector &st) {
  if (sanityCheck_) {
    const Vector::Index &s = st.size();
    switch (controlInputType_) {
    case CONTROL_INPUT_TWO_INTEGRATION:
      dgRTLOG()
          << "Sanity check for this control is not well supported. "
             "In order to make it work, use pinocchio and the contact forces "
             "to estimate the joint torques for the given acceleration.\n";
      if (s != lowerTorque_.size() || s != upperTorque_.size())
        throw std::invalid_argument(
            "Upper and/or lower torque bounds "
            "do not match state size. Set them first with setTorqueBounds");
      // fall through
    case CONTROL_INPUT_ONE_INTEGRATION:
      if (s != lowerVelocity_.size() || s != upperVelocity_.size())
        throw std::invalid_argument("Upper and/or lower velocity bounds "
                                    "do not match state size."
                                    " Set them first with setVelocityBounds");
      // fall through
    case CONTROL_INPUT_NO_INTEGRATION:
      break;
    default:
      throw std::invalid_argument("Invalid control mode type.");
    }
  }
  state_ = st;
  stateSOUT.setConstant(state_);
  motorcontrolSOUT.setConstant(state_);
}

void Device::setVelocity(const Vector &vel) {
  velocity_ = vel;
  velocitySOUT.setConstant(velocity_);
}

void Device::setRoot(const Matrix &root) {
  Eigen::Matrix4d _matrix4d(root);
  MatrixHomogeneous _root(_matrix4d);
  setRoot(_root);
}

void Device::setRoot(const MatrixHomogeneous &worldMwaist) {
  VectorRollPitchYaw r = (worldMwaist.linear().eulerAngles(2, 1, 0)).reverse();
  Vector q = state_;
  q = worldMwaist.translation(); // abusive ... but working.
  for (unsigned int i = 0; i < 3; ++i)
    q(i + 3) = r(i);
}

void Device::setSecondOrderIntegration() {
  controlInputType_ = CONTROL_INPUT_TWO_INTEGRATION;
  velocity_.resize(state_.size());
  velocity_.setZero();
  velocitySOUT.setConstant(velocity_);
}

void Device::setNoIntegration() {
  controlInputType_ = CONTROL_INPUT_NO_INTEGRATION;
  velocity_.resize(state_.size());
  velocity_.setZero();
  velocitySOUT.setConstant(velocity_);
}

void Device::setControlInputType(const std::string &cit) {
  for (int i = 0; i < CONTROL_INPUT_SIZE; i++)
    if (cit == ControlInput_s[i]) {
      controlInputType_ = (ControlInput)i;
      sotDEBUG(25) << "Control input type: " << ControlInput_s[i] << endl;
      return;
    }
  sotDEBUG(25) << "Unrecognized control input type: " << cit << endl;
}

void Device::setSanityCheck(const bool &enableCheck) {
  if (enableCheck) {
    const Vector::Index &s = state_.size();
    switch (controlInputType_) {
    case CONTROL_INPUT_TWO_INTEGRATION:
      dgRTLOG()
          << "Sanity check for this control is not well supported. "
             "In order to make it work, use pinocchio and the contact forces "
             "to estimate the joint torques for the given acceleration.\n";
      if (s != lowerTorque_.size() || s != upperTorque_.size())
        throw std::invalid_argument(
            "Upper and/or lower torque bounds "
            "do not match state size. Set them first with setTorqueBounds");
      // fall through
    case CONTROL_INPUT_ONE_INTEGRATION:
      if (s != lowerVelocity_.size() || s != upperVelocity_.size())
        throw std::invalid_argument(
            "Upper and/or lower velocity bounds "
            "do not match state size. Set them first with setVelocityBounds");
      // fall through
    case CONTROL_INPUT_NO_INTEGRATION:
      if (s != lowerPosition_.size() || s != upperPosition_.size())
        throw std::invalid_argument(
            "Upper and/or lower position bounds "
            "do not match state size. Set them first with setPositionBounds");
      break;
    default:
      throw std::invalid_argument("Invalid control mode type.");
    }
  }
  sanityCheck_ = enableCheck;
}

void Device::setPositionBounds(const Vector &lower, const Vector &upper) {
  std::ostringstream oss;
  if (lower.size() != state_.size()) {
    oss << "Lower bound size should be " << state_.size();
    throw std::invalid_argument(oss.str());
  }
  if (upper.size() != state_.size()) {
    oss << "Upper bound size should be " << state_.size();
    throw std::invalid_argument(oss.str());
  }
  lowerPosition_ = lower;
  upperPosition_ = upper;
}

void Device::setVelocityBounds(const Vector &lower, const Vector &upper) {
  std::ostringstream oss;
  if (lower.size() != velocity_.size()) {
    oss << "Lower bound size should be " << velocity_.size();
    throw std::invalid_argument(oss.str());
  }
  if (upper.size() != velocity_.size()) {
    oss << "Upper bound size should be " << velocity_.size();
    throw std::invalid_argument(oss.str());
  }
  lowerVelocity_ = lower;
  upperVelocity_ = upper;
}

void Device::setTorqueBounds(const Vector &lower, const Vector &upper) {
  // TODO I think the torque bounds size are state_.size()-6...
  std::ostringstream oss;
  if (lower.size() != state_.size()) {
    oss << "Lower bound size should be " << state_.size();
    throw std::invalid_argument(oss.str());
  }
  if (upper.size() != state_.size()) {
    oss << "Lower bound size should be " << state_.size();
    throw std::invalid_argument(oss.str());
  }
  lowerTorque_ = lower;
  upperTorque_ = upper;
}

void Device::increment(const double &dt) {
  int time = stateSOUT.getTime();
  sotDEBUG(25) << "Time : " << time << std::endl;

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try {
    periodicCallBefore_.run(time + 1);
  } catch (std::exception &e) {
    dgRTLOG() << "exception caught while running periodical commands (before): "
              << e.what() << std::endl;
  } catch (const char *str) {
    dgRTLOG() << "exception caught while running periodical commands (before): "
              << str << std::endl;
  } catch (...) {
    dgRTLOG() << "unknown exception caught while"
              << " running periodical commands (before)" << std::endl;
  }

  /* Force the recomputation of the control. */
  controlSIN(time);
  sotDEBUG(25) << "u" << time << " = " << controlSIN.accessCopy() << endl;

  /* Integration of numerical values. This function is virtual. */
  integrate(dt);
  sotDEBUG(25) << "q" << time << " = " << state_ << endl;

  /* Position the signals corresponding to sensors. */
  stateSOUT.setConstant(state_);
  stateSOUT.setTime(time + 1);
  // computation of the velocity signal
  if (controlInputType_ == CONTROL_INPUT_TWO_INTEGRATION) {
    velocitySOUT.setConstant(velocity_);
    velocitySOUT.setTime(time + 1);
  } else if (controlInputType_ == CONTROL_INPUT_ONE_INTEGRATION) {
    velocitySOUT.setConstant(controlSIN.accessCopy());
    velocitySOUT.setTime(time + 1);
  }
  for (int i = 0; i < 4; ++i) {
    if (!withForceSignals[i])
      forcesSOUT[i]->setConstant(forceZero6);
  }
  Vector zmp(3);
  zmp.fill(.0);
  ZMPPreviousControllerSOUT.setConstant(zmp);

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try {
    periodicCallAfter_.run(time + 1);
  } catch (std::exception &e) {
    dgRTLOG() << "exception caught while running periodical commands (after): "
              << e.what() << std::endl;
  } catch (const char *str) {
    dgRTLOG() << "exception caught while running periodical commands (after): "
              << str << std::endl;
  } catch (...) {
    dgRTLOG() << "unknown exception caught while"
              << " running periodical commands (after)" << std::endl;
  }

  // Others signals.
  motorcontrolSOUT.setConstant(state_);
}

// Return true if it saturates.
inline bool saturateBounds(double &val, const double &lower,
                           const double &upper) {
  assert(lower <= upper);
  if (val < lower) {
    val = lower;
    return true;
  }
  if (upper < val) {
    val = upper;
    return true;
  }
  return false;
}

#define CHECK_BOUNDS(val, lower, upper, what)                                  \
  for (int i = 0; i < val.size(); ++i) {                                       \
    double old = val(i);                                                       \
    if (saturateBounds(val(i), lower(i), upper(i))) {                          \
      std::ostringstream oss;                                                  \
      oss << "Robot " what " bound violation at DoF " << i << ": requested "   \
          << old << " but set " << val(i) << '\n';                             \
      SEND_ERROR_STREAM_MSG(oss.str());                                        \
    }                                                                          \
  }

void Device::integrate(const double &dt) {
  const Vector &controlIN = controlSIN.accessCopy();

  if (sanityCheck_ && controlIN.hasNaN()) {
    dgRTLOG() << "Device::integrate: Control has NaN values: " << '\n'
              << controlIN.transpose() << '\n';
    return;
  }

  if (controlInputType_ == CONTROL_INPUT_NO_INTEGRATION) {
    state_.tail(controlIN.size()) = controlIN;
    return;
  }

  if (vel_control_.size() == 0)
    vel_control_ = Vector::Zero(controlIN.size());

  // If control size is state size - 6, integrate joint angles,
  // if control and state are of same size, integrate 6 first degrees of
  // freedom as a translation and roll pitch yaw.

  if (controlInputType_ == CONTROL_INPUT_TWO_INTEGRATION) {
    // TODO check acceleration
    // Position increment
    vel_control_ = velocity_.tail(controlIN.size()) + (0.5 * dt) * controlIN;
    // Velocity integration.
    velocity_.tail(controlIN.size()) += controlIN * dt;
  } else {
    vel_control_ = controlIN;
  }

  // Velocity bounds check
  if (sanityCheck_) {
    CHECK_BOUNDS(velocity_, lowerVelocity_, upperVelocity_, "velocity");
  }

  if (vel_control_.size() == state_.size()) {
    // Freeflyer integration
    integrateRollPitchYaw(state_, vel_control_, dt);
    // Joints integration
    state_.tail(state_.size() - 6) += vel_control_.tail(state_.size() - 6) * dt;
  } else {
    // Position integration
    state_.tail(controlIN.size()) += vel_control_ * dt;
  }

  // Position bounds check
  if (sanityCheck_) {
    CHECK_BOUNDS(state_, lowerPosition_, upperPosition_, "position");
  }
}

/* --- DISPLAY ------------------------------------------------------------ */

void Device::display(std::ostream &os) const {
  os << name << ": " << state_ << endl
     << "sanityCheck: " << sanityCheck_ << endl
     << "controlInputType:" << controlInputType_ << endl;
}

void Device::cmdDisplay() {
  std::cout << name << ": " << state_ << endl
            << "sanityCheck: " << sanityCheck_ << endl
            << "controlInputType:" << controlInputType_ << endl;
}
