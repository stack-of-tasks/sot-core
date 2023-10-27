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
#include <cstdint>
#include "sot/core/device.hh"

#include <iostream>
#include <sot/core/debug.hh>
#include <sot/core/macros.hh>
#include <sot/core/integrator.hh>

using namespace std;

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/real-time-logger.h>

#include <Eigen/Geometry>
#include <pinocchio/multibody/liegroup/special-euclidean.hpp>
#include <sot/core/matrix-geometry.hh>
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string Device::CLASS_NAME = "Device";

// Return positive difference between input value and bounds if it saturates,
// 0 if it does not saturate
inline double saturateBounds(double &val, const double &lower,
                             const double &upper) {
  double res = 0;
  assert(lower <= upper);
  if (val < lower) {
    res = lower - val;
    val = lower;
    return res;
  }
  if (upper < val) {
    res = val - upper;
    val = upper;
    return res;
  }
  return res;
}

#define CHECK_BOUNDS(val, lower, upper, what, eps)                           \
  for (size_type i = 0; i < val.size(); ++i) {                               \
    double old = val(i);                                                     \
    if (saturateBounds(val(i), lower(i), upper(i)) > eps) {                  \
      std::ostringstream oss;                                                \
      oss << "Robot " what " bound violation at DoF " << i << ": requested " \
          << old << " but set " << val(i) << '\n';                           \
      SEND_ERROR_STREAM_MSG(oss.str());                                      \
    }                                                                        \
  }

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

Device::~Device() {
  for (std::size_t i = 0; i < 4; ++i) {
    delete forcesSOUT[i];
  }
}

Device::Device(const std::string &n)
    : Entity(n),
      state_(6),
      sanityCheck_(true),
      controlInputType_(POSITION_CONTROL),
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
      lastTimeControlWasRead_(0),
      controlSize_(0),
      forceZero6(6) {
  forceZero6.fill(0);
  /* --- SIGNALS --- */
  for (size_type i = 0; i < 4; ++i) {
    withForceSignals[i] = false;
  }
  forcesSOUT[0] = new Signal<Vector, sigtime_t>(
      "Device(" + n + ")::output(vector6)::forceRLEG");
  forcesSOUT[1] = new Signal<Vector, sigtime_t>(
      "Device(" + n + ")::output(vector6)::forceLLEG");
  forcesSOUT[2] = new Signal<Vector, sigtime_t>(
      "Device(" + n + ")::output(vector6)::forceRARM");
  forcesSOUT[3] = new Signal<Vector, sigtime_t>(
      "Device(" + n + ")::output(vector6)::forceLARM");

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
    docstring =
        "\n"
        "    Set size of state vector\n"
        "\n";
    addCommand("resize", new command::Setter<Device, std::int64_t>(
                             *this, &Device::setStateSize, docstring));
    docstring =
        "\n"
        "    Set state vector value\n"
        "\n";
    addCommand("set", new command::Setter<Device, Vector>(
                          *this, &Device::setState, docstring));

    docstring =
        "\n"
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
    docstring =
        "\n"
        "    Set the position calculous starting from  \n"
        "    acceleration measure instead of velocity \n"
        "\n";

    addCommand("setSecondOrderIntegration",
               command::makeCommandVoid0(
                   *this, &Device::setSecondOrderIntegration, docstring));

    /* Display information */
    docstring =
        "\n"
        "    Display information on device  \n"
        "\n";
    addCommand("display", command::makeCommandVoid0(*this, &Device::cmdDisplay,
                                                    docstring));

    /* SET of control input type. */
    docstring =
        "\n"
        "    Set the type of control input which can be  \n"
        "    acceleration, velocity, or position\n"
        "\n";

    addCommand("setControlInputType",
               new command::Setter<Device, string>(
                   *this, &Device::setControlInputType, docstring));

    docstring =
        "\n"
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
  }
}

void Device::getControl(map<string, ControlValues> &controlOut,
                        const double &period) {
  sotDEBUGIN(25);
  std::vector<double> control;
  typedef std::vector<double>::size_type control_stype;
  lastTimeControlWasRead_ += (sigtime_t)floor(period / Integrator::dt);

  Vector dgControl(controlSIN(lastTimeControlWasRead_));
  // Specify the joint values for the controller.
  control.resize(static_cast<control_stype>(dgControl.size()));

  if (controlInputType_ == POSITION_CONTROL) {
    CHECK_BOUNDS(dgControl, lowerPosition_, upperPosition_, "position", 1e-6);
  }
  for (size_type i = 0; i < dgControl.size(); ++i)
    control[static_cast<control_stype>(i)] = dgControl[i];
  controlOut["control"].setValues(control);
  sotDEBUGOUT(25);
}

void Device::setStateSize(const std::int64_t &size) {
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

void Device::setControlSize(const size_type &size) { controlSize_ = size; }

size_type Device::getControlSize() const { return controlSize_; }

void Device::setVelocitySize(const size_type &size) {
  velocity_.resize(size);
  velocity_.fill(.0);
  velocitySOUT.setConstant(velocity_);
}

void Device::setState(const Vector &) {}

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
  q = worldMwaist.translation();  // abusive ... but working.
  for (Eigen::Index i = 0; i < 3; ++i) q(i + 3) = r(i);
}

void Device::setSecondOrderIntegration() {}

void Device::setNoIntegration() {}

void Device::setControlInputType(const std::string &) {}

void Device::setSanityCheck(const bool &) {}

void Device::setPositionBounds(const Vector &lower, const Vector &upper) {
  std::ostringstream oss;
  if (lower.size() != controlSize_) {
    oss << "Lower bound size should be " << controlSize_ << ", got "
        << lower.size();
    throw std::invalid_argument(oss.str());
  }
  if (upper.size() != controlSize_) {
    oss << "Upper bound size should be " << controlSize_ << ", got "
        << upper.size();
    throw std::invalid_argument(oss.str());
  }
  lowerPosition_ = lower;
  upperPosition_ = upper;
}

void Device::setVelocityBounds(const Vector &lower, const Vector &upper) {
  std::ostringstream oss;
  if (lower.size() != controlSize_) {
    oss << "Lower bound size should be " << controlSize_ << ", got "
        << lower.size();
    throw std::invalid_argument(oss.str());
  }
  if (upper.size() != controlSize_) {
    oss << "Upper bound size should be " << controlSize_ << ", got "
        << upper.size();
    throw std::invalid_argument(oss.str());
  }
  lowerVelocity_ = lower;
  upperVelocity_ = upper;
}

void Device::setTorqueBounds(const Vector &lower, const Vector &upper) {
  // TODO I think the torque bounds size are controlSize_-6...
  std::ostringstream oss;
  if (lower.size() != controlSize_) {
    oss << "Lower bound size should be " << controlSize_ << ", got "
        << lower.size();
    throw std::invalid_argument(oss.str());
  }
  if (upper.size() != controlSize_) {
    oss << "Lower bound size should be " << controlSize_ << ", got "
        << upper.size();
    throw std::invalid_argument(oss.str());
  }
  lowerTorque_ = lower;
  upperTorque_ = upper;
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
