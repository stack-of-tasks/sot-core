/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS
 *
 */

#ifndef SOT_DEVICE_HH
#define SOT_DEVICE_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <pinocchio/fwd.hpp>

/* -- MaaL --- */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/matrix-geometry.hh>

#include "sot/core/api.hh"

namespace dynamicgraph {
namespace sot {

/// Define the type of input expected by the robot
enum ControlInput {
  POSITION_CONTROL = 0,
  VELOCITY_CONTROL = 1,
  TORQUE_CONTROL = 2,
  CONTROL_SIZE = 3
};

const std::string ControlInput_s[] = {"noInteg", "oneInteg", "twoInteg"};

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOT_CORE_EXPORT Device : public Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  enum ForceSignalSource {
    FORCE_SIGNAL_RLEG,
    FORCE_SIGNAL_LLEG,
    FORCE_SIGNAL_RARM,
    FORCE_SIGNAL_LARM
  };

 protected:
  dynamicgraph::Vector state_;
  dynamicgraph::Vector velocity_;
  bool sanityCheck_;
  dynamicgraph::Vector vel_control_;
  ControlInput controlInputType_;
  bool withForceSignals[4];
  double timestep_;

  /// \name Robot bounds used for sanity checks
  /// \{
  Vector upperPosition_;
  Vector upperVelocity_;
  Vector upperTorque_;
  Vector lowerPosition_;
  Vector lowerVelocity_;
  Vector lowerTorque_;
  /// \}
 public:
  /* --- CONSTRUCTION --- */
  Device(const std::string &name);
  /* --- DESTRUCTION --- */
  virtual ~Device();

  virtual void setStateSize(const size_type &size);
  // Set number of joints that are controlled by the device.
  void setControlSize(const size_type &size);
  // Get the number of joints that are controlled by the device.
  size_type getControlSize() const;
  virtual void setState(const dynamicgraph::Vector &st);
  void setVelocitySize(const size_type &size);
  virtual void setVelocity(const dynamicgraph::Vector &vel);
  virtual void setSecondOrderIntegration();
  virtual void setNoIntegration();
  virtual void setControlInputType(const std::string &cit);
  void getControl(std::map<std::string, ControlValues> &anglesOut,
                  const double &period);

  /// \name Sanity check parameterization
  /// \{
  void setSanityCheck(const bool &enableCheck);
  void setPositionBounds(const Vector &lower, const Vector &upper);
  void setVelocityBounds(const Vector &lower, const Vector &upper);
  void setTorqueBounds(const Vector &lower, const Vector &upper);
  /// \}

 public: /* --- DISPLAY --- */
  virtual void display(std::ostream &os) const;
  virtual void cmdDisplay();
  SOT_CORE_EXPORT friend std::ostream &operator<<(std::ostream &os,
                                                  const Device &r) {
    r.display(os);
    return os;
  }

 public: /* --- SIGNALS --- */
  dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> controlSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> attitudeSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> zmpSIN;

  /// \name Device current state.
  /// \{
  dynamicgraph::Signal<dynamicgraph::Vector, sigtime_t> stateSOUT;
  dynamicgraph::Signal<dynamicgraph::Vector, sigtime_t> velocitySOUT;
  dynamicgraph::Signal<MatrixRotation, sigtime_t> attitudeSOUT;
  /*! \brief The current state of the robot from the command viewpoint. */
  dynamicgraph::Signal<dynamicgraph::Vector, sigtime_t> motorcontrolSOUT;
  dynamicgraph::Signal<dynamicgraph::Vector, sigtime_t> previousControlSOUT;
  /*! \brief The ZMP reference send by the previous controller. */
  dynamicgraph::Signal<dynamicgraph::Vector, sigtime_t>
      ZMPPreviousControllerSOUT;
  /// \}

  /// \name Real robot current state
  /// This corresponds to the real encoders values and take into
  /// account the stabilization step. Therefore, this usually
  /// does *not* match the state control input signal.
  /// \{
  /// Motor positions
  dynamicgraph::Signal<dynamicgraph::Vector, sigtime_t> robotState_;
  /// Motor velocities
  dynamicgraph::Signal<dynamicgraph::Vector, sigtime_t> robotVelocity_;
  /// The force torque sensors
  dynamicgraph::Signal<dynamicgraph::Vector, sigtime_t> *forcesSOUT[4];
  /// Motor torques
  /// \todo why pseudo ?
  dynamicgraph::Signal<dynamicgraph::Vector, sigtime_t> pseudoTorqueSOUT;
  /// \}

 public:
  virtual void setRoot(const dynamicgraph::Matrix &root);

  virtual void setRoot(const MatrixHomogeneous &worldMwaist);

 private:
  sigtime_t lastTimeControlWasRead_;
  size_type controlSize_;
  // Intermediate variable to avoid dynamic allocation
  dynamicgraph::Vector forceZero6;
};
}  // namespace sot
}  // namespace dynamicgraph

#endif /* #ifndef SOT_DEVICE_HH */
