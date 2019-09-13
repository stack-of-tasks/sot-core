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
namespace dg = dynamicgraph;
/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include "sot/core/periodic-call.hh"
#include <sot/core/matrix-geometry.hh>
#include "sot/core/api.hh"

namespace dynamicgraph {
  namespace sot {

    /// Define the type of input expected by the robot
    enum ControlInput
    {
      CONTROL_INPUT_NO_INTEGRATION=0,
      CONTROL_INPUT_ONE_INTEGRATION=1,
      CONTROL_INPUT_TWO_INTEGRATION=2,
      CONTROL_INPUT_SIZE=3
    };

    const std::string ControlInput_s[] =
    {
      "noInteg", "oneInteg", "twoInteg"
    };

    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    class SOT_CORE_EXPORT Device
        :public Entity
    {
    public:
      static const std::string CLASS_NAME;
      virtual const std::string& getClassName(void) const {
        return CLASS_NAME;
      }

      enum ForceSignalSource
      {
        FORCE_SIGNAL_RLEG,
        FORCE_SIGNAL_LLEG,
        FORCE_SIGNAL_RARM,
        FORCE_SIGNAL_LARM
      };

    protected:
      dg::Vector state_;
      dg::Vector velocity_;
      bool sanityCheck_;
      dg::Vector vel_control_;
      ControlInput controlInputType_;
      bool withForceSignals[4];
      PeriodicCall periodicCallBefore_;
      PeriodicCall periodicCallAfter_;
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
      Device(const std::string& name);
      /* --- DESTRUCTION --- */
      virtual ~Device();

      virtual void setStateSize(const unsigned int& size);
      virtual void setState(const dg::Vector& st);
      void setVelocitySize(const unsigned int& size);
      virtual void setVelocity(const dg::Vector & vel);
      virtual void setSecondOrderIntegration();
      virtual void setNoIntegration();
      virtual void setControlInputType(const std::string& cit);
      virtual void increment(const double & dt = 5e-2);

      /// \name Sanity check parameterization
      /// \{
      void setSanityCheck   (const bool & enableCheck);
      void setPositionBounds(const Vector& lower, const Vector& upper);
      void setVelocityBounds(const Vector& lower, const Vector& upper);
      void setTorqueBounds  (const Vector& lower, const Vector& upper);
      /// \}

    public: /* --- DISPLAY --- */
      virtual void display(std::ostream& os) const;
      virtual void cmdDisplay();
      SOT_CORE_EXPORT friend std::ostream&
      operator<<(std::ostream& os,const Device& r) {
        r.display(os); return os;
      }

    public: /* --- SIGNALS --- */

      dynamicgraph::SignalPtr<dg::Vector,int> controlSIN;
      dynamicgraph::SignalPtr<dg::Vector,int> attitudeSIN;
      dynamicgraph::SignalPtr<dg::Vector,int> zmpSIN;

      /// \name Device current state.
      /// \{
      dynamicgraph::Signal<dg::Vector,int> stateSOUT;
      dynamicgraph::Signal<dg::Vector,int> velocitySOUT;
      dynamicgraph::Signal<MatrixRotation,int> attitudeSOUT;
      /*! \brief The current state of the robot from the command viewpoint. */
      dynamicgraph::Signal<dg::Vector,int> motorcontrolSOUT;
      dynamicgraph::Signal<dg::Vector,int> previousControlSOUT;
      /*! \brief The ZMP reference send by the previous controller. */
      dynamicgraph::Signal<dg::Vector,int> ZMPPreviousControllerSOUT;
      /// \}

      /// \name Real robot current state
      /// This corresponds to the real encoders values and take into
      /// account the stabilization step. Therefore, this usually
      /// does *not* match the state control input signal.
      /// \{
      /// Motor positions
      dynamicgraph::Signal<dg::Vector, int> robotState_;
      /// Motor velocities
      dynamicgraph::Signal<dg::Vector, int> robotVelocity_;
      /// The force torque sensors
      dynamicgraph::Signal<dg::Vector,int>* forcesSOUT[4];
      /// Motor torques
      /// \todo why pseudo ?
      dynamicgraph::Signal<dg::Vector,int> pseudoTorqueSOUT;
      /// \}

    protected:
      /// Compute roll pitch yaw angles of freeflyer joint.
      void integrateRollPitchYaw(dg::Vector& state, const dg::Vector& control,
                                 double dt);
      /// Store Position of free flyer joint
      MatrixHomogeneous ffPose_;
      /// Compute the new position, from the current control.
      ///
      /// When sanity checks are enabled, this checks that the control is within
      /// bounds. There are three cases, depending on what the control is:
      /// - position: checks that the position is within bounds,
      /// - velocity: checks that the velocity and the future position are
      ///             within bounds,
      /// - acceleration: checks that the acceleration, the future velocity and
      ///                 position are within bounds.
      ///                 \todo in order to check the acceleration, we need
      ///                 pinocchio and the contact forces in order to estimate
      ///                 the joint torques for the given acceleration.
      virtual void integrate( const double & dt );
    protected:
      /// Get freeflyer pose
      const MatrixHomogeneous& freeFlyerPose() const;
    public:
      virtual void setRoot( const dg::Matrix & root );


      virtual void setRoot( const MatrixHomogeneous & worldMwaist );
    private:
      // Intermediate variable to avoid dynamic allocation
      dg::Vector forceZero6;
    };
  } // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef SOT_DEVICE_HH */
