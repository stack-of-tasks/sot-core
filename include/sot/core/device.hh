/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SOT_DEVICE_HH
#define SOT_DEVICE_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

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
      bool vel_controlInit_;
      dg::Vector vel_control_;
      ControlInput controlInputType_;
      bool withForceSignals[4];
      PeriodicCall periodicCallBefore_;
      PeriodicCall periodicCallAfter_;
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
      
    public: /* --- DISPLAY --- */
      virtual void display(std::ostream& os) const;
      SOT_CORE_EXPORT friend std::ostream&
      operator<<(std::ostream& os,const Device& r) {
        r.display(os); return os;
      }

    public: /* --- SIGNALS --- */

      dynamicgraph::SignalPtr<dg::Vector,int> controlSIN;
      dynamicgraph::SignalPtr<dg::Vector,int> attitudeSIN;
      dynamicgraph::SignalPtr<dg::Vector,int> zmpSIN;

      dynamicgraph::Signal<dg::Vector,int> stateSOUT;
      /// This corresponds to the real encoders values and take into
      /// account the stabilization step. Therefore, this usually
      /// does *not* match the state control input signal.
      ///
      dynamicgraph::Signal<dg::Vector, int> robotState_;
      dynamicgraph::Signal<dg::Vector, int> robotVelocity_;
      dynamicgraph::Signal<dg::Vector,int> velocitySOUT;
      dynamicgraph::Signal<MatrixRotation,int> attitudeSOUT;
      dynamicgraph::Signal<dg::Vector,int>* forcesSOUT[4];

      dynamicgraph::Signal<dg::Vector,int> pseudoTorqueSOUT;
      dynamicgraph::Signal<dg::Vector,int> previousControlSOUT;

      /*! \brief The current state of the robot from the command viewpoint. */
      dynamicgraph::Signal<dg::Vector,int> motorcontrolSOUT;
      /*! \brief The ZMP reference send by the previous controller. */
      dynamicgraph::Signal<dg::Vector,int> ZMPPreviousControllerSOUT;

    public: /* --- COMMANDS --- */
      void commandLine(const std::string&, std::istringstream&,
                       std::ostream&){}
    protected:
      /// Compute roll pitch yaw angles of freeflyer joint.
      void integrateRollPitchYaw(dg::Vector& state, const dg::Vector& control,
                                 double dt);
      /// Store Position of free flyer joint
      MatrixHomogeneous ffPose_;
      /// Compute the new position, from the current control.
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




