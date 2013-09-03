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

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include "sot/core/vector-roll-pitch-yaw.hh"
#include "sot/core/periodic-call.hh"
#include "sot/core/matrix-homogeneous.hh"
#include "sot/core/api.hh"

namespace dynamicgraph {
  namespace sot {

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
      dynamicgraph::Vector state_;
      dynamicgraph::Vector velocity_;
      bool vel_controlInit_;
      dynamicgraph::Vector vel_control_;
      bool secondOrderIntegration_;
      bool withForceSignals[4];
      PeriodicCall periodicCallBefore_;
      PeriodicCall periodicCallAfter_;
    public:
      
      /* --- CONSTRUCTION --- */
      Device(const std::string& name);
      /* --- DESTRUCTION --- */
      virtual ~Device();
      
      void setStateSize(const unsigned int& size);
      void setState(const dynamicgraph::Vector& st);
      void setVelocitySize(const unsigned int& size);
      void setVelocity(const dynamicgraph::Vector & vel);
      void setSecondOrderIntegration();
      void increment(const double & dt = 5e-2);
      
    public: /* --- DISPLAY --- */
      virtual void display(std::ostream& os) const;
      SOT_CORE_EXPORT friend std::ostream&
	operator<<(std::ostream& os,const Device& r) {
	r.display(os); return os;
      }

    public: /* --- SIGNALS --- */

      dynamicgraph::SignalPtr<dynamicgraph::Vector,int> controlSIN;
      dynamicgraph::SignalPtr<dynamicgraph::Vector,int> attitudeSIN;
      dynamicgraph::SignalPtr<dynamicgraph::Vector,int> zmpSIN;

      dynamicgraph::Signal<dynamicgraph::Vector,int> stateSOUT;
      dynamicgraph::Signal<dynamicgraph::Vector,int> velocitySOUT;
      dynamicgraph::Signal<MatrixRotation,int> attitudeSOUT;
      dynamicgraph::Signal<dynamicgraph::Vector,int>* forcesSOUT[4];

      dynamicgraph::Signal<dynamicgraph::Vector,int> pseudoTorqueSOUT;
      dynamicgraph::Signal<dynamicgraph::Vector,int> previousControlSOUT;

      /*! \brief The current state of the robot from the command viewpoint. */
      dynamicgraph::Signal<dynamicgraph::Vector,int> motorcontrolSOUT;
      /*! \brief The ZMP reference send by the previous controller. */
      dynamicgraph::Signal<dynamicgraph::Vector,int> ZMPPreviousControllerSOUT;

    public: /* --- COMMANDS --- */
      void commandLine(const std::string&, std::istringstream&,
		       std::ostream&){}
    protected:
      /// Compute roll pitch yaw angles of freeflyer joint.
      void integrateRollPitchYaw(dynamicgraph::Vector& state, const dynamicgraph::Vector& control,
				 double dt);
      /// Store Position of free flyer joint
      MatrixHomogeneous ffPose_;
      /// Compute the new position, from the current control.
      virtual void integrate( const double & dt );
    protected:
      /// Get freeflyer pose
      const MatrixHomogeneous& freeFlyerPose() const;
    public:
      void setRoot( const dynamicgraph::Matrix & root );
      void setRoot( const MatrixHomogeneous & worldMwaist );
    private:
      // Intermediate variable to avoid dynamic allocation
      dynamicgraph::Vector forceZero6;
    };
  } // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef SOT_DEVICE_HH */




