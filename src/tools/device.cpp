/*
 * Copyright 2010,
 * Nicolas Mansard, Olivier Stasse, François Bleibel, Florent Lamiraux
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

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#define ENABLE_RT_LOG

#include "sot/core/device.hh"
#include <sot/core/debug.hh>
using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/real-time-logger.h>
#include <dynamic-graph/all-commands.h>
#include <Eigen/Geometry>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/matrix-geometry.hh>

#include <pinocchio/multibody/liegroup/special-euclidean.hpp>
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string Device::CLASS_NAME = "Device";

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void Device::integrateRollPitchYaw(Vector& state, const Vector& control,
                                   double dt)
{
  using Eigen::AngleAxisd;
  using Eigen::Vector3d;
  using Eigen::QuaternionMapd;

  typedef se3::SpecialEuclideanOperation<3> SE3;
  Eigen::Matrix<double, 7, 1> qin, qout;
  qin.head<3>() = state.head<3>();

  QuaternionMapd quat (qin.tail<4>().data());
  quat = AngleAxisd(state(5), Vector3d::UnitZ())
       * AngleAxisd(state(4), Vector3d::UnitY())
       * AngleAxisd(state(3), Vector3d::UnitX());

  SE3::integrate (qin, control.head<6>()*dt, qout);

  // Update freeflyer pose
  ffPose_.translation() = qout.head<3>();
  state.head<3>() = qout.head<3>();

  ffPose_.linear() = QuaternionMapd(qout.tail<4>().data()).toRotationMatrix();
  state.segment<3>(3) = ffPose_.linear().eulerAngles(2,1,0).reverse();
}

const MatrixHomogeneous& Device::freeFlyerPose() const
{
  return ffPose_;
}

Device::
~Device( )
{
  for( unsigned int i=0; i<4; ++i ) {
    delete forcesSOUT[i];
  }
}

Device::
Device( const std::string& n )
  :Entity(n)
  ,state_(6)
  ,vel_controlInit_(false)
  ,controlInputType_(CONTROL_INPUT_ONE_INTEGRATION)
  ,controlSIN( NULL,"Device("+n+")::input(double)::control" )   
  ,attitudeSIN(NULL,"Device("+ n +")::input(vector3)::attitudeIN")
  ,zmpSIN(NULL,"Device("+n+")::input(vector3)::zmp")
  ,stateSOUT( "Device("+n+")::output(vector)::state" )   
  ,robotState_("Device(" + n + ")::output(vector)::robotState")
  ,robotVelocity_("Device(" + n + ")::output(vector)::robotVelocity")
  //,attitudeSIN(NULL,"Device::input(matrixRot)::attitudeIN")
  ,velocitySOUT( "Device("+n+")::output(vector)::velocity"  )
  ,attitudeSOUT( "Device("+n+")::output(matrixRot)::attitude" )
  ,motorcontrolSOUT   ( "Device("+n+")::output(vector)::motorcontrol" )
  ,previousControlSOUT( "Device("+n+")::output(vector)::previousControl" )
  ,ZMPPreviousControllerSOUT( "Device("+n+")::output(vector)::zmppreviouscontroller" )

  ,robotState_     ("Device("+n+")::output(vector)::robotState")
  ,robotVelocity_  ("Device("+n+")::output(vector)::robotVelocity")
  ,pseudoTorqueSOUT("Device("+n+")::output(vector)::ptorque" )

  ,ffPose_()
  ,forceZero6 (6)
{
  forceZero6.fill (0);
  /* --- SIGNALS --- */
  for( int i=0;i<4;++i ){ withForceSignals[i] = false; }
  forcesSOUT[0] =
      new Signal<Vector, int>("Device("+n+")::output(vector6)::forceRLEG");
  forcesSOUT[1] =
      new Signal<Vector, int>("Device("+n+")::output(vector6)::forceLLEG");
  forcesSOUT[2] =
      new Signal<Vector, int>("Device("+n+")::output(vector6)::forceRARM");
  forcesSOUT[3] =
      new Signal<Vector, int>("Device("+n+")::output(vector6)::forceLARM");

  signalRegistration( controlSIN<<stateSOUT<<robotState_<<robotVelocity_
                      <<velocitySOUT<<attitudeSOUT
                      <<attitudeSIN<<zmpSIN <<*forcesSOUT[0]<<*forcesSOUT[1]
                      <<*forcesSOUT[2]<<*forcesSOUT[3] <<previousControlSOUT
                      <<pseudoTorqueSOUT << motorcontrolSOUT << ZMPPreviousControllerSOUT );
  state_.fill(.0); stateSOUT.setConstant( state_ );

  velocity_.resize(state_.size()); velocity_.setZero();
  velocitySOUT.setConstant( velocity_ );

  /* --- Commands --- */
  {
    std::string docstring;
    /* Command setStateSize. */
    docstring =
        "\n"
        "    Set size of state vector\n"
        "\n";
    addCommand("resize",
               new command::Setter<Device, unsigned int>
               (*this, &Device::setStateSize, docstring));
    docstring =
        "\n"
        "    Set state vector value\n"
        "\n";
    addCommand("set",
               new command::Setter<Device, Vector>
               (*this, &Device::setState, docstring));

    docstring =
        "\n"
        "    Set velocity vector value\n"
        "\n";
    addCommand("setVelocity",
               new command::Setter<Device, Vector>
               (*this, &Device::setVelocity, docstring));

    void(Device::*setRootPtr)(const Matrix&) = &Device::setRoot;
    docstring
        = command::docCommandVoid1("Set the root position.",
                                   "matrix homogeneous");
    addCommand("setRoot",
               command::makeCommandVoid1(*this,setRootPtr,
                                         docstring));

    /* Second Order Integration set. */
    docstring =
        "\n"
        "    Set the position calculous starting from  \n"
        "    acceleration measure instead of velocity \n"
        "\n";

    addCommand("setSecondOrderIntegration",
               command::makeCommandVoid0(*this,&Device::setSecondOrderIntegration,
                                         docstring));

    /* SET of control input type. */
    docstring =
        "\n"
        "    Set the type of control input which can be  \n"
        "    acceleration, velocity, or position\n"
        "\n";

    addCommand("setControlInputType",
               new command::Setter<Device,string>
               (*this, &Device::setControlInputType, docstring));

    // Handle commands and signals called in a synchronous way.
    periodicCallBefore_.addSpecificCommands(*this, commandMap, "before.");
    periodicCallAfter_.addSpecificCommands(*this, commandMap, "after.");

  }
}

void Device::
setStateSize( const unsigned int& size )
{
  state_.resize(size); state_.fill( .0 );
  stateSOUT .setConstant( state_ );
  previousControlSOUT.setConstant( state_ );
  pseudoTorqueSOUT.setConstant( state_ );
  motorcontrolSOUT .setConstant( state_ );

  Device::setVelocitySize(size);

  Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );
}

void Device::
setVelocitySize( const unsigned int& size )
{
  velocity_.resize(size);
  velocity_.fill(.0);
  velocitySOUT.setConstant( velocity_ );
}

void Device::
setState( const Vector& st )
{
  state_ = st;
  stateSOUT .setConstant( state_ );
  motorcontrolSOUT .setConstant( state_ );
}

void Device::
setVelocity( const Vector& vel )
{
  velocity_ = vel;
  velocitySOUT .setConstant( velocity_ );
}

void Device::
setRoot( const Matrix & root )
{
  Eigen::Matrix4d _matrix4d(root);
  MatrixHomogeneous _root(_matrix4d);
  setRoot( _root );
}

void Device::
setRoot( const MatrixHomogeneous & worldMwaist )
{
  VectorRollPitchYaw r = (worldMwaist.linear().eulerAngles(2,1,0)).reverse();
  Vector q = state_;
  q = worldMwaist.translation(); // abusive ... but working.
  for( unsigned int i=0;i<3;++i ) q(i+3) = r(i);
}

void Device::
setSecondOrderIntegration()
{
  controlInputType_ = CONTROL_INPUT_TWO_INTEGRATION;
  velocity_.resize(state_.size());
  velocity_.setZero();
  velocitySOUT.setConstant( velocity_ );
}

void Device::
setNoIntegration()
{
  controlInputType_ = CONTROL_INPUT_NO_INTEGRATION;
  velocity_.resize(state_.size());
  velocity_.setZero();
  velocitySOUT.setConstant( velocity_ );
}

void Device::
setControlInputType(const std::string& cit)
{
  for(int i=0; i<CONTROL_INPUT_SIZE; i++)
    if(cit==ControlInput_s[i])
    {
      controlInputType_ = (ControlInput)i;
      sotDEBUG(25)<<"Control input type: "<<ControlInput_s[i]<<endl;
      return;
    }
  sotDEBUG(25)<<"Unrecognized control input type: "<<cit<<endl;
}

void Device::
increment( const double & dt )
{
  int time = stateSOUT.getTime();
  sotDEBUG(25) << "Time : " << time << std::endl;

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
  {
    periodicCallBefore_.run(time+1);
  }
  catch (std::exception& e)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (before): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (before): "
        << str << std::endl;
  }
  catch (...)
  {
    dgRTLOG()
        << "unknown exception caught while"
        << " running periodical commands (before)" << std::endl;
  }


  /* Force the recomputation of the control. */
  controlSIN( time );
  sotDEBUG(25) << "u" <<time<<" = " << controlSIN.accessCopy() << endl;

  /* Integration of numerical values. This function is virtual. */
  integrate( dt );
  sotDEBUG(25) << "q" << time << " = " << state_ << endl;

  /* Position the signals corresponding to sensors. */
  stateSOUT .setConstant( state_ ); stateSOUT.setTime( time+1 );
  //computation of the velocity signal
  if( controlInputType_==CONTROL_INPUT_TWO_INTEGRATION )
  {
    velocitySOUT.setConstant( velocity_ );
    velocitySOUT.setTime( time+1 );
  }
  else if (controlInputType_==CONTROL_INPUT_ONE_INTEGRATION)
  {
    velocitySOUT.setConstant( controlSIN.accessCopy() );
    velocitySOUT.setTime( time+1 );
  }
  for( int i=0;i<4;++i ){
    if(  !withForceSignals[i] ) forcesSOUT[i]->setConstant(forceZero6);
  }
  Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
  {
    periodicCallAfter_.run(time+1);
  }
  catch (std::exception& e)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (after): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (after): "
        << str << std::endl;
  }
  catch (...)
  {
    dgRTLOG()
        << "unknown exception caught while"
        << " running periodical commands (after)" << std::endl;
  }


  // Others signals.
  motorcontrolSOUT .setConstant( state_ );
}

void Device::integrate( const double & dt )
{
  const Vector & controlIN = controlSIN.accessCopy();

  if (controlInputType_==CONTROL_INPUT_NO_INTEGRATION)
  {
    assert(state_.size()==controlIN.size()+6);
    state_.tail(controlIN.size()) = controlIN;
    return;
  }

  if( vel_control_.size() == 0 )
    vel_control_ = Vector::Zero(controlIN.size());

  // If control size is state size - 6, integrate joint angles,
  // if control and state are of same size, integrate 6 first degrees of
  // freedom as a translation and roll pitch yaw.

  if (controlInputType_==CONTROL_INPUT_TWO_INTEGRATION)
  {
    // Position increment
    vel_control_ = velocity_.tail(controlIN.size()) + (0.5*dt)*controlIN;
    // Velocity integration.
    velocity_.tail(controlIN.size()) += controlIN*dt;
  }
  else
  {
    vel_control_ = controlIN;
  }

  if (vel_control_.size() == state_.size()) {
    integrateRollPitchYaw(state_, vel_control_, dt);
  }

  // Position integration
  state_.tail(controlIN.size()) += vel_control_ * dt;
}

/* --- DISPLAY ------------------------------------------------------------ */

void Device::display ( std::ostream& os ) const
{os <<name<<": "<<state_<<endl; }
