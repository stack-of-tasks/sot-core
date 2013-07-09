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

/* jrl-mathtools */
#include <jrl/mathtools/vector3.hh>

/* SOT */
#include "sot/core/device.hh"
#include <sot/core/debug.hh>
using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string Device::CLASS_NAME = "Device";

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void Device::integrateRollPitchYaw(ml::Vector& state, const ml::Vector& control,
				   double dt)
{
  jrlMathTools::Vector3D<double> omega;
  // Translation part
  for (unsigned int i=0; i<3; i++) {
    state(i) += control(i)*dt;
    ffPose_(i,3) = state(i);
    omega(i) = control(i+3);
  }
  // Rotation part
  double roll = state(3);
  double pitch = state(4);
  double yaw = state(5);
  jrlMathTools::Vector3D<double> column [3];

  // Build rotation matrix as a vector of colums
  column [0](0) = cos(pitch)*cos(yaw);
  column [0](1) = cos(pitch)*sin(yaw);
  column [0](2) = -sin(pitch);

  column [1](0) = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
  column [1](1) = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
  column [1](2) = sin(roll)*cos(pitch);

  column [2](0) = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
  column [2](1) = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
  column [2](2) = cos(roll)*cos(pitch);

  // Apply Rodrigues (1795–1851) formula for rotation about omega vector
  double angle = dt*omega.norm();
  if (angle == 0) {
    return;
  }
  jrlMathTools::Vector3D<double> k = omega/omega.norm();
  // ei <- ei cos(angle) + sin(angle)(k ^ ei) + (k.ei)(1-cos(angle))k
  for (unsigned int i=0; i<3; i++) {
    jrlMathTools::Vector3D<double> ei = column[i];
    column[i] = ei*cos(angle) + (k^ei)*sin(angle) + k*((k*ei)*(1-cos(angle)));
  }
  // Store new position if ffPose_ member.
  for (unsigned int r = 0; r < 3; r++) {
    for (unsigned int c = 0; c < 3; c++) {
      ffPose_(r,c) = column[c](r);
    }
  }
  const double & nx = column[2](2);
  const double & ny = column[1](2);

  state(3) = atan2(ny,nx);
  state(4) = atan2(-column[0](2),
		    sqrt(ny*ny+nx*nx));
  state(5) = atan2(column[0](1),column[0](0));

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
  ,secondOrderIntegration_(false)
  ,vel_controlInit_(false)
  ,controlSIN( NULL,"Device("+n+")::input(double)::control" )
   //,attitudeSIN(NULL,"Device::input(matrixRot)::attitudeIN")
  ,attitudeSIN(NULL,"Device::input(vector3)::attitudeIN")
  ,zmpSIN(NULL,"Device::input(vector3)::zmp")
  ,stateSOUT( "Device("+n+")::output(vector)::state" )
  ,velocitySOUT( "Device("+n+")::output(vector)::velocity"  )
  ,attitudeSOUT( "Device("+n+")::output(matrixRot)::attitude" )
  ,pseudoTorqueSOUT( "Device::output(vector)::ptorque" )
  ,previousControlSOUT( "Device("+n+")::output(vector)::previousControl" )
  ,motorcontrolSOUT( "Device("+n+")::output(vector)::motorcontrol" )
  ,ZMPPreviousControllerSOUT( "Device("+n+")::output(vector)::zmppreviouscontroller" ), ffPose_(),
   forceZero6 (6)
{
  forceZero6.fill (0);
  /* --- SIGNALS --- */
  for( int i=0;i<4;++i ){ withForceSignals[i] = false; }
  forcesSOUT[0] =
    new Signal<ml::Vector, int>("OpenHRP::output(vector6)::forceRLEG");
  forcesSOUT[1] =
    new Signal<ml::Vector, int>("OpenHRP::output(vector6)::forceLLEG");
  forcesSOUT[2] =
    new Signal<ml::Vector, int>("OpenHRP::output(vector6)::forceRARM");
  forcesSOUT[3] =
    new Signal<ml::Vector, int>("OpenHRP::output(vector6)::forceLARM");

  signalRegistration( controlSIN<<stateSOUT<<attitudeSOUT<<attitudeSIN<<zmpSIN
		      <<*forcesSOUT[0]<<*forcesSOUT[1]<<*forcesSOUT[2]<<*forcesSOUT[3]
		      <<previousControlSOUT <<pseudoTorqueSOUT
		      << motorcontrolSOUT << ZMPPreviousControllerSOUT );
  state_.fill(.0); stateSOUT.setConstant( state_ );

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

    /* Command setVelocitySize. */
    docstring =
      "\n"
      "    Set size of velocity vector\n"
      "\n";
    addCommand("resizeVelocity",
	       new command::Setter<Device, unsigned int>
	       (*this, &Device::setVelocitySize, docstring));
    /* Command set. */
    docstring =
      "\n"
      "    Set state vector value\n"
      "\n";
    addCommand("set",
	       new command::Setter<Device, Vector>
	       (*this, &Device::setState, docstring));

    void(Device::*setRootPtr)(const ml::Matrix&) = &Device::setRoot;
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

  ml::Vector zmp(3); zmp.fill( .0 );
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
setState( const ml::Vector& st )
{
  state_ = st;
  stateSOUT .setConstant( state_ );
  motorcontrolSOUT .setConstant( state_ );
}

void Device::
setRoot( const ml::Matrix & root )
{
  setRoot( (MatrixHomogeneous) root );
}
void Device::
setRoot( const MatrixHomogeneous & worldMwaist )
{
  MatrixRotation R; worldMwaist.extract(R);
  VectorRollPitchYaw r; r.fromMatrix(R);

  ml::Vector q = state_;
  worldMwaist.extract(q); // abusive ... but working.
  for( unsigned int i=0;i<3;++i ) q(i+3) = r(i);
}

void Device::
setSecondOrderIntegration()
{
  secondOrderIntegration_ = true;
  signalRegistration( velocitySOUT );
  velocity_.resize(state_.size());
  velocity_.setZero();
  velocitySOUT.setConstant( velocity_ );
}

void Device::
increment( const double & dt )
{
  int time = stateSOUT.getTime();
  sotDEBUG(25) << "Time : " << time << std::endl;

  /* Position the signals corresponding to sensors. */
  stateSOUT .setConstant( state_ ); stateSOUT.setTime( time+1 );
  if( secondOrderIntegration_  )
    {
      velocitySOUT.setConstant( velocity_ );
      velocitySOUT.setTime( time+1 );
    }
  for( int i=0;i<4;++i ){
    if(  !withForceSignals[i] ) forcesSOUT[i]->setConstant(forceZero6);
  }
  ml::Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
    {
      periodicCallBefore_.run(time+1);
    }
  catch (std::exception& e)
    {
      std::cerr
	<< "exception caught while running periodical commands (before): "
	<< e.what () << std::endl;
    }
  catch (const char* str)
    {
      std::cerr
	<< "exception caught while running periodical commands (before): "
	<< str << std::endl;
    }
  catch (...)
    {
      std::cerr
	<< "unknown exception caught while"
	<< " running periodical commands (before)" << std::endl;
    }


  /* Force the recomputation of the control. */
  controlSIN( time+1 );
  sotDEBUG(25) << "u" <<time<<" = " << controlSIN.accessCopy() << endl;

  /* Integration of numerical values. This function is virtual. */
  integrate( dt );
  sotDEBUG(25) << "q" << time << " = " << state_ << endl;

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
    {
      periodicCallAfter_.run(time+1);
    }
  catch (std::exception& e)
    {
      std::cerr
	<< "exception caught while running periodical commands (after): "
	<< e.what () << std::endl;
    }
  catch (const char* str)
    {
      std::cerr
	<< "exception caught while running periodical commands (after): "
	<< str << std::endl;
    }
  catch (...)
    {
      std::cerr
	<< "unknown exception caught while"
	<< " running periodical commands (after)" << std::endl;
    }

  // Others signals.
  motorcontrolSOUT .setConstant( state_ );
}

void Device::integrate( const double & dt )
{
  const ml::Vector & control = controlSIN.accessCopy();
  
  if( !vel_controlInit_ )
    {
      vel_control_ = ml::Vector(control.size());
      vel_control_.setZero();
      vel_controlInit_ = true;
    }

  // If control size is state size - 6, integrate joint angles,
  // if control and state are of same size, integrate 6 first degrees of
  // freedom as a translation and roll pitch yaw.
  unsigned int offset = 6;

  

  if (secondOrderIntegration_)
    {
      for( unsigned int i=0;i<control.size();++i )
	{
	  if(control.size() == velocity_.size()) offset = 0;
	  vel_control_(i) = velocity_(i+offset) + control(i)*dt*0.5;
	  velocity_(i+offset) = velocity_(i+offset) + control(i)*dt;
	}
    }
  else
    {
      vel_control_ = control;
    }

  if (vel_control_.size() == state_.size()) {
    offset = 0;
    integrateRollPitchYaw(state_, vel_control_, dt);
  }
  
  for( unsigned int i=6;i<state_.size();++i )
    { state_(i) += (vel_control_(i-offset)*dt); }
}

/* --- DISPLAY ------------------------------------------------------------ */

void Device::display ( std::ostream& os ) const
{os <<name<<": "<<state_<<endl; }
