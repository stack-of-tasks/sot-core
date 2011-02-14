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
#include "sot-core/debug.h"
using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string Device::CLASS_NAME = "Device";

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

static void integrateRollPitchYaw(ml::Vector& state, const ml::Vector& control,
				  double dt)
{
  jrlMathTools::Vector3D<double> omega;
  // Translation part
  for (unsigned int i=0; i<3; i++) {
    state(i) += control(i)*dt;
    omega(i) = control(i+3);
  }
  // Rotation part
  double roll = state(3);
  double pitch = state(4);
  double yaw = state(5);
  std::vector<jrlMathTools::Vector3D<double> > column;

  // Build rotation matrix as a vector of colums
  jrlMathTools::Vector3D<double> e1;
  e1(0) = cos(pitch)*cos(yaw);
  e1(1) = cos(pitch)*sin(yaw);
  e1(2) = -sin(pitch);
  column.push_back(e1);

  jrlMathTools::Vector3D<double> e2;
  e2(0) = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
  e2(1) = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
  e2(2) = sin(roll)*cos(pitch);
  column.push_back(e2);

  jrlMathTools::Vector3D<double> e3;
  e3(0) = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
  e3(1) = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
  e3(2) = cos(roll)*cos(pitch);
  column.push_back(e3);

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
  const double & nx = column[2](2);
  const double & ny = column[1](2);

  state(3) = atan2(ny,nx);
  state(4) = atan2(-column[0](2),
		    sqrt(ny*ny+nx*nx));
  state(5) = atan2(column[0](1),column[0](0));

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
   ,state(6)
   ,controlSIN( NULL,"Device("+n+")::input(double)::control" )
  //,attitudeSIN(NULL,"Device::input(matrixRot)::attitudeIN")
   ,attitudeSIN(NULL,"Device::input(vector3)::attitudeIN")
   ,zmpSIN(NULL,"Device::input(vector3)::zmp")
   ,stateSOUT( "Device("+n+")::output(vector)::state" )
   ,attitudeSOUT( "Device("+n+")::output(matrixRot)::attitude" )
   ,pseudoTorqueSOUT( "Device::output(vector)::ptorque" )
   ,previousControlSOUT( "Device("+n+")::output(vector)::previousControl" )
   ,motorcontrolSOUT( "Device("+n+")::output(vector)::motorcontrol" )
   ,ZMPPreviousControllerSOUT( "Device("+n+")::output(vector)::zmppreviouscontroller" )
{
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
  state.fill(.0); stateSOUT.setConstant( state );

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
    /* Command set. */
    docstring =
      "\n"
      "    Set state vector value\n"
      "\n";
    addCommand("set",
	       new command::Setter<Device, Vector>
	       (*this, &Device::setState, docstring));
  }
}

void Device::
setStateSize( const unsigned int& size )
{
  state.resize(size); state.fill( .0 );
  stateSOUT .setConstant( state );
  previousControlSOUT.setConstant( state );
  pseudoTorqueSOUT.setConstant( state );
  motorcontrolSOUT .setConstant( state );

  ml::Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );
}

void Device::
setState( const ml::Vector& st )
{
  state = st;
  stateSOUT .setConstant( state );
  motorcontrolSOUT .setConstant( state );
}

void Device::
increment( const double & dt )
{
  sotDEBUG(25) << "Time : " << controlSIN.getTime()+1 << std::endl;

   stateSOUT .setConstant( state );
  const ml::Vector control = controlSIN( controlSIN.getTime()+1 );

  sotDEBUG(25) << "Cl" <<controlSIN.getTime()<<" = "
	       << control*dt << ": " << control << endl;

  sotDEBUG(25) << "St"<<state.size() << controlSIN.getTime() << ": " << state << endl;
  // If control size is state size - 6, integrate joint angles,
  // if control and state are of same size, integrate 6 first degrees of
  // freedom as a translation and roll pitch yaw.
  unsigned int offset = 6;
  if (control.size() == state.size()) {
    offset = 0;
    integrateRollPitchYaw(state, control, dt);
  }
  for( unsigned int i=6;i<state.size();++i )
    { state(i) += (control(i-offset)*dt); }

  sotDEBUG(25) << "St"<<state.size() << controlSIN.getTime() << ": " << state << endl;


   ml::Vector forceNull(6); forceNull.fill(0);
   for( int i=0;i<4;++i ){
     if(  withForceSignals[i] ) forcesSOUT[i]->setConstant(forceNull);
   }

  motorcontrolSOUT .setConstant( state );

  ml::Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );
}

/* --- DISPLAY ------------------------------------------------------------ */

void Device::display ( std::ostream& os ) const
{os <<name<<": "<<state<<endl; }
