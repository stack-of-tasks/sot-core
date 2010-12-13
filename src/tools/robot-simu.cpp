/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
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
#include <sot-core/robot-simu.h>
#include <sot-core/debug.h>
using namespace std;

#include <dynamic-graph/factory.h>
using namespace sot;
using namespace dynamicgraph;

#include "tools/robot-simu-command.h"

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RobotSimu,"RobotSimu");


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


RobotSimu::
RobotSimu( const std::string& n )
  :Entity(n)
   ,state(6)
   ,periodicCallBefore( )
   ,periodicCallAfter( )
   ,controlSIN( NULL,"RobotSimu("+n+")::input(double)::control" )
  //,attitudeSIN(NULL,"RobotSimu::input(matrixRot)::attitudeIN")
   ,attitudeSIN(NULL,"RobotSimu::input(vector3)::attitudeIN")
   ,zmpSIN(NULL,"RobotSimu::input(vector3)::zmp")
   ,stateSOUT( "RobotSimu("+n+")::output(vector)::state" )
   ,attitudeSOUT( "RobotSimu("+n+")::output(matrixRot)::attitude" )
   ,pseudoTorqueSOUT( "RobotSimu::output(vector)::ptorque" )
   ,previousControlSOUT( "RobotSimu("+n+")::output(vector)::previousControl" )
   ,motorcontrolSOUT( "RobotSimu("+n+")::output(vector)::motorcontrol" )
   ,ZMPPreviousControllerSOUT( "RobotSimu("+n+")::output(vector)::zmppreviouscontroller" )
{
  /* --- FORCES --- */
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
  //
  // Commands
  //
  std::string docstring;
  // Increment
    "\n"
    "    Integrate dynamics for time step provided as input\n"
    "\n"
    "      take one floating point number as input\n"
    "\n";
  addCommand(std::string("increment"),
	     new command::Increment(*this, docstring));
  // setStateSize
  docstring =
    "\n"
    "    Set size of state vector\n"
    "\n";
  addCommand("resize",
	     new ::dynamicgraph::command::Setter<RobotSimu, unsigned>
	     (*this, &RobotSimu::setStateSize, docstring);
  // set
  docstring =
    "\n"
    "    Set state vector value\n"
    "\n";
  addCommand("set",
	     new ::dynamicgraph::command::Setter<RobotSimu, Vector>
	     (*this, &RobotSimu::set, docstring);
}

void RobotSimu::
setStateSize( const unsigned int size )
{
  state.resize(size); state.fill( .0 ); 
  stateSOUT .setConstant( state );
  previousControlSOUT.setConstant( state );
  pseudoTorqueSOUT.setConstant( state );
  motorcontrolSOUT .setConstant( state );
  
  ml::Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );
}

void RobotSimu::
setState( const ml::Vector& st )
{
  state = st; 
  stateSOUT .setConstant( state );
  motorcontrolSOUT .setConstant( state );
}

void RobotSimu::
increment( const double dt )
{
  sotDEBUG(25) << "Time : " << controlSIN.getTime()+1 << std::endl;

   periodicCallBefore.runSignals( controlSIN.getTime()+1 );
   periodicCallBefore.runCmds();

   stateSOUT .setConstant( state ); 
  const ml::Vector control = controlSIN( controlSIN.getTime()+1 );

  sotDEBUG(25) << "Cl" <<controlSIN.getTime()<<" = "
	       << control*dt << ": " << control << endl;

  sotDEBUG(25) << "St"<<state.size() << controlSIN.getTime() << ": " << state << endl;
  for( unsigned int i=6;i<state.size();++i )
    { state(i) += (control(i-6)*dt); }

  sotDEBUG(25) << "St"<<state.size() << controlSIN.getTime() << ": " << state << endl;


   ml::Vector forceNull(6); forceNull.fill(0);
   for( int i=0;i<4;++i ){ 
     if(  withForceSignals[i] ) forcesSOUT[i]->setConstant(forceNull); 
   }

  motorcontrolSOUT .setConstant( state );
  
  ml::Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );


   periodicCallAfter.runSignals( controlSIN.getTime() );
   periodicCallAfter.runCmds();

}



/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */

void RobotSimu::display ( std::ostream& os ) const
{os <<name<<": "<<state<<endl; }


/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
#include <dynamic-graph/pool.h>

void RobotSimu::
commandLine( const std::string& cmdLine
	     ,std::istringstream& cmdArgs
	     ,std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "RobotSimu: "<<endl
	 << "  - resize <uint size>"<<endl
	 << "  - set <vector>"<<endl
	 << "  - inc [<dt>]"<<endl;
	Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine=="resize" )
    {
      unsigned int size; cmdArgs >> size;
      setStateSize( size ); 
    }
  else if( cmdLine=="set" )
    {
      ml::Vector q; cmdArgs >> q;
      setState( q );
    }
  else if( cmdLine == "pause" ) { os << "Not valid in simu" << endl; }
  else if( cmdLine == "play" )  { os << "Not valid in simu" << endl; }
  else if( cmdLine == "withForces" )
  {
    int index;
    cmdArgs >> index;
    if((index >= 0) && (index < 4))
      {
	std::string val;
	cmdArgs >> val;
	if ( ("1"==val)||("true"==val) )
	  {
	    withForceSignals[index] = true;
	    ml::Vector forceNull(6); forceNull.fill(0);
	    forcesSOUT[index]->setConstant(forceNull); 
	  } else withForceSignals[index] = false;
      }
  }
  else if( cmdLine == "whichForces" )
  {
    os << "Force signals: " << endl; 
    for( unsigned int i=0;i<4;++i )
      {
	os << "\t- Force " << i << ": ";
	if( withForceSignals[i] )
	  { os << "Active";	  } else { os << "Inactive"; } 
	os << endl;
      }
  }
  else if( (cmdLine == "withPreviousControl")||(cmdLine == "withPseudoTorque") )
  {
    // Just for compatibility.
  }
  else if( cmdLine=="inc" )
    {
	increment();

    }
  else if( cmdLine=="time" )
    {
      os << "control.time = " << controlSIN.getTime() << endl;
    }
  else if(( cmdLine=="periodicCall" )||( cmdLine=="periodicCallAfter" ))
    {
      string cmd2; cmdArgs >> cmd2;
      periodicCallAfter .commandLine( cmd2,cmdArgs,os );
    }
  else if( cmdLine=="periodicCallBefore" )
    {
      string cmd2; cmdArgs >> cmd2;
      periodicCallBefore .commandLine( cmd2,cmdArgs,os );
    }
  else  
    {
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
}
