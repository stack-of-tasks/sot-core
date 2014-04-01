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

#include <sot/core/gripper-control.hh>
#include <sot/core/debug.hh>
#include <sot/core/factory.hh>
#include <sot/core/macros-signal.hh>

#include <dynamic-graph/all-commands.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(GripperControlPlugin,"GripperControl");



/* --- PLUGIN --------------------------------------------------------------- */
/* --- PLUGIN --------------------------------------------------------------- */
/* --- PLUGIN --------------------------------------------------------------- */


#define SOT_FULL_TO_REDUCED( sotName )					\
  sotName##FullSizeSIN(NULL,"GripperControl("+name+")::input(vector)::"	\
		       +#sotName+"FullIN")				\
    ,sotName##ReduceSOUT( SOT_INIT_SIGNAL_2( GripperControlPlugin::selector, \
					     sotName##FullSizeSIN,ml::Vector, \
					     selectionSIN,Flags ),	\
			  "GripperControl("+name+")::input(vector)::"	\
			  +#sotName+"ReducedOUT") 



const double GripperControl::OFFSET_DEFAULT = 0.9;

// TODO: hard coded
const double DT = 0.005;

GripperControl::
GripperControl(void)
  :offset( GripperControl::OFFSET_DEFAULT )
  ,factor()
{}

GripperControlPlugin::
GripperControlPlugin( const std::string & name )
  :Entity(name)
  ,calibrationStarted(false)
  ,positionSIN(NULL,"GripperControl("+name+")::input(vector)::position")
  ,positionDesSIN(NULL,"GripperControl("+name+")::input(vector)::positionDes")
  ,torqueSIN(NULL,"GripperControl("+name+")::input(vector)::torque")
  ,torqueLimitSIN(NULL,"GripperControl("+name+")::input(vector)::torqueLimit")
  ,selectionSIN(NULL,"GripperControl("+name+")::input(vector)::selec")
   
  ,SOT_FULL_TO_REDUCED( position )
  ,SOT_FULL_TO_REDUCED( torque )
  ,SOT_FULL_TO_REDUCED( torqueLimit )
  ,desiredPositionSOUT( SOT_MEMBER_SIGNAL_4( GripperControl::computeDesiredPosition,
                                             positionSIN,ml::Vector,
                                             positionDesSIN,ml::Vector,
                                             torqueSIN,ml::Vector,
                                             torqueLimitSIN,ml::Vector ),
                        "GripperControl("+name+")::output(vector)::reference" )
{
  sotDEBUGIN(5);

  positionSIN.plug( &positionReduceSOUT );
  torqueSIN.plug( &torqueReduceSOUT );
  torqueLimitSIN.plug( &torqueLimitReduceSOUT );

  signalRegistration( positionSIN << positionDesSIN
		      << torqueSIN  << torqueLimitSIN  << selectionSIN 
		      << desiredPositionSOUT 
		      << positionFullSizeSIN
		      << torqueFullSizeSIN
		      << torqueLimitFullSizeSIN);
  sotDEBUGOUT(5);

  initCommands();
}


GripperControlPlugin::
~GripperControlPlugin( void )
{}

std::string GripperControlPlugin::
getDocString () const
{
  std::string docstring ="Control of gripper.";
  return docstring;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */

void GripperControl::
computeIncrement( const ml::Vector& torques,
		  const ml::Vector& torqueLimits,
		  const ml::Vector& currentNormVel )
{
  const unsigned int SIZE = currentNormVel.size();

  // initialize factor, if needed.
  if( factor.size()!=SIZE ) { factor.resize(SIZE); factor.fill(1.); }

  // Torque not provided?
  if (torques.size() == 0)
  {
    std::cerr << "torque is not provided " << std::endl;
    return;
  }

  for( unsigned int i=0;i<SIZE;++i )
  {
    // apply a reduction factor if the torque limits are exceeded
    // and the velocity goes in the same way
    if( (torques(i)>torqueLimits(i))&&(currentNormVel(i)>0) )
      { factor(i)*=offset; } 
    else if( (torques(i)< -torqueLimits(i))&&(currentNormVel(i)<0) )
      { factor(i)*=offset; }
    // otherwise, release smoothly the reduction if possible/needed
    else { factor(i)/=offset; }
    
    // ensure factor is in )0,1(
    factor(i) = std::min(1., std::max(factor(i),0.));
  }
}

ml::Vector& GripperControl::
computeDesiredPosition( const ml::Vector& currentPos,
			const ml::Vector& desiredPos,
			const ml::Vector& torques,
			const ml::Vector& torqueLimits,
			ml::Vector& referencePos )
{
  const unsigned int SIZE = currentPos.size();
  //  if( (SIZE==torques.size()) )
  //    { /* ERROR ... */ }

  // compute the desired velocity
  ml::Vector velocity = (desiredPos - currentPos)* (1. / DT);

  computeIncrement(torques, torqueLimits, velocity);  

  sotDEBUG(25) << " velocity " << velocity << std::endl;
  sotDEBUG(25) << " factor " << factor << std::endl;

  // multiply the velocity elmt per elmt
  ml::Vector weightedVel(SIZE);
  velocity.multiply(factor, weightedVel);
  sotDEBUG(25) << " weightedVel " << weightedVel << std::endl;

  // integrate the desired velocity
  referencePos.resize(SIZE);
  referencePos = currentPos + weightedVel * DT;
  return referencePos;
}



ml::Vector& GripperControl::
selector( const ml::Vector& fullsize,
	  const Flags& selec,
	  ml::Vector& desPos )
{
  unsigned int size = 0;
  for( unsigned int i=0;i<fullsize.size();++i )
    { if( selec(i) ) size++; }

  unsigned int curs=0;
  desPos.resize(size);
  for( unsigned int i=0;i<fullsize.size();++i )
    { if( selec(i) ) desPos(curs++)=fullsize(i); }

  return desPos;
}


/* --- COMMANDLINE ---------------------------------------------------------- */
/* --- COMMANDLINE ---------------------------------------------------------- */
/* --- COMMANDLINE ---------------------------------------------------------- */
void GripperControlPlugin::initCommands()
{
  namespace dc = ::dynamicgraph::command;
  addCommand("offset",
    dc::makeCommandVoid1(*this,&GripperControlPlugin::setOffset,
    "set the offset (should be in )0, 1( )."));
}


void GripperControlPlugin::setOffset(const double & value)
{
  if( (value>0)&&(value<1) ) offset = value;
  else std::cerr << "The offset should be in )0, 1(." << std::endl;
}


