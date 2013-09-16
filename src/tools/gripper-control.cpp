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


#define SOT_FULL_TO_REDUCED( sotName )                                         \
  sotName##FullSizeSIN(NULL,"GripperControl("+name+")::input(vector)::"     \
		            +#sotName+"FullIN")                                  \
  ,sotName##ReduceSOUT( SOT_INIT_SIGNAL_2( GripperControlPlugin::selector,  \
					   sotName##FullSizeSIN,dynamicgraph::Vector,    \
					   selectionSIN,Flags ),            \
			"GripperControl("+name+")::input(vector)::"         \
			+#sotName+"ReducedOUT") 



const double GripperControl::OFFSET_DEFAULT = 0.9;

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
  ,upperLimitSIN(NULL,"GripperControl("+name+")::input(vector)::upperLimit")
  ,lowerLimitSIN(NULL,"GripperControl("+name+")::input(vector)::lowerLimit")
  ,torqueSIN(NULL,"GripperControl("+name+")::input(vector)::torque")
  ,torqueLimitSIN(NULL,"GripperControl("+name+")::input(vector)::torqueLimit")
  ,selectionSIN(NULL,"GripperControl("+name+")::input(vector)::selec")
  
   ,SOT_FULL_TO_REDUCED( position )
   ,SOT_FULL_TO_REDUCED( upperLimit )
   ,SOT_FULL_TO_REDUCED( lowerLimit )
   ,SOT_FULL_TO_REDUCED( torque )
   ,SOT_FULL_TO_REDUCED( torqueLimit )
 
  ,desiredPositionSOUT( SOT_MEMBER_SIGNAL_5( GripperControl::computeDesiredPosition,
 					     positionSIN,dynamicgraph::Vector,
 					     torqueSIN,dynamicgraph::Vector,
 					     upperLimitSIN,dynamicgraph::Vector,
 					     lowerLimitSIN,dynamicgraph::Vector,
 					     torqueLimitSIN,dynamicgraph::Vector ),
 			"GripperControl("+name+")::output(vector)::reference" )

{
  sotDEBUGIN(5);
  
  positionSIN.plug( &positionReduceSOUT );
  upperLimitSIN.plug( &upperLimitReduceSOUT );
  lowerLimitSIN.plug( &lowerLimitReduceSOUT );
  torqueSIN.plug( &torqueReduceSOUT );
  torqueLimitSIN.plug( &torqueLimitReduceSOUT );

  signalRegistration( positionSIN  << upperLimitSIN  << lowerLimitSIN  
		      << torqueSIN  << torqueLimitSIN  << selectionSIN 
		      << desiredPositionSOUT 
		      << positionFullSizeSIN << positionReduceSOUT 
		      << upperLimitFullSizeSIN << upperLimitReduceSOUT 
		      << lowerLimitFullSizeSIN << lowerLimitReduceSOUT 
		      << torqueFullSizeSIN << torqueReduceSOUT 
		      << torqueLimitFullSizeSIN << torqueLimitReduceSOUT );
  sotDEBUGOUT(5);

  initCommands();
}


GripperControlPlugin::
~GripperControlPlugin( void )
{
  return;
}

std::string GripperControlPlugin::
getDocString () const
{
  std::string docstring =
    "\n"
    "\n  Control of HRP2 gripper."
    "\n";
  return docstring;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */

void GripperControl::
computeIncrement( const dynamicgraph::Vector& torques,
		  const dynamicgraph::Vector& torqueLimits,
		  const dynamicgraph::Vector& currentNormPos )
{
  const int SIZE = torques.size();
  if( (SIZE==torqueLimits.size())||(SIZE==currentNormPos.size()) )
    { /* ERROR ... */ }
  
  if( factor.size()!=SIZE ) { factor.resize(SIZE); factor.fill(1.); }
  for( int i=0;i<SIZE;++i )
    {
      if( (torques(i)>torqueLimits(i))&&(currentNormPos(i)>0) ) 
	{ factor(i)*=offset; } 
      else if( (torques(i)<-torqueLimits(i))&&(currentNormPos(i)<0) ) 
	{ factor(i)*=offset; } 
      else { factor(i)/=offset; } 
      if( factor(i)>1 ) factor(i)=1;
      if( factor(i)<0 ) factor(i)=0;
    }
}


void GripperControl::
computeNormalizedPosition( const dynamicgraph::Vector& currentPos,
			   const dynamicgraph::Vector& upperLim,
			   const dynamicgraph::Vector& lowerLim,
			   dynamicgraph::Vector& currentNormPos )
{
  sotDEBUG(25) << "UJL = " << upperLim;
  sotDEBUG(25) << "LJL = " << lowerLim;

  const int SIZE = currentPos.size();
   if( (SIZE==upperLim.size())||(SIZE==lowerLim.size()) )
     { /* ERROR ... */ } 
   currentNormPos.resize(SIZE);
   for( int i=0;i<SIZE;++i )
     {
       currentNormPos(i) = ( -1+2*( currentPos(i)-lowerLim(i) )
			     /( upperLim(i)-lowerLim(i) ));
     }
}

void GripperControl::
computeDenormalizedPosition( const dynamicgraph::Vector& currentNormPos,
			     const dynamicgraph::Vector& upperLim,
			     const dynamicgraph::Vector& lowerLim,
			     dynamicgraph::Vector& currentPos )
{
  const int SIZE = currentNormPos.size();
   if( (SIZE==upperLim.size())||(SIZE==lowerLim.size()) )
     { /* ERROR ... */ } 
   currentPos.resize(SIZE);
   for( int i=0;i<SIZE;++i )
     {
       currentPos(i) = ( (currentNormPos(i)+1)*( upperLim(i)-lowerLim(i) )/2
			 +lowerLim(i) );
     }
}

dynamicgraph::Vector& GripperControl::
computeDesiredPosition( const dynamicgraph::Vector& currentPos,
			const dynamicgraph::Vector& torques,
			const dynamicgraph::Vector& upperLim,
			const dynamicgraph::Vector& lowerLim,
			const dynamicgraph::Vector& torqueLimits,
			dynamicgraph::Vector& desPos )
{
  const int SIZE = currentPos.size();
  if( (SIZE==torques.size()) )
    { /* ERROR ... */ } 
  desPos.resize(SIZE);
  
  dynamicgraph::Vector normPos;
  computeNormalizedPosition( currentPos,upperLim,lowerLim,normPos );
  sotDEBUG(25) << "Norm pos = " << normPos;

  computeIncrement( torques,torqueLimits,normPos );
  sotDEBUG(25) << "Factor = " << factor;
  
  dynamicgraph::Vector desNormPos(SIZE);
  desNormPos = normPos*factor;

  computeDenormalizedPosition( desNormPos,upperLim,lowerLim,desPos );
  
  return desPos;
}



dynamicgraph::Vector& GripperControl::
selector( const dynamicgraph::Vector& fullsize,
	  const Flags& selec,
	  dynamicgraph::Vector& desPos )
{
  unsigned int size = 0;
  for( int i=0;i<fullsize.size();++i )
    { if( selec(i) ) size++; }

  unsigned int curs=0;
  desPos.resize(size);
  for( int i=0;i<fullsize.size();++i )
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
    "set the offset (should be in )0, 1(."));
}


void GripperControlPlugin::setOffset(const double & value)
{
  if( (value>0)&&(value<1) ) offset = value;
  else std::cerr << "The offset should be in )0, 1(." << std::endl;
}


