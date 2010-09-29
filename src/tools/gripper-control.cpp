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

#include <sot-core/gripper-control.h>
#include <sot-core/debug.h>
#include <sot-core/factory.h>
#include <sot-core/macros-signal.h>

using namespace sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(GripperControlPlugin,"GripperControl");



/* --- PLUGIN --------------------------------------------------------------- */
/* --- PLUGIN --------------------------------------------------------------- */
/* --- PLUGIN --------------------------------------------------------------- */


#define SOT_FULL_TO_REDUCED( sotName )                                         \
  sotName##FullSizeSIN(NULL,"GripperControl("+name+")::input(vector)::"     \
		            +#sotName+"FullIN")                                  \
  ,sotName##ReduceSOUT( SOT_INIT_SIGNAL_2( GripperControlPlugin::selector,  \
					   sotName##FullSizeSIN,ml::Vector,    \
					   selectionSIN,Flags ),            \
			"GripperControl("+name+")::input(vector)::"         \
			+#sotName+"ReducedOUT") 



const double GripperControl::OFFSET_DEFAULT = 0.9;

GripperControl::
GripperControl(void)
  :offset( GripperControl::OFFSET_DEFAULT )
  ,factor((unsigned int)0)
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
 					     positionSIN,ml::Vector,
 					     torqueSIN,ml::Vector,
 					     upperLimitSIN,ml::Vector,
 					     lowerLimitSIN,ml::Vector,
 					     torqueLimitSIN,ml::Vector ),
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
}


GripperControlPlugin::
~GripperControlPlugin( void )
{
  return;
}


/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */

void GripperControl::
computeIncrement( const ml::Vector& torques,
		  const ml::Vector& torqueLimits,
		  const ml::Vector& currentNormPos )
{
  const unsigned int SIZE = torques.size();
  if( (SIZE==torqueLimits.size())||(SIZE==currentNormPos.size()) )
    { /* ERROR ... */ }
  
  if( factor.size()!=SIZE ) { factor.resize(SIZE); factor.fill(1.); }
  for( unsigned int i=0;i<SIZE;++i )
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
computeNormalizedPosition( const ml::Vector& currentPos,
			   const ml::Vector& upperLim,
			   const ml::Vector& lowerLim,
			   ml::Vector& currentNormPos )
{
  sotDEBUG(25) << "UJL = " << upperLim;
  sotDEBUG(25) << "LJL = " << lowerLim;

  const unsigned int SIZE = currentPos.size();
   if( (SIZE==upperLim.size())||(SIZE==lowerLim.size()) )
     { /* ERROR ... */ } 
   currentNormPos.resize(SIZE);
   for( unsigned int i=0;i<SIZE;++i )
     {
       currentNormPos(i) = ( -1+2*( currentPos(i)-lowerLim(i) )
			     /( upperLim(i)-lowerLim(i) ));
     }
}

void GripperControl::
computeDenormalizedPosition( const ml::Vector& currentNormPos,
			     const ml::Vector& upperLim,
			     const ml::Vector& lowerLim,
			     ml::Vector& currentPos )
{
  const unsigned int SIZE = currentNormPos.size();
   if( (SIZE==upperLim.size())||(SIZE==lowerLim.size()) )
     { /* ERROR ... */ } 
   currentPos.resize(SIZE);
   for( unsigned int i=0;i<SIZE;++i )
     {
       currentPos(i) = ( (currentNormPos(i)+1)*( upperLim(i)-lowerLim(i) )/2
			 +lowerLim(i) );
     }
}

ml::Vector& GripperControl::
computeDesiredPosition( const ml::Vector& currentPos,
			const ml::Vector& torques,
			const ml::Vector& upperLim,
			const ml::Vector& lowerLim,
			const ml::Vector& torqueLimits,
			ml::Vector& desPos )
{
  const unsigned int SIZE = currentPos.size();
  if( (SIZE==torques.size()) )
    { /* ERROR ... */ } 
  desPos.resize(SIZE);
  
  ml::Vector normPos;
  computeNormalizedPosition( currentPos,upperLim,lowerLim,normPos );
  sotDEBUG(25) << "Norm pos = " << normPos;

  computeIncrement( torques,torqueLimits,normPos );
  sotDEBUG(25) << "Factor = " << factor;
  
  ml::Vector desNormPos(SIZE);
  normPos.multiply(factor,desNormPos);

  computeDenormalizedPosition( desNormPos,upperLim,lowerLim,desPos );
  
  return desPos;
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

void GripperControlPlugin::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( "help"==cmdLine )
    {
      os << "GripperControl: " << std::endl
	 << "  - offset [<value>]: set/get the offset value." <<std::endl;
    }
  else if( "offset"==cmdLine )
    {
      cmdArgs>>std::ws; if( cmdArgs.good() )
	{
	  double val; cmdArgs>>val; if( (val>0)&&(val<1) ) offset=val;
	} else {
	  os  << "offset = " << offset << std:: endl; 
	}
    }
  else if( "factor"==cmdLine )
    {
      os  << "factor = " << factor << std:: endl; 
    }
  else{ Entity::commandLine( cmdLine,cmdArgs,os ); }


}

