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
#include <sot-core/time-stamp.h>
#include <sot-core/matrix-homogeneous.h>
#include <dynamic-graph/factory.h>
#include <sot-core/macros-signal.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

using namespace dynamicgraph;
using namespace sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TimeStamp,"TimeStamp");


/* --- CONSTRUCTION ---------------------------------------------------- */
TimeStamp::
TimeStamp( const std::string& name )
  :Entity(name)
   ,offsetValue( 0 ),offsetSet(false)
    ,timeSOUT( "TimeStamp("+name+")::output(vector2)::time" )
   ,timeDoubleSOUT( "TimeStamp("+name+")::output(double)::timeDouble" )
   ,timeOnceSOUT( boost::bind(&TimeStamp::getTimeStamp,this,_1,_2),
		  sotNOSIGNAL,
		  "TimeStamp("+name+")::output(vector2)::synchro" )
   ,timeOnceDoubleSOUT(  boost::bind(&TimeStamp::getTimeStampDouble,this,
				     SOT_CALL_SIG(timeSOUT,ml::Vector),_1),
			 timeSOUT,
			"TimeStamp("+name+")::output(double)::synchroDouble" )
{
  sotDEBUGIN(15);
  
  timeSOUT.setFunction( boost::bind(&TimeStamp::getTimeStamp,this,_1,_2) );
  timeDoubleSOUT.setFunction( boost::bind(&TimeStamp::getTimeStampDouble,this,
 					  SOT_CALL_SIG(timeSOUT,ml::Vector),_1) );
  timeOnceSOUT.setNeedUpdateFromAllChildren( true );
  timeOnceDoubleSOUT.setNeedUpdateFromAllChildren( true );
  signalRegistration( timeSOUT << timeDoubleSOUT 
		      << timeOnceSOUT << timeOnceDoubleSOUT );

  gettimeofday( &val,NULL );

  sotDEBUGOUT(15);
}

/* --- DISPLAY --------------------------------------------------------- */
void TimeStamp::
display( std::ostream& os ) const
{
  os << "TimeStamp <> : " << val.tv_sec << "s; "
     << val.tv_usec << "us." << std::endl;
}

/* --------------------------------------------------------------------- */
/* --- CONTROL --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

ml::Vector& TimeStamp::
getTimeStamp( ml::Vector& res,const int& /*time*/ )
{ 
  sotDEBUGIN(15);
  gettimeofday( &val,NULL );
  if( res.size()!=2 ) res.resize(2);
  

    res(0) = val.tv_sec;
  res(1) = val.tv_usec;
  sotDEBUGOUT(15);
  return res;
}

double& TimeStamp::
getTimeStampDouble( const ml::Vector& vect,double& res )
{ 
  sotDEBUGIN(15);

  if( offsetSet ) res = (vect(0)-offsetValue)*1000;
    else res = vect(0)*1000;
  res += vect(1)/1000;
  sotDEBUGOUT(15);
  return res;
}


void TimeStamp::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "TimeStamp: "<<std::endl
	 << " - offset [{<value>|now}] : set/get the offset for double sig." << std::endl;      
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine=="offset" )
    {
      cmdArgs >> std::ws; 
      if( cmdArgs.good() )
	{ 
	  std::string offnow; 
	  cmdArgs >> offnow;  
	  if(offnow=="now") 
	    {
	      gettimeofday( &val,NULL );
	      offsetValue = val.tv_sec;
	    }
	  else { offsetValue = atoi(offnow.c_str()); }
	  offsetSet = ( offsetValue>0 );
	} else {
	  os << "offset = " << (offsetSet ? offsetValue : 0) << std::endl;
	} 
    }
  else { Entity::commandLine( cmdLine,cmdArgs,os ); }
}


