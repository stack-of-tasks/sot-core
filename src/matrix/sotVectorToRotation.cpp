/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotVectorToRotation.cp
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <sot-core/sotVectorToRotation.h>

#include <sot-core/sotFactory.h>
#include <sot-core/sotMacrosSignal.h>
#include <sot-core/sotDebug.h>
SOT_FACTORY_ENTITY_PLUGIN(sotVectorToRotation,"VectorToRotation");

using namespace std;


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

sotVectorToRotation::
sotVectorToRotation( const std::string& name )
  :Entity( name )
  ,size(0),axes(0)
  ,SIN( NULL,"sotVectorToRotation("+name+")::output(vector)::in" )
  ,SOUT( SOT_MEMBER_SIGNAL_1( sotVectorToRotation::computeRotation,
			    SIN,ml::Vector),
	 "sotVectorToRotation("+name+")::output(matrixRotation)::out" )
{

  signalRegistration( SIN << SOUT );
}
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

sotMatrixRotation& sotVectorToRotation::
computeRotation( const ml::Vector& angles,
		 sotMatrixRotation& res )
{
  res.setIdentity();
  sotMatrixRotation Ra,Rtmp;
  for( unsigned int i=0;i<size;++i )
    {
      Ra.setIdentity();
      const double ca = cos( angles(i) );
      const double sa = sin( angles(i) );
      const unsigned int _X=0,_Y=1,_Z=2;
      switch( axes[i] )
	{
	case AXIS_X:
	  {
	    Ra(_Y,_Y) = ca; Ra(_Y,_Z) = -sa;
	    Ra(_Z,_Y) = sa; Ra(_Z,_Z) = ca;
	    break;
	  }
	case AXIS_Y:
	  {
	    Ra(_Z,_Z) = ca; Ra(_Z,_X) = -sa;
	    Ra(_X,_Z) = sa; Ra(_X,_X) = ca;
	    break;
	  }
	case AXIS_Z:
	  {
	    Ra(_X,_X) = ca; Ra(_X,_Y) = -sa;
	    Ra(_Y,_X) = sa; Ra(_Y,_Y) = ca;
	    break;
	  }
	}
      
      sotDEBUG(15) << "R" << i << " = " << Ra;
      res.multiply(Ra,Rtmp);
      res=Rtmp;
    }

  return res;
}


/* --------------------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void sotVectorToRotation::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs, 
	     std::ostream& os )
{
  if( cmdLine == "size" )
    {
      cmdArgs>> std::ws;
      if( cmdArgs.good() )
	{ cmdArgs >>size; axes.resize(size); axes.assign(size,AXIS_X ); }
      else { os <<"size = " << size << std::endl; }
    }
  else if( cmdLine == "axis" ) 
    {
      unsigned int i; cmdArgs >> i >> std::ws;
      if( (i>=0)&&i<size )
	{
	  if( cmdArgs.good() )
	    {
	      std::string axname;  cmdArgs >> axname;
	      if( "X"==axname ) { axes[i] = AXIS_X; }
	      else if( "Y"==axname ) { axes[i] = AXIS_Y; }
	      else if( "Z"==axname ) { axes[i] = AXIS_Z; }
	      else { os << "!! Error in axis name '" << axname << "'" << std::endl; }
	    }
	  else { os << "axis[" << i << "] = " << axes[i] <<std::endl; }
	} else { /* TODO ERROR */ }
    }
  else if( cmdLine == "help" )
    {
      os << "sotVectorToRotation"<<endl
	 << "  - size [<size>] \t\t:get/set the input size." <<endl
	 << "  - axis <axis_num> [X|Y|Z]\t\t:get/set the rotation axis number i."<<endl;
    }
  else Entity::commandLine(cmdLine,cmdArgs,os);
}
