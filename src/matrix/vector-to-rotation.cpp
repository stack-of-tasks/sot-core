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

#include <sot/core/vector-to-rotation.hh>

#include <sot/core/factory.hh>
#include <sot/core/macros-signal.hh>
#include <sot/core/debug.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(VectorToRotation,"VectorToRotation");

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

VectorToRotation::
VectorToRotation( const std::string& name )
  :Entity( name )
  ,size(0),axes(0)
  ,SIN( NULL,"sotVectorToRotation("+name+")::output(vector)::sin" )
  ,SOUT( SOT_MEMBER_SIGNAL_1( VectorToRotation::computeRotation,
			    SIN,ml::Vector),
	 "sotVectorToRotation("+name+")::output(matrixRotation)::sout" )
{

  signalRegistration( SIN << SOUT );
}
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

MatrixRotation& VectorToRotation::
computeRotation( const ml::Vector& angles,
		 MatrixRotation& res )
{
  res.setIdentity();
  MatrixRotation Ra,Rtmp;
  for( unsigned int i=0;i<size;++i )
    {
      Ra.setIdentity();
      const double ca = cos( angles(i) );
      const double sa = sin( angles(i) );
      const unsigned int i_X=0,i_Y=1,i_Z=2;
      switch( axes[i] )
	{
	case AXIS_X:
	  {
	    Ra(i_Y,i_Y) = ca; Ra(i_Y,i_Z) = -sa;
	    Ra(i_Z,i_Y) = sa; Ra(i_Z,i_Z) = ca;
	    break;
	  }
	case AXIS_Y:
	  {
	    Ra(i_Z,i_Z) = ca; Ra(i_Z,i_X) = -sa;
	    Ra(i_X,i_Z) = sa; Ra(i_X,i_X) = ca;
	    break;
	  }
	case AXIS_Z:
	  {
	    Ra(i_X,i_X) = ca; Ra(i_X,i_Y) = -sa;
	    Ra(i_Y,i_X) = sa; Ra(i_Y,i_Y) = ca;
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
void VectorToRotation::
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
      if( i<size )
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
