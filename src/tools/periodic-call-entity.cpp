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

/* --- SOT --- */
#include <sot/core/periodic-call-entity.hh>
#include <dynamic-graph/pool.h>
#include <sot/core/debug.hh>
#include <sot/core/factory.hh>

using namespace std;
using namespace dynamicgraph::sot;


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PeriodicCallEntity,"PeriodicCallEntity");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */



PeriodicCallEntity::
PeriodicCallEntity( const string& fName )
  : Entity( fName )
  ,PeriodicCall()
  ,triger( "Tracer("+fName+")::triger" )
  ,trigerOnce( "Tracer("+fName+")::trigerOnce" )
{
  signalRegistration( triger << trigerOnce );

  triger.setFunction( boost::bind(&PeriodicCallEntity::trigerCall,this,_1,_2) );
  trigerOnce.setFunction( boost::bind(&PeriodicCallEntity::trigerOnceCall,
				      this,_1,_2) );

}

int& PeriodicCallEntity::
trigerCall( int& dummy,const int & time )
{
  run(time);
  return dummy;
}
int& PeriodicCallEntity::
trigerOnceCall( int& dummy,const int & time )
{
  run( time );
  clear();
  return dummy;
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


void PeriodicCallEntity::
display( std::ostream& os ) const
{
  os << "PeriodicCallEntity <"<<name<<"> ";
  PeriodicCall::display( os );
}




void PeriodicCallEntity::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if(! PeriodicCall::commandLine( cmdLine,cmdArgs,os) )
    { Entity::commandLine( cmdLine,cmdArgs,os ); }
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

