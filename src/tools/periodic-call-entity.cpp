/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      PeriodicCall.cpp
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


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
#include <sot-core/periodic-call-entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/interpreter.h>
#include <sot-core/debug.h>
#include <sot-core/factory.h>

using namespace std;
using namespace dynamicgraph;
using namespace sot;


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

