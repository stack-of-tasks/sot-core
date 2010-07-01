/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      BinaryIntToUint.cpp
 * Project:   SOT
 * Author:    Paul Evrard
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

/* --- SOT --- */
#include <sot-core/pool.h>
#include <sot-core/binary-int-to-uint.h>
#include <sot-core/exception-feature.h>
#include <sot-core/debug.h>
using namespace std;

#include <dynamic-graph/factory.h>

using namespace sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(BinaryIntToUint,"BinaryIntToUint");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

BinaryIntToUint::BinaryIntToUint( const string& fname )
  : Entity( fname)
  ,binaryIntSIN( NULL,"BinaryIntToUint("+name+")::input(int)::in" )
  ,binaryUintSOUT( boost::bind(&BinaryIntToUint::computeOutput,this,_1,_2),
		   binaryIntSIN,
		   "BinaryIntToUint("+name+")::output(unsigned int)::out" )
{
  signalRegistration( binaryIntSIN<<binaryUintSOUT );
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned& BinaryIntToUint::computeOutput( unsigned& res,int time )
{
  sotDEBUGIN(15);

  int in = binaryIntSIN.access(time);
  if(in < 0){ res = 0; }
  else{ res = 1; }

  sotDEBUGOUT(15);
  return res;
}

void BinaryIntToUint::display( std::ostream& os ) const
{
  os << "BinaryIntToUint <" << name << "> TODO..." << endl;
}

void BinaryIntToUint::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine == "help" )
  {
    os << "BinaryIntToUint: "<<endl;
  }
  else{ Entity::commandLine( cmdLine,cmdArgs,os ); }
}
