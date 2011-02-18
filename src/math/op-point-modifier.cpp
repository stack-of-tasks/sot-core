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

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/all-commands.h>

#include <sot/core/op-point-modifier.hh>
#include <sot/core/matrix-twist.hh>


using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(OpPointModifier,"OpPointModifier");


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

OpPointModifier::
OpPointModifier( const std::string& name )
  :Entity( name )
   ,jacobianSIN(NULL,"OpPointModifior("+name+")::input(matrix)::jacobianIN")
   ,positionSIN(NULL,"OpPointModifior("+name+")::input(matrixhomo)::positionIN")
   ,jacobianSOUT( boost::bind(&OpPointModifier::jacobianSOUT_function,this,_1,_2),
		  jacobianSIN,
		  "OpPointModifior("+name+")::output(matrix)::jacobian" )
   ,positionSOUT( boost::bind(&OpPointModifier::positionSOUT_function,this,_1,_2),
		  positionSIN,
		  "OpPointModifior("+name+")::output(matrixhomo)::position" )
   ,transformation()
{
  sotDEBUGIN(15);

  signalRegistration( jacobianSIN<<positionSIN<<jacobianSOUT<<positionSOUT );

  {
    using namespace dynamicgraph::command;
    addCommand("getTransformation",
	       makeDirectGetter(*this,&(ml::Matrix&)transformation,
				docDirectGetter("transformation","matrix 4x4 homo")));
    addCommand("setTransformation",
	       makeDirectSetter(*this, &(ml::Matrix&)transformation,
				docDirectSetter("dimension","matrix 4x4 homo")));
  }

  sotDEBUGOUT(15);
}

ml::Matrix&
OpPointModifier::jacobianSOUT_function( ml::Matrix& res,const int& iter )
{
  const ml::Matrix& jacobian = jacobianSIN( iter );
  MatrixTwist V( transformation.inverse () );
  res = V * jacobian;
  return res;
}

MatrixHomogeneous&
OpPointModifier::positionSOUT_function( MatrixHomogeneous& res,const int& iter )
{
  sotDEBUGIN(15);
  sotDEBUGIN(15) << iter << " " << positionSIN.getTime()
		 << positionSOUT.getTime() << endl;
  const MatrixHomogeneous& position = positionSIN( iter );
  position.multiply(transformation,res);
  sotDEBUGOUT(15);
  return res;
}

void
OpPointModifier::setTransformation( const MatrixHomogeneous& tr )
{ transformation = tr; }
const MatrixHomogeneous&
OpPointModifier::getTransformation( void )
{ return transformation; }


/* The following function needs an access to a specific signal via
 * the pool, using the signal path <entity.signal>. this functionnality
 * is deprecated, and the following function will have to be removed
 * in a near future. A similar functionality is available using
 * the <setTransformation> mthod, bound in python.
 */
#include <dynamic-graph/pool.h>
void
OpPointModifier::setTransformationBySignalName( std::istringstream& cmdArgs )
{
  Signal< MatrixHomogeneous,int > &sig
    = dynamic_cast< Signal< MatrixHomogeneous,int >& >
    (g_pool.getSignal( cmdArgs ));
  setTransformation(sig.accessCopy());
}

void OpPointModifier::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine == "transfo" )
    {
      MatrixHomogeneous tr;
      cmdArgs >> tr;
      setTransformation(tr);
    }
  else if( cmdLine == "transfoSignal" )
    {
      setTransformationBySignalName(cmdArgs);
    }
  else if( cmdLine == "getTransfo" )
    {
      os << "Transformation: " << endl << transformation <<endl;
    }
  else if( cmdLine == "help" )
    {
      os << "sotOpPointModifior"<<endl
	 << "  - transfo MatrixHomo"<<endl
	 << "  - transfoSignal ent.signal<matrixhomo>"<<endl
	 << "  - getTransfo"<<endl;
    }
  else Entity::commandLine(cmdLine,cmdArgs,os);
}
