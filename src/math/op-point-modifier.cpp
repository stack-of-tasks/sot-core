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

#include <dynamic-graph/all-signals.h>

#include <sot-core/op-point-modifier.h>
#include <sot-core/matrix-twist.h>

#include <dynamic-graph/pool.h>
#include <sot-core/factory.h>

using namespace std;
using namespace sot;
using namespace dynamicgraph;


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(OpPointModifier,"OpPointModifier");


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

OpPointModifier::
OpPointModifier( const std::string& name )
  :Entity( name )
   ,transformation()
   ,jacobianSIN(NULL,"OpPointModifior("+name+")::input(matrix)::jacobianIN")
   ,positionSIN(NULL,"OpPointModifior("+name+")::input(matrixhomo)::positionIN")
   ,jacobianSOUT( boost::bind(&OpPointModifier::computeJacobian,this,_1,_2),
		  jacobianSIN,
		  "OpPointModifior("+name+")::output(matrix)::jacobian" )
   ,positionSOUT( boost::bind(&OpPointModifier::computePosition,this,_1,_2),
		  positionSIN,
		  "OpPointModifior("+name+")::output(matrixhomo)::position" )


{
  signalRegistration( jacobianSIN<<positionSIN<<jacobianSOUT<<positionSOUT );
  sotDEBUGINOUT(15);
}

ml::Matrix&
OpPointModifier::computeJacobian( ml::Matrix& res,const int& time )
{
  const ml::Matrix& jacobian = jacobianSIN( time );
  MatrixTwist V( transformation );
  res = V*jacobian;
  return res;
}

MatrixHomogeneous&
OpPointModifier::computePosition( MatrixHomogeneous& res,const int& time )
{
  sotDEBUGIN(15);
  sotDEBUGIN(15) << time << " " << positionSIN.getTime() << positionSOUT.getTime() << endl;
  const MatrixHomogeneous& position = positionSIN( time );
  position.multiply(transformation,res);
  sotDEBUGOUT(15);
  return res;
}

void
OpPointModifier::setTransformation( const MatrixHomogeneous& tr )
{ transformation = tr; }


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
      Signal< MatrixHomogeneous,int > &sig
	= dynamic_cast< Signal< MatrixHomogeneous,int >& >
	(g_pool.getSignal( cmdArgs ));
      setTransformation(sig.accessCopy());
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
