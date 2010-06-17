/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotOpPointModifior.cp
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

#include <dynamic-graph/all-signals.h>

#include <sot-core/op-point-modifier.h>
#include <sot-core/matrix-twist.h>

#include <dynamic-graph/pool.h>
#include <sot-core/factory.h>
SOT_FACTORY_ENTITY_PLUGIN(sotOpPointModifior,"OpPointModifior");

using namespace std;


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

sotOpPointModifior::
sotOpPointModifior( const std::string& name )
  :Entity( name )
   ,transformation()
   ,jacobianSIN(NULL,"OpPointModifior("+name+")::input(matrix)::jacobianIN")
   ,positionSIN(NULL,"OpPointModifior("+name+")::input(matrixhomo)::positionIN")
   ,jacobianSOUT( boost::bind(&sotOpPointModifior::computeJacobian,this,_1,_2),
		  jacobianSIN,
		  "OpPointModifior("+name+")::output(matrix)::jacobian" )
   ,positionSOUT( boost::bind(&sotOpPointModifior::computePosition,this,_1,_2),
		  jacobianSIN,
		  "OpPointModifior("+name+")::output(matrixhomo)::position" )


{
  signalRegistration( jacobianSIN<<positionSIN<<jacobianSOUT<<positionSOUT );
  sotDEBUGINOUT(15);
}

ml::Matrix&
sotOpPointModifior::computeJacobian( ml::Matrix& res,const int& time )
{
  const ml::Matrix& jacobian = jacobianSIN( time );
  sotMatrixTwist V( transformation );
  res = V*jacobian;
  return res;
}

sotMatrixHomogeneous&
sotOpPointModifior::computePosition( sotMatrixHomogeneous& res,const int& time )
{
  sotDEBUGIN(15);
  sotDEBUGIN(15) << time << " " << positionSIN.getTime() << positionSOUT.getTime() << endl;
  const sotMatrixHomogeneous& position = positionSIN( time );
  position.multiply(transformation,res);
  sotDEBUGOUT(15);
  return res;
}

void
sotOpPointModifior::setTransformation( const sotMatrixHomogeneous& tr )
{ transformation = tr; }


void sotOpPointModifior::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs, 
	     std::ostream& os )
{
  if( cmdLine == "transfo" ) 
    {
      sotMatrixHomogeneous tr;
      cmdArgs >> tr;
      setTransformation(tr);
    }
  else if( cmdLine == "transfoSignal" ) 
    {
      Signal< sotMatrixHomogeneous,int > &sig 
	= dynamic_cast< Signal< sotMatrixHomogeneous,int >& > 
	(pool.getSignal( cmdArgs ));
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
