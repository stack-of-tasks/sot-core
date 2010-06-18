/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotMatrixConstant.cpp
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

#include <sot-core/matrix-constant.h>
#include <sot-core/factory.h>

using namespace std;
using namespace sot;


SOT_FACTORY_ENTITY_PLUGIN(sotMatrixConstant,"MatrixConstant");

/* --------------------------------------------------------------------- */
/* --- MATRIX ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void sotMatrixConstant::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs, 
	     std::ostream& os )
{
  if( cmdLine == "resize" )
    {
      cmdArgs >>rows>>cols;
      ml::Matrix m(rows,cols);
      m.fill(color);
      SOUT.setConstant(m);
    }
  else if( cmdLine == "fill" )
    {
      cmdArgs >>color;
      ml::Matrix m(rows,cols);
      m.fill(color);
      SOUT.setConstant(m);
    }
  else if( cmdLine == "rand" )
    {
      ml::Matrix m(rows,cols);
      for( int i=0;i<rows;++i )
        for( int j=0;j<cols;++j )
          m(i,j) = ((rand()+0.0)/RAND_MAX*2)-1.;
      SOUT.setConstant(m);
    }
  else if( cmdLine == "ping")
    { SOUT.SignalBase<int>::setReady() ;   }
  else if( cmdLine == "eye") 
    {
      ml::Matrix m(rows,cols); m.setIdentity();
      SOUT.setConstant(m);
    }
  else if( cmdLine == "[]" ) 
    {
      unsigned int i,j; double v;
      cmdArgs >> i >> j >> v;
      ml::Matrix m = SOUT.accessCopy();
      m(i,j) = v;
      SOUT.setConstant(m);
    }
  else if( cmdLine == "help" )
    {
      os << "sotMatrixConstant"<<endl
	 << "  - resize i j\t\t:resize the output to a i x j zero matrix." <<endl
	 << "  - fill x\t\t:fill the matrix with <x> value." <<endl
	 << "  - eye\t\t:fill the matrix with 0s and a diag of 1."<<endl
	 << "  - [] i j x\t\t:set matrix[i,j] = x."<<endl;
    }
  else Entity::commandLine(cmdLine,cmdArgs,os);
}
    






