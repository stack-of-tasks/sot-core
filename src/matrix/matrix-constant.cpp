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

#include <sot/core/matrix-constant.hh>
#include <sot/core/factory.hh>

#include "../src/matrix/matrix-constant-command.h"

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MatrixConstant,"MatrixConstant");

/* --------------------------------------------------------------------- */
/* --- MATRIX ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

MatrixConstant::
MatrixConstant( const std::string& name )
  :Entity( name )
  ,rows(0),cols(0),color(0.)
  ,SOUT( "sotMatrixConstant("+name+")::output(matrix)::sout" )
{
  SOUT.setDependencyType( TimeDependency<int>::BOOL_DEPENDENT );
  signalRegistration( SOUT );
  //
  // Commands

  // Resize
  std::string docstring;
  docstring = "    \n"
    "    Resize the matrix and fill with value stored in color field.\n"
    "      Input\n"
    "        - unsigned int: number of lines.\n"
    "        - unsigned int: number of columns.\n"
    "\n";
  addCommand("resize",
	     new command::matrixConstant::Resize(*this, docstring));
  // set
  docstring = "    \n"
    "    Set value of output signal\n"
    "    \n"
    "      input:\n"
    "        - a matrix\n"
    "    \n";
  addCommand("set",
	     new ::dynamicgraph::command::Setter<MatrixConstant, ml::Matrix>
	     (*this, &MatrixConstant::setValue, docstring));
}

void MatrixConstant::
setValue(const ml::Matrix& inValue)
{
  SOUT.setConstant(inValue);
}


void MatrixConstant::
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
    






