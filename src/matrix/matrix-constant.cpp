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
	     new ::dynamicgraph::command::Setter<MatrixConstant, dynamicgraph::Matrix>
	     (*this, &MatrixConstant::setValue, docstring));
}

void MatrixConstant::
setValue(const dynamicgraph::Matrix& inValue)
{
  SOUT.setConstant(inValue);
}
