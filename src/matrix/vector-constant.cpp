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

#include <sot/core/vector-constant.hh>
#include <sot/core/factory.hh>

#include "../src/matrix/vector-constant-command.h"

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(VectorConstant,"VectorConstant");



/* --------------------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */
VectorConstant::
VectorConstant( const std::string& name )
  :Entity( name )
  ,rows(0),color(0.)
  ,SOUT( "sotVectorConstant("+name+")::output(vector)::sout" )
{
  SOUT.setDependencyType( TimeDependency<int>::BOOL_DEPENDENT );
  signalRegistration( SOUT );

  //
  // Commands
  //
  // Resize
  std::string docstring;
  docstring = "    \n"
    "    Resize the vector and fill with value stored in color field.\n"
    "      Input\n"
    "        unsigned size.\n"
    "\n";
  addCommand("resize",
	     new command::vectorConstant::Resize(*this, docstring));
  // set
  docstring = "    \n"
    "    Set value of output signal\n"
    "    \n"
    "      input:\n"
    "        - a vector\n"
    "    \n";
  addCommand("set",
	     new ::dynamicgraph::command::Setter<VectorConstant, ml::Vector>
	     (*this, &VectorConstant::setValue, docstring));
}

void VectorConstant::
setValue(const ml::Vector& inValue)
{
  SOUT.setConstant(inValue);
}
