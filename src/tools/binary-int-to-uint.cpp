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

/* --- SOT --- */
#include <dynamic-graph/pool.h>
#include <sot/core/binary-int-to-uint.hh>
#include <sot/core/exception-feature.hh>
#include <sot/core/debug.hh>
using namespace std;

#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(BinaryIntToUint,"BinaryIntToUint");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

BinaryIntToUint::BinaryIntToUint( const string& fname )
  : Entity( fname)
  ,binaryIntSIN( NULL,"BinaryIntToUint("+name+")::input(int)::sin" )
  ,binaryUintSOUT( boost::bind(&BinaryIntToUint::computeOutput,this,_1,_2),
		   binaryIntSIN,
		   "BinaryIntToUint("+name+")::output(unsigned int)::sout" )
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
