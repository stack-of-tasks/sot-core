/*
 * Copyright 2018,
 * Julian Viereck
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

#include <boost/function.hpp>

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <sot/core/factory.hh>
#include <sot/core/exp-moving-avg.hh>

namespace dg = ::dynamicgraph;

/* ---------------------------------------------------------------------------*/
/* ------- GENERIC HELPERS -------------------------------------------------- */
/* ---------------------------------------------------------------------------*/

namespace dynamicgraph {
  namespace sot {


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ExpMovingAvg,"ExpMovingAvg");


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


ExpMovingAvg::
ExpMovingAvg( const std::string& n )
  :Entity(n)
   ,updateSIN(NULL, "ExpMovingAvg(" + n + ")::input(vector)::update")
   ,alpha(0.)
   ,init(false)
   ,refresherSINTERN( "ExpMovingAvg("+n+")::intern(dummy)::refresher"  )
   ,averageSOUT(
      boost::bind(&ExpMovingAvg::update,this,_1,_2),
      updateSIN << refresherSINTERN, "ExpMovingAvg(" + n + ")::output(vector)::average")
{
  // Register signals into the entity.
  signalRegistration(updateSIN << averageSOUT);
  refresherSINTERN.setDependencyType( TimeDependency<int>::ALWAYS_READY );

  std::string docstring;
  // setAlpha
  docstring =
    "\n"
    "    Set the alpha used to update the current value."
    "\n";
  addCommand(std::string("setAlpha"),
	     new ::dynamicgraph::command::Setter<ExpMovingAvg, double>
	     (*this, &ExpMovingAvg::setAlpha, docstring));
}

ExpMovingAvg::~ExpMovingAvg()
{
}

/* --- COMPUTE ----------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */

void ExpMovingAvg::setAlpha(const double& alpha_) {
  assert(alpha <= 1. && alpha >= 0.);
  alpha = alpha_;
}

dynamicgraph::Vector& ExpMovingAvg::update(dynamicgraph::Vector& res,
						 const int& inTime)
{
  const dynamicgraph::Vector& update = updateSIN(inTime);

  if (init == false) {
    init = true;
    average = update;
    average.setZero();
    res.resize(average.size());
  }

  res = average = alpha * average + (1. - alpha) * update;
  return res;
}



  } /* namespace sot */
} /* namespace dynamicgraph */
