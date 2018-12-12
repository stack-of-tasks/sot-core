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
#include <sot/core/gradient-ascent.hh>

namespace dg = ::dynamicgraph;

/* ---------------------------------------------------------------------------*/
/* ------- GENERIC HELPERS -------------------------------------------------- */
/* ---------------------------------------------------------------------------*/

namespace dynamicgraph {
  namespace sot {


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(GradientAscent,"GradientAscent");


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


GradientAscent::
GradientAscent( const std::string& n )
  :Entity(n)
   ,gradientSIN(NULL, "GradientAscent(" + n + ")::input(vector)::gradient")
   ,learningRateSIN(NULL, "GradientAscent(" + n + ")::input(double)::learningRate")
   ,init(false)
   ,refresherSINTERN( "GradientAscent("+n+")::intern(dummy)::refresher"  )
   ,valueSOUT(
      boost::bind(&GradientAscent::update,this,_1,_2),
      gradientSIN << refresherSINTERN, "GradientAscent(" + n + ")::output(vector)::value")
{
  // Register signals into the entity.
  signalRegistration(gradientSIN << learningRateSIN << valueSOUT);
  refresherSINTERN.setDependencyType( TimeDependency<int>::ALWAYS_READY );
}

GradientAscent::~GradientAscent()
{
}

/* --- COMPUTE ----------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */

dynamicgraph::Vector& GradientAscent::update(dynamicgraph::Vector& res,
						 const int& inTime)
{
  const dynamicgraph::Vector& gradient = gradientSIN(inTime);
  const double& learningRate = learningRateSIN(inTime);

  if (init == false) {
    init = true;
    value = gradient;
    value.setZero();
    res.resize(value.size());
  }

  value += learningRate * gradient;
  res = value;
  return res;
}

  } /* namespace sot */
} /* namespace dynamicgraph */
