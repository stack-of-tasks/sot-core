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

/* SOT */
#include <sot/core/gain-hyperbolic.hh>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/core/factory.hh>
#include <sot/core/debug.hh>
#include <sot/core/exception-signal.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(GainHyperbolic,"GainHyperbolic");

const double GainHyperbolic::
ZERO_DEFAULT = .1;
const double GainHyperbolic::
INFTY_DEFAULT = .1;
const double GainHyperbolic::
TAN_DEFAULT = 1;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#define __SOT_GAIN_HYPERBOLIC_INIT \
Entity(name) \
,coeff_a(0) \
,coeff_b(0) \
,coeff_c(0) \
,coeff_d(0) \
,errorSIN(NULL,"sotGainHyperbolic("+name+")::input(vector)::error") \
,gainSOUT( boost::bind(&GainHyperbolic::computeGain,this,_1,_2), \
	   errorSIN,"sotGainHyperbolic("+name+")::output(double)::gain" )



GainHyperbolic::
GainHyperbolic( const std::string & name )
  :__SOT_GAIN_HYPERBOLIC_INIT
{
  sotDEBUG(15) << "New gain <"<<name<<">"<<std::endl;
  init();
  Entity::signalRegistration( gainSOUT<<errorSIN );
}


GainHyperbolic::
GainHyperbolic( const std::string & name,const double& lambda )
  :__SOT_GAIN_HYPERBOLIC_INIT
{
  init(lambda);
  Entity::signalRegistration( gainSOUT );
}

GainHyperbolic::
GainHyperbolic( const std::string & name,
		   const double& valueAt0,
		   const double& valueAtInfty,
		   const double& tanAt0,
		   const double& decal0 )
  :__SOT_GAIN_HYPERBOLIC_INIT
{
  init(valueAt0,valueAtInfty,tanAt0,decal0);
  Entity::signalRegistration( gainSOUT );
}


void GainHyperbolic::
init( const double& valueAt0,
      const double& valueAtInfty,
      const double& tanAt0,
      const double& decal0 )
{
  coeff_a = valueAt0 - valueAtInfty;
  if (0 == coeff_a)    { coeff_b = 0; }
  else { coeff_b = tanAt0 / coeff_a/2;  }
  coeff_c = valueAtInfty;
  coeff_d = decal0;

  return;
}

void GainHyperbolic::
forceConstant( void )
{
  coeff_a = 0;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void GainHyperbolic::
display( std::ostream& os ) const
{
  os << "Gain Hyperbolic "<<getName();
  try {
    os <<" = "<<double(gainSOUT.accessCopy ());
  } catch (ExceptionSignal e) {}
  //os <<" ("<<coeff_a<<";"<<coeff_b<<";"<<coeff_c<<coeff_d<<") ";
  os <<" ("<<coeff_a<<".exp(-"<<coeff_b<<"(x-" << coeff_d << "))+" <<coeff_c;
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
double& GainHyperbolic::
computeGain( double&res, int t )
{
  sotDEBUGIN(15);
  const dynamicgraph::Vector& error = errorSIN(t);
  const double norm =  error.norm();
  res = coeff_a *.5* (tanh( -coeff_b*(norm-coeff_d) )+1 ) + coeff_c;

  sotDEBUGOUT(15);
  return res;
}
