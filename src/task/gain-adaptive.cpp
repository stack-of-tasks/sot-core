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
#include <sot-core/gain-adaptive.h>


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot-core/debug.h>
#include <sot-core/factory.h>
#include <sot-core/exception-signal.h>

#include "../src/task/gain-adaptive-command.h"

using namespace sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(GainAdaptive,"GainAdaptive");

const double GainAdaptive::
ZERO_DEFAULT = .1;
const double GainAdaptive::
INFTY_DEFAULT = .1;
const double GainAdaptive::
TAN_DEFAULT = 1;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#define __SOT_GAIN_ADAPTATIVE_INIT \
Entity(name) \
,coeff_a(0) \
,coeff_b(0) \
,coeff_c(0) \
,errorSIN(NULL,"sotGainAdaptive("+name+")::input(vector)::error") \
,gainSOUT( boost::bind(&GainAdaptive::computeGain,this,_1,_2), \
	   errorSIN,"sotGainAdaptive("+name+")::output(double)::gain" )

void GainAdaptive::addCommands()
{
  std::string docstring;
  // Command SetConstant
  docstring = "    \n"
    "    setConstant\n"
    "      Input:\n"
    "        floating point value: value at 0. Other values are set to"
    "default.\n"
    "    \n";
  addCommand("setConstant",
	     new command::gainAdaptive::SetConstant(*this, docstring));

  // Command Set
  docstring = "    \n"
    "    set\n"
    "      Input:\n"
    "        floating point value: value at 0,\n"
    "        floating point value: value at infinity,\n"
    "        floating point value: value at slope,\n"
    "    \n";
  addCommand("set",
	     new command::gainAdaptive::SetConstant(*this, docstring));

}

GainAdaptive::
GainAdaptive( const std::string & name )
  :__SOT_GAIN_ADAPTATIVE_INIT
{
  sotDEBUG(15) << "New gain <"<<name<<">"<<std::endl;
  init();
  Entity::signalRegistration( gainSOUT<<errorSIN );
  addCommands();
}


GainAdaptive::
GainAdaptive( const std::string & name,const double& lambda )
  :__SOT_GAIN_ADAPTATIVE_INIT
{
  init(lambda);
  Entity::signalRegistration( gainSOUT );
  addCommands();
}

GainAdaptive::
GainAdaptive( const std::string & name,
		   const double& valueAt0,
		   const double& valueAtInfty,
		   const double& tanAt0 )
  :__SOT_GAIN_ADAPTATIVE_INIT
{
  init(valueAt0,valueAtInfty,tanAt0);
  Entity::signalRegistration( gainSOUT );
  addCommands();
}


void GainAdaptive::
init( const double& valueAt0,
      const double& valueAtInfty,
      const double& tanAt0 )
{
  coeff_a = valueAt0 - valueAtInfty;
  if (0 == coeff_a)    { coeff_b = 0; }
  else { coeff_b = tanAt0 / coeff_a;  }
  coeff_c = valueAtInfty;

  return;
}

void GainAdaptive::
forceConstant( void )
{
  coeff_a = 0;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void GainAdaptive::
display( std::ostream& os ) const
{
  os << "Gain Adaptative "<<getName();
  try{ os <<" = "<<double(gainSOUT); } catch (ExceptionSignal e) {}
  os <<" ("<<coeff_a<<";"<<coeff_b<<";"<<coeff_c<<") ";
}

#include <sot-core/exception-task.h>
void GainAdaptive::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine == "set" )
    {
      double c0(ZERO_DEFAULT);
	  double cinf(INFTY_DEFAULT);
	  double p0(TAN_DEFAULT);
      cmdArgs >> c0>>cinf>>p0;
      init(c0,cinf,p0);
    }
  else if( cmdLine == "setConstant")
    {
      double c(ZERO_DEFAULT); cmdArgs>>c;
      init(c);
    }
  else if( cmdLine == "forceConstant") forceConstant();
  else if( cmdLine == "help" )
    {
      os << "GainAdaptive: \n"
	 << "\t- set gain_0=%f gain_inf=%f tan_0=%f\t<Set the gain parameters>\n"
	 << "\t- setConstant gain=%f\t\t\t<Set the adaptative gain to a constant value>\n"
	 << "\t- forceConstant\t\t\t\t<Set the gain to a constant value equals to gain_inf>"
	 << std::endl;
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else Entity::commandLine( cmdLine,cmdArgs,os );

}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
double& GainAdaptive::
computeGain( double&res, int t )
{
  sotDEBUGIN(15);
  const ml::Vector& error = errorSIN(t);
  const double norm =  error.norm();
  res = coeff_a * exp( -coeff_b*norm ) + coeff_c;

  sotDEBUGOUT(15);
  return res;
}
