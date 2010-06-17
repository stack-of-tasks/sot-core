/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotGainAdaptative.cpp
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


/* SOT */
#include <sot-core/sotGainAdaptative.h>


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot-core/sotDebug.h>
#include <sot-core/factory.h>
#include <sot-core/exception-signal.h>
SOT_FACTORY_ENTITY_PLUGIN(sotGainAdaptative,"GainAdaptative");

const double sotGainAdaptative::
ZERO_DEFAULT = .1;
const double sotGainAdaptative::
INFTY_DEFAULT = .1;
const double sotGainAdaptative::
TAN_DEFAULT = 1;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#define __SOT_GAIN_ADAPTATIVE_INIT \
Entity(name) \
,coeff_a(0) \
,coeff_b(0) \
,coeff_c(0) \
,errorSIN(NULL,"sotGainAdaptative("+name+")::input(vector)::error") \
,gainSOUT( boost::bind(&sotGainAdaptative::computeGain,this,_1,_2), \
	   errorSIN,"sotGainAdaptative("+name+")::output(double)::gain" )



sotGainAdaptative::
sotGainAdaptative( const std::string & name )
  :__SOT_GAIN_ADAPTATIVE_INIT
{
  sotDEBUG(15) << "New gain <"<<name<<">"<<std::endl;
  init();
  Entity::signalRegistration( gainSOUT<<errorSIN );
}


sotGainAdaptative::
sotGainAdaptative( const std::string & name,const double& lambda )
  :__SOT_GAIN_ADAPTATIVE_INIT
{
  init(lambda);
  Entity::signalRegistration( gainSOUT );
}

sotGainAdaptative::
sotGainAdaptative( const std::string & name,
		   const double& valueAt0,
		   const double& valueAtInfty,
		   const double& tanAt0 )
  :__SOT_GAIN_ADAPTATIVE_INIT
{
  init(valueAt0,valueAtInfty,tanAt0);
  Entity::signalRegistration( gainSOUT );
}


void sotGainAdaptative::
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

void sotGainAdaptative::
forceConstant( void )
{
  coeff_a = 0;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void sotGainAdaptative::
display( std::ostream& os ) const
{
  os << "Gain Adaptative "<<getName();
  try{ os <<" = "<<double(gainSOUT); } catch (sotExceptionSignal e) {}
  os <<" ("<<coeff_a<<";"<<coeff_b<<";"<<coeff_c<<") ";
}

#include <sot-core/exception-task.h>
void sotGainAdaptative::
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
      os << "GainAdaptative: \n"
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
double& sotGainAdaptative::
computeGain( double&res, int t )
{
  sotDEBUGIN(15);
  const ml::Vector& error = errorSIN(t);
  const double norm =  error.norm();
  res = coeff_a * exp( -coeff_b*norm ) + coeff_c;

  sotDEBUGOUT(15);
  return res;
}
