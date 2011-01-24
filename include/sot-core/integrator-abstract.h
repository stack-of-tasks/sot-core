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

#ifndef __SOT_INTEGRATOR_ABSTRACT_H__
#define __SOT_INTEGRATOR_ABSTRACT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <sot-core/flags.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/debug.h>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

/*! \brief integrates an ODE. If Y is the output and X the input, the
 * following equation is integrated:
 * a_p * d(p)Y / dt^p + .... + a_0 Y = b_m * d(m)X / dt^m + ... . b_0 X
 * a_i are the coefficients of the denominator of the associated transfer
 * function between X and Y, while the b_i are those of the numerator.
*/
template<class sigT, class coefT>
class IntegratorAbstract
:public dg::Entity
{
 public:
  virtual const std::string& getClassName() const { return dg::Entity::getClassName(); }
  static std::string getTypeName( void ) { return "Unknown"; }
  static const std::string CLASS_NAME;

 public:
  IntegratorAbstract ( const std::string& name )
    :dg::Entity(name)
     ,SIN(NULL,"sotIntegratorAbstract("+name+")::input(vector)::in")
     ,SOUT(boost::bind(&IntegratorAbstract<sigT,coefT>::integrate,this,_1,_2),
		 SIN,
		 "sotIntegratorAbstract("+name+")::output(vector)::out")
  {
    signalRegistration( SIN<<SOUT );
  }

  virtual ~IntegratorAbstract() {}

  virtual sigT& integrate( sigT& res,int time ) = 0;

 public:  /* --- INHERITANCE --- */
  virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,std::ostream& os )
  {
    if( cmdLine == "pushNumCoef" )
    {
      std::string objname, signame;
      // Florent: remove reference to g_shell
      //dg::Interpreter::objectNameParser(cmdArgs, objname, signame);
      dg::Entity& obj = dg::g_pool.getEntity(objname);
      dg::SignalBase<int>& sig = obj.getSignal(signame);
      try {
	dg::Signal<coefT,int>& sigc = dynamic_cast<dg::Signal<coefT,int>&>(sig);
	pushNumCoef(sigc.accessCopy());
      }
      catch(std::bad_cast& bc) {
	os << "Command ignored: bad_cast exception...";
      }
      cmdArgs >> std::ws;
    }
    else if( cmdLine == "pushDenomCoef" )
    {
      std::string objname, signame;
      // Florent: remove reference to g_shell
      //dg::Interpreter::objectNameParser(cmdArgs, objname, signame);
      dg::Entity& obj = dg::g_pool.getEntity(objname);
      dg::SignalBase<int>& sig = obj.getSignal(signame);
      try {
	dg::Signal<coefT,int>& sigc = dynamic_cast<dg::Signal<coefT,int>&>(sig);
	pushDenomCoef(sigc.accessCopy());
      }
      catch(std::bad_cast& bc) {
	os << "Command ignored: bad_cast exception...";
      }
      cmdArgs >> std::ws;
    }
    else if( cmdLine == "popNumCoef" )
    {
      popNumCoef();
    }
    else if( cmdLine == "popDenomCoef" )
    {
      popDenomCoef();
    }
    else 
    {
    	dg::Entity::commandLine(cmdLine, cmdArgs, os);
    }
  }

 public:
  void pushNumCoef(const coefT& numCoef) { numerator.push_back(numCoef); }
  void pushDenomCoef(const coefT& denomCoef) { denominator.push_back(denomCoef); }
  void popNumCoef() { numerator.pop_back(); }
  void popDenomCoef() { denominator.pop_back(); }

 public:
  dg::SignalPtr<sigT, int> SIN;

  dg::SignalTimeDependent<sigT, int> SOUT;

 protected:
  std::vector<coefT> numerator;
  std::vector<coefT> denominator;
};


} // namespace sot




#endif
