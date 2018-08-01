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
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <sot/core/flags.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/command-bind.h>
#include <sot/core/debug.hh>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
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
  IntegratorAbstract ( const std::string& name )
    :dg::Entity(name)
     ,SIN(NULL,"sotIntegratorAbstract("+name+")::input(vector)::sin")
     ,SOUT(boost::bind(&IntegratorAbstract<sigT,coefT>::integrate,this,_1,_2),
		 SIN,
		 "sotIntegratorAbstract("+name+")::output(vector)::sout")
  {
    signalRegistration( SIN<<SOUT );

    using namespace dg::command;

    const std::string typeName =
      Value::typeName(dg::command::ValueHelper<coefT>::TypeID);

    addCommand ("pushNumCoef",
        makeCommandVoid1 (*this, &IntegratorAbstract::pushNumCoef,
          docCommandVoid1 ("Push a new numerator coefficient", typeName)
          ));
    addCommand ("pushDenomCoef",
        makeCommandVoid1 (*this, &IntegratorAbstract::pushDenomCoef,
          docCommandVoid1 ("Push a new denomicator coefficient", typeName)
          ));

    addCommand ("popNumCoef",
        makeCommandVoid0 (*this, &IntegratorAbstract::popNumCoef,
          docCommandVoid0 ("Pop a new numerator coefficient")
          ));
    addCommand ("popDenomCoef",
        makeCommandVoid0 (*this, &IntegratorAbstract::popDenomCoef,
          docCommandVoid0 ("Pop a new denomicator coefficient")
          ));
  }

  virtual ~IntegratorAbstract() {}

  virtual sigT& integrate( sigT& res,int time ) = 0;

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


} /* namespace sot */} /* namespace dynamicgraph */




#endif
