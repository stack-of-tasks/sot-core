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

#ifndef __SOT_BINARYOP_H__
#define __SOT_BINARYOP_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <sot-core/flags.h>
#include <dynamic-graph/entity.h>
#include <sot-core/pool.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/all-signals.h>

/* STD */
#include <string>

#include <boost/function.hpp>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

template< class Tin,class Tout,typename Operator >
class UnaryOp
:public dg::Entity
{
  Operator op;

 public: /* --- CONSTRUCTION --- */

  static std::string getTypeInName( void ) { return "UnknownIn"; }
  static std::string getTypeOutName( void ) { return "UnknownOut"; }
  static const std::string CLASS_NAME;

  UnaryOp( const std::string& name )
    : dg::Entity(name)
    ,SIN(NULL,UnaryOp::CLASS_NAME+"("+name+")::input("+getTypeInName()+")::sin")
    ,SOUT( boost::bind(&UnaryOp<Tin,Tout,Operator>::computeOperation,this,_1,_2),
	   SIN,CLASS_NAME+"("+name+")::output("+getTypeOutName()+")::sout")
    {
      signalRegistration( SIN<<SOUT );
    }


  virtual ~UnaryOp( void ) {};

 public: /* --- SIGNAL --- */

  dg::SignalPtr<Tin,int> SIN;
  dg::SignalTimeDependent<Tout,int> SOUT;

 protected:
  Tout& computeOperation( Tout& res,int time )
    {
      const Tin &x1 = SIN(time);
      op(x1,res);
      return res;
    }

 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
 			    std::ostream& os ) ;


};

} /* namespace sot */} /* namespace dynamicgraph */



#endif // #ifndef __SOT_BINARYOP_H__
