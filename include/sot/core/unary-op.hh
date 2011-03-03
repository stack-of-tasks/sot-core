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

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
  namespace sot {

    template< typename Operator >
    class UnaryOp
      :public Entity
    {
      Operator op;
      typedef typename Operator::Tin Tin;
      typedef typename Operator::Tout Tout;
      typedef UnaryOp<Operator> Self;

    public: /* --- CONSTRUCTION --- */

      static std::string getTypeInName( void ) { return Operator::nameTypeIn(); }
      static std::string getTypeOutName( void ) { return Operator::nameTypeOut(); }
      static const std::string CLASS_NAME;

      virtual const std::string& getClassName  () const
      {
	return CLASS_NAME;
      }

      UnaryOp( const std::string& name )
	: Entity(name)
	,SIN(NULL,Self::CLASS_NAME+"("+name+")::input("+Self::getTypeInName()+")::sin")
	,SOUT( boost::bind(&Self::computeOperation,this,_1,_2),
	       SIN,Self::CLASS_NAME+"("+name+")::output("+Self::getTypeOutName()+")::sout")
      {
	signalRegistration( SIN<<SOUT );
	op.addSpecificCommands(*this,commandMap);
      }

      virtual ~UnaryOp( void ) {};

    public: /* --- SIGNAL --- */

      SignalPtr<Tin,int> SIN;
      SignalTimeDependent<Tout,int> SOUT;

    protected:
      Tout& computeOperation( Tout& res,int time )
      {
	const Tin &x1 = SIN(time);
	op(x1,res);
	return res;
      }

    public: /* --- PARAMS --- */

    };
  } /* namespace sot */
} /* namespace dynamicgraph */

#endif // #ifndef __SOT_BINARYOP_H__
