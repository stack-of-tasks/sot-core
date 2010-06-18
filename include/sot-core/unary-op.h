/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      unary-op.h
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



#ifndef __SOT_BINARYOP_H__
#define __SOT_BINARYOP_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
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

namespace sot {

template< class Tin,class Tout,typename Operator >
class sotUnaryOp
:public Entity
{
  Operator op;

 public: /* --- CONSTRUCTION --- */

  static std::string getTypeInName( void ) { return "UnknownIn"; }
  static std::string getTypeOutName( void ) { return "UnknownOut"; }
  static const std::string CLASS_NAME;

  sotUnaryOp( const std::string& name )
    : Entity(name)
    ,SIN(NULL,sotUnaryOp::CLASS_NAME+"("+name+")::input("+getTypeInName()+")::in") 
    ,SOUT( boost::bind(&sotUnaryOp<Tin,Tout,Operator>::computeOperation,this,_1,_2), 
	   SIN,CLASS_NAME+"("+name+")::output("+getTypeOutName()+")::out") 
    {
      signalRegistration( SIN<<SOUT );
    }


  virtual ~sotUnaryOp( void ) {};

 public: /* --- SIGNAL --- */

  SignalPtr<Tin,int> SIN;
  SignalTimeDependant<Tout,int> SOUT;

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

} // namespace sot



#endif // #ifndef __SOT_BINARYOP_H__
