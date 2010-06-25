/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      binary-op.h
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
#include <sot-core/vector-quaternion.h>

/* STD */
#include <string>

#include <boost/function.hpp>

namespace sot {
namespace dg = dynamicgraph;
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

template< class Tin1,class Tin2,class Tout,typename Operator >
class BinaryOp
:public dg::Entity
{
  Operator op;

 public: /* --- CONSTRUCTION --- */

  static std::string getTypeIn1Name( void ) { return "UnknownIn1"; }
  static std::string getTypeIn2Name( void ) { return "UnknownIn2"; }
  static std::string getTypeOutName( void ) { return "UnknownOut"; }
  static const std::string CLASS_NAME;

  BinaryOp( const std::string& name )
    : dg::Entity(name)
    ,SIN1(NULL,BinaryOp::CLASS_NAME+"("+name+")::input("+getTypeIn1Name()+")::in1") 
    ,SIN2(NULL,CLASS_NAME+"("+name+")::input("+getTypeIn2Name()+")::in2") 
    ,SOUT( boost::bind(&BinaryOp<Tin1,Tin2,Tout,Operator>::computeOperation,this,_1,_2), 
	   SIN1<<SIN2,CLASS_NAME+"("+name+")::output("+getTypeOutName()+")::out") 
    {
      signalRegistration( SIN1<<SIN2<<SOUT );
    }


  virtual ~BinaryOp( void ) {};

 public: /* --- SIGNAL --- */

  dg::SignalPtr<Tin1,int> SIN1;
  dg::SignalPtr<Tin2,int> SIN2;
  dg::SignalTimeDependant<Tout,int> SOUT;

 protected:
  Tout& computeOperation( Tout& res,int time )
    {
      const Tin1 &x1 = SIN1(time);
      const Tin2 &x2 = SIN2(time);
      op(x1,x2,res);
      return res;
    }

 public: /* --- PARAMS --- */
   virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs, 
			     std::ostream& os );
    

};


} // namespace sot


#endif // #ifndef __SOT_BINARYOP_H__
