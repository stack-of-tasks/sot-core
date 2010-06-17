/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotBinaryOp.h
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
#include <sot-core/sotFlags.h>
#include <dynamic-graph/entity.h>
#include <sot-core/pool.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/sotVectorQuaternion.h>

/* STD */
#include <string>

#include <boost/function.hpp>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

template< class Tin1,class Tin2,class Tout,typename Operator >
class sotBinaryOp
:public Entity
{
  Operator op;

 public: /* --- CONSTRUCTION --- */

  static std::string getTypeIn1Name( void ) { return "UnknownIn1"; }
  static std::string getTypeIn2Name( void ) { return "UnknownIn2"; }
  static std::string getTypeOutName( void ) { return "UnknownOut"; }
  static const std::string CLASS_NAME;

  sotBinaryOp( const std::string& name )
    : Entity(name)
    ,SIN1(NULL,sotBinaryOp::CLASS_NAME+"("+name+")::input("+getTypeIn1Name()+")::in1") 
    ,SIN2(NULL,CLASS_NAME+"("+name+")::input("+getTypeIn2Name()+")::in2") 
    ,SOUT( boost::bind(&sotBinaryOp<Tin1,Tin2,Tout,Operator>::computeOperation,this,_1,_2), 
	   SIN1<<SIN2,CLASS_NAME+"("+name+")::output("+getTypeOutName()+")::out") 
    {
      signalRegistration( SIN1<<SIN2<<SOUT );
    }


  virtual ~sotBinaryOp( void ) {};

 public: /* --- SIGNAL --- */

  SignalPtr<Tin1,int> SIN1;
  SignalPtr<Tin2,int> SIN2;
  SignalTimeDependant<Tout,int> SOUT;

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





#endif // #ifndef __SOT_BINARYOP_H__
