/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      SignalCast.h
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



#ifndef __SOT_SIGNAL_CAST_HH__
#define __SOT_SIGNAL_CAST_HH__


#include <sot-core/flags.h>
#include <MatrixAbstractLayer/boost.h>
#include <sot-core/matrix-homogeneous.h>
#include <sot-core/matrix-twist.h>
#include <sot-core/vector-utheta.h>
#include <sot-core/vector-quaternion.h>
#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot-core/matrix-force.h>
#include <sot-core/multi-bound.h>
#include <sot-core/sot-core-api.h>
#ifdef WIN32
#include <sot-core/sotUtilsWindows.h>
#endif
#include <dynamic-graph/signal-caster.h>

/* -------------------------------------------------------------------------- */
/* --- CLASS ---------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

namespace sot {


class FeatureAbstract;


/*!
 * This class is only used to group together static functions who differ by
 * a template parameter. It is never actually instanced (the private constructor
 * makes sure of that).
 */
template< class T >
class SignalCast
{
public:
  SOT_CORE_EXPORT static T cast( std::istringstream& stringValue ) { throw 1;}
  SOT_CORE_EXPORT static void disp( const T& t,std::ostream& os )  { throw 1;  }
  SOT_CORE_EXPORT static void trace( const T& t,std::ostream& os ) { disp(t,os); }
public:
  // adapter functions for SignalCast
  static boost::any cast_( std::istringstream& stringValue ) {
	  return boost::any_cast<T>(cast(stringValue));
  }
  static void disp_( const boost::any& t,std::ostream& os )  {
	  disp(boost::any_cast<T>(t), os);
  }
  static void trace_( const boost::any& t,std::ostream& os ) {
	  trace(boost::any_cast<T>(t),os);
  }
private:
  SignalCast() {}
};


/* -------------------------------------------------------------------------- */
/* --- MACROS --------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* Declaration macro: one instance of each class needs to be present in
 * order for casts to be registered.
 */

#define SOT_SIGNAL_CAST_DECLARATION(TYPE) \
		dynamicgraph::SignalCastRegisterer sotCastRegisterer_##TYPE \
				(typeid(TYPE), \
				SignalCast<TYPE>::disp_, \
				SignalCast<TYPE>::cast_, \
				SignalCast<TYPE>::trace_);

#define SOT_SIGNAL_CAST_DECLARATION_NAMED(TYPE,NAME) \
		dynamicgraph::SignalCastRegisterer sotCastRegisterer_##NAME \
				(typeid(TYPE), \
				SignalCast<TYPE>::disp_, \
				SignalCast<TYPE>::cast_, \
				SignalCast<TYPE>::trace_);

/* Standard definition macros: the three functions can be specified
 * in the macros. To define then in the cpp, just put ';' in the args.
 */
#define SOT_SIGNAL_CAST_FULL_DEFINITION(TYPE,CAST,DISP,TRACE)            \
template<>                                                               \
class SignalCast<TYPE>												\
{                                                                        \
public:                                                                  \
    SOT_CORE_EXPORT static TYPE cast( std::istringstream& iss )         CAST         \
    SOT_CORE_EXPORT static void disp( const TYPE& t,std::ostream& os )  DISP         \
    SOT_CORE_EXPORT static void trace( const TYPE& t,std::ostream& os ) TRACE        \
public:            																\
	static boost::any cast_( std::istringstream& stringValue ) {           		\
		  return boost::any_cast<TYPE>(cast(stringValue));           			\
	}            																\
	static void disp_( const boost::any& t,std::ostream& os )  {       			\
	  disp(boost::any_cast<TYPE>(t), os);         								\
	}            																\
	static void trace_( const boost::any& t,std::ostream& os ) {      			\
		  trace(boost::any_cast<TYPE>(t),os);      							    \
	}          																	\
}

/* Standard definition macros: the functions <cast> and <disp> have
 * to be implemented in the cpp files. The function <trace> is
 * implemented as a proxy on <disp>.
 */
#define SOT_SIGNAL_CAST_DEFINITION_HPP(TYPE)                             \
 SOT_SIGNAL_CAST_FULL_DEFINITION(TYPE,;,;,{ disp(t,os); })

/* Lazy definition: <cast> and <disp> are to proxys on the standard
 * std input (>>) and output (<<). The function <trace> has to be 
 * implemented in the cpp.
 */
#define SOT_SIGNAL_CAST_DEFINITION_TRACE_HPP(TYPE,TRACE)                 \
 SOT_SIGNAL_CAST_FULL_DEFINITION(TYPE,                                   \
 {TYPE res; iss >> res; return res; },                                   \
 { os << t <<std::endl; },                                               \
 TRACE )

/* Lazy lazy definition: the three functions are implemented as
 * proxys on std::io operation. 
 */
#define SOT_SIGNAL_CAST_DEFINITION(TYPE)                                 \
 SOT_SIGNAL_CAST_FULL_DEFINITION(TYPE,                                   \
 {TYPE res; iss >> res; return res; },                                   \
 { os << t <<std::endl; },                                               \
 { disp(t,os); })

/* Lazy definition of <cast> and <disp> with implementation of
 * <trace> in the cpp. 
 */
#define SOT_SIGNAL_CAST_DEFINITION_TRACE(TYPE)         \
 SOT_SIGNAL_CAST_FULL_DEFINITION(TYPE,                                   \
 {TYPE res; iss >> res; return res; },                                   \
 { os << t <<std::endl; },                                               \
 ;)

/* -------------------------------------------------------------------------- */
/* --- MACROS --------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* --- OTHER --- */
SOT_SIGNAL_CAST_DEFINITION(Flags);
SOT_SIGNAL_CAST_DEFINITION_TRACE(VectorMultiBound);

typedef FeatureAbstract* SignalCast_sotFeatureAbstractPtr  ;

SOT_SIGNAL_CAST_DEFINITION_HPP( SignalCast_sotFeatureAbstractPtr );
SOT_SIGNAL_CAST_DEFINITION_HPP( struct timeval );

/* --- Matrices and Vectors --- */
#define SOT_SIGNAL_CAST_DEFINITION_MATRIX(TYPE)         \
SOT_SIGNAL_CAST_FULL_DEFINITION(TYPE,{TYPE res; iss >> res; return res; }, \
                                { SignalCast<ml::Matrix>::disp(t,os); }, \
                                { SignalCast<ml::Matrix>::trace(t,os); })
//SOT_SIGNAL_CAST_DEFINITION_TRACE_HPP(TYPE,{ SignalCast<ml::Matrix>::trace(t,os); })

#define SOT_SIGNAL_CAST_DEFINITION_VECTOR(TYPE)         \
SOT_SIGNAL_CAST_FULL_DEFINITION(TYPE,{TYPE res; iss >> res; return res; }, \
                                { SignalCast<ml::Vector>::disp(t,os); }, \
                                { SignalCast<ml::Vector>::trace(t,os); })
//SOT_SIGNAL_CAST_DEFINITION_TRACE_HPP(TYPE,{ SignalCast<ml::Vector>::trace(t,os); })

SOT_SIGNAL_CAST_FULL_DEFINITION(maal::boost::Vector,; SOT_CORE_EXPORT static maal::boost::DisplayType displayType;,;,;);
SOT_SIGNAL_CAST_FULL_DEFINITION(maal::boost::Matrix,;,;,;);

/* All the followings are defined with proxys on the equivalent
 * functions ml:: based.
 */
SOT_SIGNAL_CAST_DEFINITION_VECTOR(VectorUTheta);
SOT_SIGNAL_CAST_DEFINITION_VECTOR(VectorQuaternion);
SOT_SIGNAL_CAST_DEFINITION_VECTOR(VectorRollPitchYaw);
SOT_SIGNAL_CAST_DEFINITION_MATRIX(MatrixRotation);
SOT_SIGNAL_CAST_DEFINITION_MATRIX(MatrixHomogeneous);
SOT_SIGNAL_CAST_DEFINITION_MATRIX(MatrixTwist);
SOT_SIGNAL_CAST_DEFINITION_MATRIX(MatrixForce);

} // namespace sot


#endif // #ifndef __SOT_SIGNAL_CAST_HH__



