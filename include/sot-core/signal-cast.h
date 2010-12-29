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

#ifndef __SOT_SIGNAL_CAST_HH__
#define __SOT_SIGNAL_CAST_HH__


#include <sot-core/flags.h>
#include <sot-core/multi-bound.h>
#include <sot-core/sot-core-api.h>
#ifdef WIN32
#include <sot-core/utils-windows.h>
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
  dynamicgraph::SignalCastRegisterer sotCastRegisterer_##TYPE	\
    (typeid(TYPE),						\
     SignalCast<TYPE>::disp_,					\
     SignalCast<TYPE>::cast_,					\
     SignalCast<TYPE>::trace_)

#define SOT_SIGNAL_CAST_DECLARATION_NAMED(TYPE,NAME)			\
  dynamicgraph::SignalCastRegisterer sotCastRegisterer_##NAME		\
    (typeid(TYPE),							\
     SignalCast<TYPE>::disp_,						\
     SignalCast<TYPE>::cast_,						\
     SignalCast<TYPE>::trace_)

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

template<>
class SignalCast<FeatureAbstract*>
{
public:
    SOT_CORE_EXPORT static FeatureAbstract*
      cast( std::istringstream& iss );
    SOT_CORE_EXPORT static void
    disp( FeatureAbstract* t,std::ostream& os );
    SOT_CORE_EXPORT static void
    trace( FeatureAbstract* t, std::ostream& os ) {
      disp(t,os);
    }
public:
  static boost::any cast_( std::istringstream& stringValue ) {
    return boost::any_cast<FeatureAbstract*>(cast(stringValue));
  }
  static void disp_( const boost::any& t,std::ostream& os )  {
    disp(boost::any_cast<FeatureAbstract*>(t), os);
  }
  static void trace_( const boost::any& t,std::ostream& os ) {
    trace(boost::any_cast<FeatureAbstract*>(t),os);
  }
}

  template<>
  class SignalCast<struct timeval>
  {
  public:
    SOT_CORE_EXPORT static struct timeval cast( std::istringstream& iss );
    SOT_CORE_EXPORT static void disp( const struct timeval& t,
				      std::ostream& os );
    SOT_CORE_EXPORT static void trace( const struct timeval& t,
				       std::ostream& os ) { disp(t,os); }
  public:
    static boost::any cast_( std::istringstream& stringValue ) {
      return boost::any_cast<struct timeval>(cast(stringValue));
    }
    static void disp_( const boost::any& t,std::ostream& os )  {
      disp(boost::any_cast<struct timeval>(t), os);
    }
    static void trace_( const boost::any& t,std::ostream& os ) {
      trace(boost::any_cast<struct timeval>(t),os);
    }
  };
} // namespace sot


#endif // #ifndef __SOT_SIGNAL_CAST_HH__



