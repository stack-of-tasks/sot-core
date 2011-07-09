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

#ifndef __SOT_INTEGRATOR_ABSTRACT_VECTOR_H__
#define __SOT_INTEGRATOR_ABSTRACT_VECTOR_H__

/* --- SOT PLUGIN  --- */
#include <sot/core/integrator-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (integrator_abstract_EXPORTS)
#    define INTEGRATOR_ABSTRACT_EXPORT __declspec(dllexport)
#  else  
#    define INTEGRATOR_ABSTRACT_EXPORT  __declspec(dllimport)
#  endif 
#else
#  define INTEGRATOR_ABSTRACT_EXPORT 
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#ifdef WIN32
# define DECLARE_SPECIFICATION(className, sotSigType, sotCoefType)\
   class INTEGRATOR_ABSTRACT_EXPORT className :                   \
	public IntegratorAbstract<sotSigType, sotCoefType>            \
   {                                                              \
 public:                                                          \
   className( const std::string& name );                          \
   };
#else
# define DECLARE_SPECIFICATION(className, sotSigType, sotCoefType) \
   typedef IntegratorAbstract<sotSigType,sotCoefType> className;
#endif

namespace dynamicgraph { namespace sot {
	DECLARE_SPECIFICATION(IntegratorAbstractDouble,double,double)
	DECLARE_SPECIFICATION(IntegratorAbstractVector,ml::Vector,ml::Matrix)
}
}
#endif // #ifndef  __SOT_MAILBOX_HH





