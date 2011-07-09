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

#ifndef __SOT_INTEGRATOR_EULER_IMPL_H__
#define __SOT_INTEGRATOR_EULER_IMPL_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/integrator-euler.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (integrator_euler_EXPORTS)
#    define INTEGRATOR_EULER_EXPORT __declspec(dllexport)
#  else  
#    define INTEGRATOR_EULER_EXPORT  __declspec(dllimport)
#  endif 
#else
#  define INTEGRATOR_EULER_EXPORT 
#endif

# ifdef WIN32
#  define DECLARE_SPECIFICATION(className, sotSigType,sotCoefType ) \
    class INTEGRATOR_EULER_EXPORT className : public IntegratorEuler<sotSigType,sotCoefType>   \
    {                                                                              \
    public:                                                                        \
      std::string getTypeName( void );                                             \
      className( const std::string& name );                                        \
    };
# else
#  define DECLARE_SPECIFICATION(className, sotSigType,sotCoefType ) \
	typedef IntegratorEuler<sotSigType,sotCoefType> className;
# endif // WIN32

namespace dynamicgraph { 
  namespace sot {
	DECLARE_SPECIFICATION(IntegratorEulerVectorMatrix,ml::Vector,ml::Matrix)
  }
}

#endif
