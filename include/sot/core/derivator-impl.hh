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

#ifndef __SOT_DERIVATOR_IMPL_H__
#define __SOT_DERIVATOR_IMPL_H__

#include <sot/core/derivator.hh>
#include <sot/core/vector-quaternion.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (derivator_EXPORTS)
#    define DERIVATOR_EXPORT __declspec(dllexport)
#  else  
#    define DERIVATOR_EXPORT  __declspec(dllimport)
#  endif 
#else
#  define DERIVATOR_EXPORT 
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

#ifdef WIN32
# define DECLARE_SPECIFICATION(className, sotSigType )            \
   class DERIVATOR_EXPORT className : public Derivator<sotSigType>  \
   {                                                                \
 public:                                                           \
   className( const std::string& name );                            \
   };
#else
# define DECLARE_SPECIFICATION(className, sotSigType) \
   typedef Derivator<sotSigType> className;
#endif

DECLARE_SPECIFICATION(DerivatorDouble,double)
DECLARE_SPECIFICATION(DerivatorVector,ml::Vector)
DECLARE_SPECIFICATION(DerivatorMatrix,ml::Matrix)
DECLARE_SPECIFICATION(DerivatorVectorQuaternion,VectorQuaternion)
} /* namespace sot */} /* namespace dynamicgraph */



#endif // #ifndef __SOT_DERIVATOR_H__
