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

#ifndef __SOT_FIRFILTER_IMPL_HH__
#define __SOT_FIRFILTER_IMPL_HH__

#include <sot/core/fir-filter.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (fir_filter_EXPORTS)
#    define FIL_FILTER_EXPORT __declspec(dllexport)
#  else  
#    define FIL_FILTER_EXPORT  __declspec(dllimport)
#  endif 
#else
#  define FIL_FILTER_EXPORT 
#endif

# ifdef WIN32
#  define DECLARE_SPECIFICATION(className, sotSigType,sotCoefType ) \
    class FIL_FILTER_EXPORT className : public FIRFilter<sotSigType,sotCoefType>   \
    {                                                                              \
    public:                                                                        \
      className( const std::string& name );                                        \
    };
# else
#  define DECLARE_SPECIFICATION(className, sotSigType,sotCoefType ) \
	typedef FIRFilter<sotSigType,sotCoefType> className;
# endif // WIN32

DECLARE_SPECIFICATION(FIRFilterDoubleDouble,double,double)
DECLARE_SPECIFICATION(FIRFilterVectorDouble,ml::Vector,double)
DECLARE_SPECIFICATION(FIRFilterVectorMatrix,ml::Vector,ml::Matrix)

#endif
