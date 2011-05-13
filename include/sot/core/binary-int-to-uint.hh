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

#ifndef __SOT_BINARY_INT_TO_UINT_HH__
#define __SOT_BINARY_INT_TO_UINT_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <dynamic-graph/entity.h>
#include <sot/core/exception-task.hh>
#include <dynamic-graph/all-signals.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (binary_int_to_uint_EXPORTS)
#    define SOTBINARYINTTOUINT_EXPORT __declspec(dllexport)
#  else  
#    define SOTBINARYINTTOUINT_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTBINARYINTTOUINT_EXPORT
#endif

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

class SOTBINARYINTTOUINT_EXPORT BinaryIntToUint
  : public dg::Entity
{
public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  /* --- SIGNALS ------------------------------------------------------------ */
public:

  dg::SignalPtr< int,int > binaryIntSIN;
  dg::SignalTimeDependent< unsigned,int > binaryUintSOUT;

public:
  BinaryIntToUint( const std::string& name );
  virtual ~BinaryIntToUint() {}

  virtual unsigned& computeOutput( unsigned& res,int time );

  virtual void display( std::ostream& os ) const;
  void commandLine( const std::string& cmdLine,
		    std::istringstream& cmdArgs,
		    std::ostream& os );
};


} /* namespace sot */} /* namespace dynamicgraph */


#endif
