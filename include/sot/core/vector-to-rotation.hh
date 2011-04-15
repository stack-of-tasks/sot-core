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

#ifndef __SOTVECTORTOMATRIX_HH
#define __SOTVECTORTOMATRIX_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/all-signals.h>
#include <sot/core/matrix-rotation.hh>

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* STD */
#include <vector>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (vector_to_rotation_EXPORTS)
#    define SOTVECTORTOROTATION_EXPORT __declspec(dllexport)
#  else
#    define SOTVECTORTOROTATION_EXPORT  __declspec(dllimport)
#  endif
#else
#  define SOTVECTORTOROTATION_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

class SOTVECTORTOROTATION_EXPORT VectorToRotation
: public dg::Entity
{
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  enum sotAxis
    {
      AXIS_X
      ,AXIS_Y
      ,AXIS_Z
    };

  unsigned int size;
  std::vector< sotAxis > axes;


public:
  VectorToRotation( const std::string& name );

  virtual ~VectorToRotation( void ){}

  dg::SignalPtr<ml::Vector,int> SIN;
  dg::SignalTimeDependent<MatrixRotation,int> SOUT;

  MatrixRotation& computeRotation( const ml::Vector& angles,
				      MatrixRotation& res );


  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

};

} /* namespace sot */} /* namespace dynamicgraph */



#endif // #ifndef __SOTVECTORTOMATRIX_HH


