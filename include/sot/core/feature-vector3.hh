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

#ifndef __SOT_FEATURE_VECTOR3_HH__
#define __SOT_FEATURE_VECTOR3_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/feature-abstract.hh>
#include <sot/core/exception-task.hh>
#include <sot/core/matrix-geometry.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_vector3_EXPORTS)
#    define SOTFEATUREVECTOR3_EXPORT __declspec(dllexport)
#  else
#    define SOTFEATUREVECTOR3_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFEATUREVECTOR3_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
  \class FeatureVector3
  \brief Class that defines point-3d control feature
*/
class SOTFEATUREVECTOR3_EXPORT FeatureVector3
: public FeatureAbstract
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  DECLARE_NO_REFERENCE;

 protected:

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalPtr< dg::Vector,int > vectorSIN;
  dg::SignalPtr< MatrixHomogeneous,int > positionSIN;
  dg::SignalPtr< dg::Matrix,int > articularJacobianSIN;
  dg::SignalPtr< dg::Vector,int > positionRefSIN;

  using FeatureAbstract::selectionSIN;
  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::errorSOUT;

 public:
  FeatureVector3( const std::string& name );
  virtual ~FeatureVector3( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );

  virtual dg::Vector& computeError( dg::Vector& res,int time );
  virtual dg::Matrix& computeJacobian( dg::Matrix& res,int time );

  virtual void display( std::ostream& os ) const;

} ;

} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_FEATURE_VECTOR3_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
