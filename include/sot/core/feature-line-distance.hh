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

#ifndef __SOT_FEATURE_LINEDISTANCE_HH__
#define __SOT_FEATURE_LINEDISTANCE_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/feature-abstract.hh>
#include <sot/core/exception-task.hh>
#include <sot/core/matrix-homogeneous.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_line_distance_EXPORTS)
#    define SOTFEATURELINEDISTANCE_EXPORT __declspec(dllexport)
#  else
#    define SOTFEATURELINEDISTANCE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFEATURELINEDISTANCE_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
  \class FeatureLineDistance
  \brief Class that defines point-3d control feature
*/
class SOTFEATURELINEDISTANCE_EXPORT FeatureLineDistance
: public FeatureAbstract
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalPtr< MatrixHomogeneous,int > positionSIN;
  dg::SignalPtr< ml::Matrix,int > articularJacobianSIN;
  dg::SignalPtr< ml::Vector,int > positionRefSIN;
  dg::SignalPtr< ml::Vector,int > vectorSIN;
  dg::SignalTimeDependent<ml::Vector,int> lineSOUT;

  using FeatureAbstract::selectionSIN;
  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::errorSOUT;

  /*! \name Dealing with the reference value to be reach with this feature.
    @{
  */
  DECLARE_NO_REFERENCE;
  /*! @} */


 public:
  FeatureLineDistance( const std::string& name );
  virtual ~FeatureLineDistance( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );

  virtual ml::Vector& computeError( ml::Vector& res,int time );
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );
  ml::Vector& computeLineCoordinates( ml::Vector& cood,int time );

  virtual void display( std::ostream& os ) const;

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

} ;


} /* namespace sot */} /* namespace dynamicgraph */

#endif // #ifndef __SOT_FEATURE_LINEDISTANCE_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
