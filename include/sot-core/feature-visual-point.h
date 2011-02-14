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

#ifndef __SOT_FEATURE_VISUALPOINT_HH__
#define __SOT_FEATURE_VISUALPOINT_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-core/feature-abstract.h>
#include <sot-core/exception-task.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_visual_point_EXPORTS)
#    define SOTFEATUREVISUALPOINT_EXPORT __declspec(dllexport)
#  else
#    define SOTFEATUREVISUALPOINT_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFEATUREVISUALPOINT_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
  \class FeatureVisualPoint
  \brief Class that defines 2D visualPoint visual feature
*/
class SOTFEATUREVISUALPOINT_EXPORT FeatureVisualPoint
: public FeatureAbstract
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  ml::Matrix L;



  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalPtr< ml::Vector,int > xySIN;
  /** FeatureVisualPoint depth (required to compute the interaction matrix)
   * default Z = 1m. */
  dg::SignalPtr< double,int > ZSIN;
  dg::SignalPtr< ml::Matrix,int > articularJacobianSIN;

  using FeatureAbstract::desiredValueSIN;
  using FeatureAbstract::selectionSIN;

  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::errorSOUT;
  using FeatureAbstract::activationSOUT;

 public:
  FeatureVisualPoint( const std::string& name );
  virtual ~FeatureVisualPoint( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );

  virtual ml::Vector& computeError( ml::Vector& res,int time );
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );
  virtual ml::Vector& computeActivation( ml::Vector& res,int time );

  /** Static Feature selection. */
  inline static Flags selectX( void ) { return FLAG_LINE_1; }
  inline static Flags selectY( void ) { return FLAG_LINE_2; }

  virtual void display( std::ostream& os ) const;


} ;

} /* namespace sot */} /* namespace dynamicgraph */

#endif // #ifndef __SOT_FEATURE_VISUALPOINT_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
