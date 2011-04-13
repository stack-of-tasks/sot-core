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

#ifndef __SOT_FEATURE_POINT6DRELATIVE_HH__
#define __SOT_FEATURE_POINT6DRELATIVE_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/feature-abstract.hh>
#include <sot/core/feature-point6d.hh>
#include <sot/core/exception-task.hh>
#include <sot/core/vector-utheta.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_point6d_relative_EXPORTS)
#    define SOTFEATUREPOINT6DRELATIVE_EXPORT __declspec(dllexport)
#  else
#    define SOTFEATUREPOINT6DRELATIVE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFEATUREPOINT6DRELATIVE_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
  \class FeaturePoint6dRelative
  \brief Class that defines the motion of a point of the body wrt. another
  point.
*/
class SOTFEATUREPOINT6DRELATIVE_EXPORT FeaturePoint6dRelative
: public FeaturePoint6d
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  ml::Matrix L;



  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalPtr< MatrixHomogeneous,int > positionReferenceSIN;
  dg::SignalPtr< ml::Matrix,int > articularJacobianReferenceSIN;

  /*! dg::Signals related to the computation of the derivative of
    the error
  @{ */

  /*! dg::Signals giving the derivative of the input signals.
    @{*/
  /*! Derivative of the relative position. */
  dg::SignalPtr< MatrixHomogeneous,int > dotpositionSIN;
  /*! Derivative of the reference position. */
  dg::SignalPtr< MatrixHomogeneous,int > dotpositionReferenceSIN;
  /*! @} */
  /*! The derivative of the error.*/
  dg::SignalTimeDependent<ml::Vector,int> errordotSOUT;
  /*! @} */

 public:
  FeaturePoint6dRelative( const std::string& name );
  virtual ~FeaturePoint6dRelative( void ) {}

  virtual ml::Vector& computeError( ml::Vector& res,int time );
  virtual ml::Vector& computeErrorDot( ml::Vector& res,int time );
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );
  virtual ml::Vector& computeActivation( ml::Vector& res,int time );

  virtual void display( std::ostream& os ) const;
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
  void initCommands( void );
  void initSdes( const std::string& featureDesiredName );

} ;

} /* namespace sot */} /* namespace dynamicgraph */

#endif // #ifndef __SOT_FEATURE_POINT6DRELATIVE_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
