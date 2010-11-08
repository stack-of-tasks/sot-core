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

#ifndef __SOT_FEATURE_ABSTRACT_H__
#define __SOT_FEATURE_ABSTRACT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <sot-core/flags.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <sot-core/sot-core-api.h>

/* STD */
#include <string>

namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! \class FeatureAbstract
  \ingroup features
  \brief This class gives the abstract definition of a feature.

  A feature is a data evolving according to time.
  It is defined by a vector \f${\bf s}(t) \in \mathbb{R}^n \f$ where \f$ t \f$ is the time.
  By default a feature has a desired \f${\bf s}^*(t) \f$.
  The feature is in charge of collecting its own current state.
  A feature is supposed to compute an error between its current state and the desired one:
  \f$ e(t) = {\bf s}^*(t) - {\bf s}(t) \f$.
  A feature is supposed to compute a Jacobian according to the robot state vector
  \f$ \frac{\delta {\bf s}(t)}{\delta {\bf q}(t)}\f$.
 */
class SOT_CORE_EXPORT FeatureAbstract
:public dg::Entity
{
 public:
  /*! \brief Store the name of the class. */
  static const std::string CLASS_NAME;

  /*! \brief Returns the name class. */
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  /*! \brief Register the feature in the stack of tasks. */
  void featureRegistration( void );

 public:
  /*! \brief Default constructor: the name of the class should be given. */
  FeatureAbstract( const std::string& name );
  /*! \brief Default destructor */
  virtual ~FeatureAbstract( void ) {};

  /*! \name Methods returning the dimension of the feature.
    @{ */

  /*! \brief Verbose method.
    \par res: The integer in which the dimension will be return.
    \par time: The time at which the feature should be considered.
    \return Dimension of the feature.
    \note Be careful with features changing their dimension according to time.
   */
  virtual unsigned int& getDimension( unsigned int& res,int time ) = 0;

  /*! \brief Short method
    \par time: The time at which the feature should be considered.
    \return Dimension of the feature.
    \note Be careful with features changing their dimension according to time.
   */
  inline unsigned int getDimension( int time )
    {unsigned int res; getDimension(res,time); return res;}

  /*! \brief Shortest method
    \return Dimension of the feature.
    \note The feature is not changing its dimension according to time.
   */
  inline unsigned int getDimension( void ) const
    { return dimensionSOUT; }
  /*! @} */

  /*! \name Methods to control internal computation.
    The main idea is that some feature may have a lower frequency
    than the internal control loop. In this case, the methods for
    computation are called only when needed.

   @{*/

  /*! \brief Compute the error between the desired feature and
   the current value of the feature measured or deduced from the robot state.

   \par[out] res: The error will be set into res.
   \par[in] time: The time at which the error is computed.
   \return The vector res with the appropriate value.
  */
  virtual ml::Vector& computeError( ml::Vector& res,int time ) = 0;

  /*! \brief Compute the Jacobian of the error according the robot state.

    \par[out] res: The matrix in which the error will be written.
    \par[in] time: The time at which the Jacobian is computed \f$ {\bf J}(q(t)) \f$.
    \return The matrix res with the appropriate values.
   */
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time ) = 0;

  /*! \brief Reevaluate the current value of the feature
    according to external measurement provided through a mailbox,
    or deduced from the estimated state of the robot at the time specified.

    \par[out] res: The vector in which the value will be written.
    \par[in] time: The time at which the feature is evaluated \f$ {\bf s}(t)) \f$.
    \return The vector res with the appropriate values.
   */
  virtual ml::Vector& computeActivation( ml::Vector& res,int time ) = 0;

  /*! @} */

  /* --- SIGNALS ------------------------------------------------------------ */
 public:

  /*! \name Signals
   @{
  */

  /*! \name Input signals:
   @{ */
  /*! \brief This signal specifies the desired value \f$ {\bf s}^*(t) \f$ */
  dg::SignalPtr< FeatureAbstract*,int > desiredValueSIN;

  /*! \brief This vector specifies which dimension are used to perform the computation.
   For instance let us assume that the feature is a 3D point. If only the Y-axis should
   be used for computing error, activation and Jacobian, then the vector to specify
   is \f$ [ 0 1 0] \f$.*/
  dg::SignalPtr< Flags,int > selectionSIN;
  /*! @} */

  /*! \name Output signals:
   @{ */

  /*! \brief This signal returns the error between the desired value and
    the current value : \f$ {\bf s}^*(t) - {\bf s}(t)\f$ */
  dg::SignalTimeDependent<ml::Vector,int> errorSOUT;

  /*! \brief This signal returns the Jacobian of the current value
    according to the robot state: \f$ J(t) = \frac{\delta{\bf s}^*(t)}{\delta {\bf q}(t)}\f$ */
  dg::SignalTimeDependent<ml::Matrix,int> jacobianSOUT;

  /*! \brief Compute the new value of the feature \f$ {\bf s}(t)\f$ */
  dg::SignalTimeDependent<ml::Vector,int> activationSOUT;

  /*! \brief Returns the dimension of the feature as an output signal. */
  dg::SignalTimeDependent<unsigned int,int> dimensionSOUT;

  /*! \brief This method write a graph description on the file named FileName. */
  virtual std::ostream & writeGraph(std::ostream & os) const;

  /*! @} */

};

} // namespace sot



#endif // #ifndef __SOT_FEATURE_ABSTRACT_H__

