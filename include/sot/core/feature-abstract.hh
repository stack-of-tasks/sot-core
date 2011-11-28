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
#include <sot/core/flags.hh>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <sot/core/pool.hh>
#include "sot/core/api.hh"

/* STD */
#include <string>

namespace dynamicgraph {
  namespace sot {

    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    /*! \class FeatureAbstract
      \ingroup features
      \brief This class gives the abstract definition of a feature.

      In short, a feature is a data evolving according to time.  It is defined
      by a vector \f${\bf s}(t) \in \mathbb{R}^n \f$ where \f$ t \f$ is the
      time.  By default a feature has a desired \f${\bf s}^*(t) \f$.  The
      feature is in charge of collecting its own current state.  A feature is
      supposed to compute an error between its current state and the desired
      one: \f$ e(t) = {\bf s}^*(t) - {\bf s}(t) \f$.  A feature is supposed to
      compute a Jacobian according to the robot state vector \f$ \frac{\delta
      {\bf s}(t)}{\delta {\bf q}(t)}\f$.

      \bigskip
      More precisely, a feature is a derivable function of the robot
      configuration $q$ and the universe \f$\Omega\f$ into a real vector space:
      \f$ f: q,\Omega \rightarrow f(q,\Omega) \in R^n\f$. The object is able to
      compute the value, and the value of the Jacobian of \f$f\f$ with respect
      to \f$q\f$, \f$J = \frac{\partial f}{\partial q}\f$.

      The task is in general computed from the value of the feature at the
      current instant \f$f(q(t),\Omega(t))\f$, the Jacobian \f$J\f$ and
      evolution of the feature with the evolution of the universe, abusively
      denoted as a variation along the variable \f$t\f$ alone: \f$\frac{\partial
      f}{\partial t} = \frac{\partial f}{\partial \Omega} \dot{\Omega}\f$.

      In general, the feature is computed as the error \f$f = e(q,\Omega)\f$
      between the value at the current robot and universe configurations
      \f$s(q,\Omega)\f$, and a reference value that does not depend on the robot
      current configuration, and thus that is generally denoted \f$s^* = s^*(t)\f$.
      In general, \f$s\f$ and \f$s^*\f$ evolves in the same space, and thus, two
      objects of the same classes are used to represent \f$s\f$ on one side and
      \f$s^*\f$ on the other. A generic solution to maintain a reference on the
      object \f$s^*\f$ from the object \f$s\f$ is provided, but is not mandatory. In
      that cases, the signal errorSOUT is linked to the update state
      of the input of \f$s\f$, and is also automatically linked to the input of
      \f$s^*\f$ as soon as \f$s^*\f$ is specifified.

    */
    class SOT_CORE_EXPORT FeatureAbstract
      :public Entity
    {
    public:
      /*! \brief Store the name of the class. */
      static const std::string CLASS_NAME;

      /*! \brief Returns the name class. */
      virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

      /*! \brief Register the feature in the stack of tasks. */
      void featureRegistration( void );

      void initCommands( void );

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
	\return The matrix res with the appropriate values.
      */
      virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time ) = 0;

      /*! @} */

      /* --- SIGNALS ------------------------------------------------------------ */
    public:

      /*! \name Signals
	@{
      */

      /*! \name Input signals:
	@{ */
      /*! \brief This vector specifies which dimension are used to perform the computation.
	For instance let us assume that the feature is a 3D point. If only the Y-axis should
	be used for computing error, activation and Jacobian, then the vector to specify
	is \f$ [ 0 1 0] \f$.*/
      SignalPtr< Flags,int > selectionSIN;
      /*! @} */

      /*! \name Output signals:
	@{ */

      /*! \brief This signal returns the error between the desired value and
	the current value : \f$ {\bf s}^*(t) - {\bf s}(t)\f$ */
      SignalTimeDependent<ml::Vector,int> errorSOUT;

      /*! \brief This signal returns the Jacobian of the current value
	according to the robot state: \f$ J(t) = \frac{\delta{\bf s}^*(t)}{\delta {\bf q}(t)}\f$ */
      SignalTimeDependent<ml::Matrix,int> jacobianSOUT;

      /*! \brief Returns the dimension of the feature as an output signal. */
      SignalTimeDependent<unsigned int,int> dimensionSOUT;

      /*! \brief This method write a graph description on the file named FileName. */
      virtual std::ostream & writeGraph(std::ostream & os) const;

      /*! \brief Return true for children that implements the errordot
          functionalities. */
      virtual bool withErrorDot( void ) const { return false; }
      virtual SignalTimeDependent<ml::Vector,int>& getErrorDot( void ) {throw;}

      /*! @} */

      /* --- REFERENCE VALUE S* ------------------------------------------------- */
    public:

      /*! \name Reference
	@{
      */
      virtual void setReference( FeatureAbstract * sdes ) = 0;
      virtual void unsetReference( void ) { setReference(NULL); }
      virtual const FeatureAbstract * getReferenceAbstract( void ) const = 0;
      virtual FeatureAbstract * getReferenceAbstract( void ) = 0;
      virtual bool isReferenceSet( void ) const { return false; }

      virtual void addDependenciesFromReference( void ) = 0;
      virtual void removeDependenciesFromReference( void ) = 0;

      /* Commands for bindings. */
      void setReferenceByName( const std::string& name );
      std::string getReferenceByName( void ) const ;
      /*! @} */
    };


    template <class FeatureSpecialized>
    class FeatureReferenceHelper
    {
      FeatureSpecialized * ptr;
      FeatureAbstract * ptrA;

    public:
      FeatureReferenceHelper( void ) : ptr (NULL) {}

      void setReference( FeatureAbstract * sdes );
      //void setReferenceByName( const std::string & name );
      void unsetReference( void ) { setReference(NULL); }
      bool isReferenceSet( void ) const { return ptr != NULL; }
      FeatureSpecialized * getReference( void ){ return ptr; }
      const FeatureSpecialized * getReference( void ) const { return ptr; }
    };


    template <class FeatureSpecialized>
    void FeatureReferenceHelper<FeatureSpecialized>::
    setReference( FeatureAbstract * sdes )
    {
      ptr = dynamic_cast<FeatureSpecialized*> (sdes);
      ptrA=ptr;
    }

#define DECLARE_REFERENCE_FUNCTIONS(FeatureSpecialized)	\
  typedef FeatureReferenceHelper<FeatureSpecialized> SP; \
  virtual void setReference( FeatureAbstract * sdes ) \
  { \
    if( sdes==NULL ) \
      { \
	/* UNSET */ \
	if( SP::isReferenceSet() ) \
	  removeDependenciesFromReference(); \
	SP::unsetReference(); \
      } \
    else \
      { \
	/* SET */ \
	SP::setReference(sdes); \
	if( SP::isReferenceSet() ) \
	  addDependenciesFromReference(); \
      } \
  } \
  virtual const FeatureAbstract * getReferenceAbstract( void ) const {return SP::getReference();} \
  virtual FeatureAbstract * getReferenceAbstract( void ){return (FeatureAbstract*)SP::getReference();} \
  bool isReferenceSet( void ) const { return SP::isReferenceSet(); } \
  virtual void addDependenciesFromReference( void ); \
  virtual void removeDependenciesFromReference( void )
    /* END OF define DECLARE_REFERENCE_FUNCTIONS */

#define DECLARE_NO_REFERENCE	\
    virtual void setReference( FeatureAbstract * ) {} \
    virtual const FeatureAbstract * getReferenceAbstract( void ) const {return NULL; } \
    virtual FeatureAbstract * getReferenceAbstract( void ){return NULL; } \
    virtual void addDependenciesFromReference( void ) {}		\
    virtual void removeDependenciesFromReference( void ) {} \
    /* To force a ; */bool NO_REFERENCE
    /* END OF define DECLARE_REFERENCE_FUNCTIONS */

#define WITH_ERRORDOT  \
      virtual bool withErrorDot() const { return true; }   \
      virtual SignalTimeDependent<ml::Vector,int>& getErrorDot() { return errordotSOUT; } \
      dg::SignalTimeDependent< ml::Vector,int > errordotSOUT;   \
      virtual ml::Vector& computeErrorDot( ml::Vector& res,int time )


  } // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __SOT_FEATURE_ABSTRACT_H__

