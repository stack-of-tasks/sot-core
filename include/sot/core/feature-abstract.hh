/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FEATURE_ABSTRACT_H__
#define __SOT_FEATURE_ABSTRACT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include "sot/core/api.hh"
#include "sot/core/deprecated.hh"
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <sot/core/flags.hh>
#include <sot/core/pool.hh>

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

  \par par_features_definition Definition
  In short, a feature is a data evolving according to time.  It is defined
  by a vector \f${\bf s}({\bf q}) \in \mathbb{R}^n \f$ where \f$ {\bf q} \f$ is
  a robot configuration, which depends on the time \f$ t \f$. By default a
  feature has a desired \f${\bf s}^*(t) \f$. \f${\bf s}^*\f$ is provided by
  another feature of the same type called reference. The feature is in charge of
  collecting its own current state.  A feature is supposed to compute an error
  between its current state and the desired one: \f$ E(t) = e({\bf q}(t), t) =
  {\bf s}({\bf q}(t)) \ominus {\bf s}^*(t) \f$. Here, \f$ \ominus \f$ is the
  difference operator of Lie group in which \f$ {\bf s} \f$ and \f$ {\bf s}^*
  \f$ are. The documentation below assumes the Lie group is a vector space and
  \f$\ominus\f$ is the usual difference operator.

  A feature computes:
  \li the Jacobian according to the robot state vector \f$ J =
      \frac{\partial e}{\partial {\bf q}} = \frac{\partial{\bf s}}{\partial {\bf
  q}}\f$. \li the partial derivative of the error \f$ e \f$: \f$ \frac{\partial
  e}{\partial t} = - \frac{d{\bf s}^*}{dt}\f$.

  The task is in general computed from the value of the feature at the
  current instant \f$E(t) = e({\bf q},t)\f$. The derivative of \f$ E \f$ is:
  \f[
    \frac{dE}{dt} = J({\bf q}) \dot{q} + \frac{\partial e}{\partial t}
  \f]

  \image html feature.png "Feature diagram: Feature types derive from
  FeatureAbstract. Each feature has a reference of the same type and
  compute an error by comparing
  errorSIN
  signals from itself and from the
  reference."

*/
class SOT_CORE_EXPORT FeatureAbstract : public Entity {
public:
  /*! \brief Store the name of the class. */
  static const std::string CLASS_NAME;

  /*! \brief Returns the name class. */
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  /*! \brief Register the feature in the stack of tasks. */
  void featureRegistration(void);

  void initCommands(void);

public:
  /*! \brief Default constructor: the name of the class should be given. */
  FeatureAbstract(const std::string &name);
  /*! \brief Default destructor */
  virtual ~FeatureAbstract(void){};

  /*! \name Methods returning the dimension of the feature.
    @{ */

  /*! \brief Verbose method.
    \par res: The integer in which the dimension will be return.
    \par time: The time at which the feature should be considered.
    \return Dimension of the feature.
    \note Be careful with features changing their dimension according to time.
  */
  virtual unsigned int &getDimension(unsigned int &res, int time) = 0;

  /*! \brief Short method
    \par time: The time at which the feature should be considered.
    \return Dimension of the feature.
    \note Be careful with features changing their dimension according to time.
  */
  inline unsigned int getDimension(int time) {
    unsigned int res;
    getDimension(res, time);
    return res;
  }

  /*! \brief Shortest method
    \return Dimension of the feature.
    \note The feature is not changing its dimension according to time.
  */
  inline unsigned int getDimension(void) const {
    return dimensionSOUT.accessCopy();
  }
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
  virtual dynamicgraph::Vector &computeError(dynamicgraph::Vector &res,
                                             int time) = 0;

  /*! \brief Compute the Jacobian of the error according the robot state.

    \par[out] res: The matrix in which the error will be written.
    \return The matrix res with the appropriate values.
  */
  virtual dynamicgraph::Matrix &computeJacobian(dynamicgraph::Matrix &res,
                                                int time) = 0;

  /// Callback for signal errordotSOUT
  ///
  /// Copy components of the input signal errordotSIN defined by selection
  /// flag selectionSIN.
  virtual dynamicgraph::Vector &computeErrorDot(dynamicgraph::Vector &res,
                                                int time);

  /*! @} */

  /* --- SIGNALS ------------------------------------------------------------ */
public:
  /*! \name Signals
    @{
  */

  /*! \name Input signals:
    @{ */
  /*! \brief This vector specifies which dimension are used to perform the
    computation. For instance let us assume that the feature is a 3D point. If
    only the Y-axis should be used for computing error, activation and Jacobian,
    then the vector to specify
    is \f$ [ 0 1 0] \f$.*/
  SignalPtr<Flags, int> selectionSIN;

  /// Derivative of the reference value.
  SignalPtr<dynamicgraph::Vector, int> errordotSIN;

  /*! @} */

  /*! \name Output signals:
    @{ */

  /*! \brief This signal returns the error between the desired value and
    the current value : \f$ E(t) = {\bf s}(t) - {\bf s}^*(t)\f$ */
  SignalTimeDependent<dynamicgraph::Vector, int> errorSOUT;

  /*! \brief Derivative of the error with respect to time:
   * \f$ \frac{\partial e}{\partial t} = - \frac{d{\bf s}^*}{dt} \f$ */
  SignalTimeDependent<dynamicgraph::Vector, int> errordotSOUT;

  /*! \brief Jacobian of the error wrt the robot state:
   * \f$ J = \frac{\partial {\bf s}}{\partial {\bf q}}\f$ */
  SignalTimeDependent<dynamicgraph::Matrix, int> jacobianSOUT;

  /*! \brief Returns the dimension of the feature as an output signal. */
  SignalTimeDependent<unsigned int, int> dimensionSOUT;

  /*! \brief This method write a graph description on the file named
    FileName. */
  virtual std::ostream &writeGraph(std::ostream &os) const;

  virtual SignalTimeDependent<dynamicgraph::Vector, int> &getErrorDot() {
    return errordotSOUT;
  }

  /*! @} */

  /* --- REFERENCE VALUE S* ------------------------------------------------- */
public:
  /*! \name Reference
    @{
  */
  virtual void setReference(FeatureAbstract *sdes) = 0;
  virtual void unsetReference(void) { setReference(NULL); }
  virtual const FeatureAbstract *getReferenceAbstract(void) const = 0;
  virtual FeatureAbstract *getReferenceAbstract(void) = 0;
  virtual bool isReferenceSet(void) const { return false; }

  virtual void addDependenciesFromReference(void) = 0;
  virtual void removeDependenciesFromReference(void) = 0;

  /* Commands for bindings. */
  void setReferenceByName(const std::string &name);
  std::string getReferenceByName(void) const;
  /*! @} */
};

template <class FeatureSpecialized> class FeatureReferenceHelper {
  FeatureSpecialized *ptr;
  FeatureAbstract *ptrA;

public:
  FeatureReferenceHelper(void) : ptr(NULL) {}

  void setReference(FeatureAbstract *sdes);
  // void setReferenceByName( const std::string & name );
  void unsetReference(void) { setReference(NULL); }
  bool isReferenceSet(void) const { return ptr != NULL; }
  FeatureSpecialized *getReference(void) { return ptr; }
  const FeatureSpecialized *getReference(void) const { return ptr; }
};

template <class FeatureSpecialized>
void FeatureReferenceHelper<FeatureSpecialized>::setReference(
    FeatureAbstract *sdes) {
  ptr = dynamic_cast<FeatureSpecialized *>(sdes);
  ptrA = ptr;
}

#define DECLARE_REFERENCE_FUNCTIONS(FeatureSpecialized)                        \
  typedef FeatureReferenceHelper<FeatureSpecialized> SP;                       \
  virtual void setReference(FeatureAbstract *sdes) {                           \
    if (sdes == NULL) {                                                        \
      /* UNSET */                                                              \
      if (SP::isReferenceSet())                                                \
        removeDependenciesFromReference();                                     \
      SP::unsetReference();                                                    \
    } else {                                                                   \
      /* SET */                                                                \
      SP::setReference(sdes);                                                  \
      if (SP::isReferenceSet())                                                \
        addDependenciesFromReference();                                        \
    }                                                                          \
  }                                                                            \
  virtual const FeatureAbstract *getReferenceAbstract(void) const {            \
    return SP::getReference();                                                 \
  }                                                                            \
  virtual FeatureAbstract *getReferenceAbstract(void) {                        \
    return (FeatureAbstract *)SP::getReference();                              \
  }                                                                            \
  bool isReferenceSet(void) const { return SP::isReferenceSet(); }             \
  virtual void addDependenciesFromReference(void);                             \
  virtual void removeDependenciesFromReference(void)
/* END OF define DECLARE_REFERENCE_FUNCTIONS */

#define DECLARE_NO_REFERENCE                                                   \
  virtual void setReference(FeatureAbstract *) {}                              \
  virtual const FeatureAbstract *getReferenceAbstract(void) const {            \
    return NULL;                                                               \
  }                                                                            \
  virtual FeatureAbstract *getReferenceAbstract(void) { return NULL; }         \
  virtual void addDependenciesFromReference(void) {}                           \
  virtual void removeDependenciesFromReference(void) {}                        \
  /* To force a ; */ bool NO_REFERENCE
/* END OF define DECLARE_REFERENCE_FUNCTIONS */

} // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __SOT_FEATURE_ABSTRACT_H__
