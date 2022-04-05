/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FEATURE_GENERIC_HH__
#define __SOT_FEATURE_GENERIC_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/exception-task.hh>
#include <sot/core/feature-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(feature_generic_EXPORTS)
#define SOTFEATUREGENERIC_EXPORT __declspec(dllexport)
#else
#define SOTFEATUREGENERIC_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFEATUREGENERIC_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/*!
  \class FeatureGeneric
  \brief Class that defines a generic implementation of the abstract interface
  for features.

  This class is very useful if the feature can be easily computed using
  the basic operator provided. For instance a free space controller on a
  end-effector  is basically directly computed from the Jacobian provided
  by dyn and some appropriate  addition and soustraction.
  Instead of building a specific feature for this, it is possible to use the
  signals  and plug the computed error, Jacobian and activation to the input
  of this generic feature implementation.

*/
class SOTFEATUREGENERIC_EXPORT FeatureGeneric
    : public FeatureAbstract,
      FeatureReferenceHelper<FeatureGeneric> {
 public:
  /*! Field storing the class name. */
  static const std::string CLASS_NAME;
  /*! Returns the name of the class. */
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

 protected:
  dynamicgraph::Vector::Index dimensionDefault;

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  /*! \name dynamicgraph::Signals
    @{
  */
  /*! \name Input signals
    @{
   */
  /*! \brief Input for the error. */
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> errorSIN;

  /*! \brief Input for the Jacobian. */
  dynamicgraph::SignalPtr<dynamicgraph::Matrix, int> jacobianSIN;

  /*! @} */

  /*! \name Output signals
    @{
  */
  /*! \brief Publish the jacobian of the feature according to the robot state.
   */
  using FeatureAbstract::jacobianSOUT;

  /*! \brief Publish the error between the desired and the current value of the
      feature. */
  using FeatureAbstract::errorSOUT;

 public:
  /*! \brief Default constructor */
  FeatureGeneric(const std::string &name);

  /*! \brief Default destructor */
  virtual ~FeatureGeneric(void) {}

  /*! \brief Get the dimension of the feature. */
  virtual unsigned int &getDimension(unsigned int &dim, int time);

  /*! \name Methods to trigger computation related to this feature.
    @{
  */

  /*! \brief Compute the error between the desired value and the value itself.
   */
  virtual dynamicgraph::Vector &computeError(dynamicgraph::Vector &res,
                                             int time);

  /*! \brief Compute the Jacobian of the value according to the robot state.. */
  virtual dynamicgraph::Matrix &computeJacobian(dynamicgraph::Matrix &res,
                                                int time);

  /*! @} */

  /*! \brief Display the information related to this generic implementation. */
  virtual void display(std::ostream &os) const;

  /*! \name Dealing with the reference value to be reach with this feature.
    @{
  */
  DECLARE_REFERENCE_FUNCTIONS(FeatureGeneric);
  /*! @} */
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_FEATURE_GENERIC_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
