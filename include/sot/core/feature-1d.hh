/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FEATURE_1D_HH__
#define __SOT_FEATURE_1D_HH__

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
#if defined(feature_1d_EXPORTS)
#define SOTFEATURE1D_EXPORT __declspec(dllexport)
#else
#define SOTFEATURE1D_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFEATURE1D_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {
/*!
  \class Feature1D
  \brief Simple test: the task is defined to be e_2 = .5 . e'.e, with
  e the mother task. The jacobian is then J_2 = e'.J, J being the jacobian
  of the mother task.

*/
class SOTFEATURE1D_EXPORT Feature1D : public FeatureAbstract,
                                      FeatureReferenceHelper<Feature1D> {

public:
  /*! Field storing the class name. */
  static const std::string CLASS_NAME;
  /*! Returns the name of the class. */
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

protected:
  /* --- SIGNALS ------------------------------------------------------------ */
public:
  /*! \name Signals
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
   * feature. */
  using FeatureAbstract::errorSOUT;

public:
  /*! \brief Default constructor */
  Feature1D(const std::string &name);

  /*! \brief Default destructor */
  virtual ~Feature1D(void) {}

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

  /*! \brief Display the information related to this 1D implementation. */
  virtual void display(std::ostream &os) const;

  /*! \name Dealing with the reference value to be reach with this feature.
    @{
  */
  DECLARE_REFERENCE_FUNCTIONS(Feature1D);
  /*! @} */
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif // #ifndef __SOT_FEATURE_1D_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
