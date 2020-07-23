/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FEATURE_JOINTLIMITS_HH__
#define __SOT_FEATURE_JOINTLIMITS_HH__

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
#if defined(feature_joint_limits_EXPORTS)
#define SOTFEATUREJOINTLIMITS_EXPORT __declspec(dllexport)
#else
#define SOTFEATUREJOINTLIMITS_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFEATUREJOINTLIMITS_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/*!
  \class FeatureJointLimits
  \brief Class that defines gradient vector for jl avoidance.
*/
class SOTFEATUREJOINTLIMITS_EXPORT FeatureJointLimits
    : public FeatureAbstract,
      FeatureReferenceHelper<FeatureJointLimits> {

public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

protected:
  double threshold;
  const static double THRESHOLD_DEFAULT; // = .9;

  /*   unsigned int freeFloatingIndex,freeFloatingSize; */
  /*   static const unsigned int FREE_FLOATING_INDEX = 0; */
  /*   static const unsigned int FREE_FLOATING_SIZE = 5; */

  /* --- SIGNALS ------------------------------------------------------------ */
public:
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> jointSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> upperJlSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> lowerJlSIN;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> widthJlSINTERN;

  using FeatureAbstract::selectionSIN;

  using FeatureAbstract::errorSOUT;
  using FeatureAbstract::jacobianSOUT;

  /*! \name Dealing with the reference value to be reach with this feature.
    @{
  */
  DECLARE_REFERENCE_FUNCTIONS(FeatureJointLimits);
  /*! @} */

public:
  FeatureJointLimits(const std::string &name);
  virtual ~FeatureJointLimits(void) {}

  virtual unsigned int &getDimension(unsigned int &dim, int time);

  virtual dynamicgraph::Vector &computeError(dynamicgraph::Vector &res,
                                             int time);
  virtual dynamicgraph::Matrix &computeJacobian(dynamicgraph::Matrix &res,
                                                int time);
  dynamicgraph::Vector &computeWidthJl(dynamicgraph::Vector &res,
                                       const int &time);

  /** Static Feature selection. */
  inline static Flags selectActuated(void);

  virtual void display(std::ostream &os) const;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif // #ifndef __SOT_FEATURE_JOINTLIMITS_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
