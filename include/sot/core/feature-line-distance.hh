/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FEATURE_LINEDISTANCE_HH__
#define __SOT_FEATURE_LINEDISTANCE_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/exception-task.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/matrix-geometry.hh>
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(feature_line_distance_EXPORTS)
#define SOTFEATURELINEDISTANCE_EXPORT __declspec(dllexport)
#else
#define SOTFEATURELINEDISTANCE_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFEATURELINEDISTANCE_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/*!
  \class FeatureLineDistance
  \brief Class that defines point-3d control feature
*/
class SOTFEATURELINEDISTANCE_EXPORT FeatureLineDistance
    : public FeatureAbstract {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

 protected:
  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dynamicgraph::SignalPtr<MatrixHomogeneous, int> positionSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Matrix, int> articularJacobianSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> positionRefSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> vectorSIN;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> lineSOUT;

  using FeatureAbstract::errorSOUT;
  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::selectionSIN;

  /*! \name Dealing with the reference value to be reach with this feature.
    @{
  */
  DECLARE_NO_REFERENCE;
  /*! @} */

 public:
  FeatureLineDistance(const std::string &name);
  virtual ~FeatureLineDistance(void) {}

  virtual unsigned int &getDimension(unsigned int &dim, int time);

  virtual dynamicgraph::Vector &computeError(dynamicgraph::Vector &res,
                                             int time);
  virtual dynamicgraph::Matrix &computeJacobian(dynamicgraph::Matrix &res,
                                                int time);
  dynamicgraph::Vector &computeLineCoordinates(dynamicgraph::Vector &cood,
                                               int time);

  virtual void display(std::ostream &os) const;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_FEATURE_LINEDISTANCE_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
