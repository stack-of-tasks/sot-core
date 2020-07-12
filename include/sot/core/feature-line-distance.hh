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
namespace dg = dynamicgraph;

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
  dg::SignalPtr<MatrixHomogeneous, int> positionSIN;
  dg::SignalPtr<dg::Matrix, int> articularJacobianSIN;
  dg::SignalPtr<dg::Vector, int> positionRefSIN;
  dg::SignalPtr<dg::Vector, int> vectorSIN;
  dg::SignalTimeDependent<dg::Vector, int> lineSOUT;

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

  virtual dg::Vector &computeError(dg::Vector &res, int time);
  virtual dg::Matrix &computeJacobian(dg::Matrix &res, int time);
  dg::Vector &computeLineCoordinates(dg::Vector &cood, int time);

  virtual void display(std::ostream &os) const;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif // #ifndef __SOT_FEATURE_LINEDISTANCE_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
