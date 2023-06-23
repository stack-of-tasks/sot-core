/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FEATURE_VECTOR3_HH__
#define __SOT_FEATURE_VECTOR3_HH__

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
#if defined(feature_vector3_EXPORTS)
#define SOTFEATUREVECTOR3_EXPORT __declspec(dllexport)
#else
#define SOTFEATUREVECTOR3_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFEATUREVECTOR3_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/*!
  \class FeatureVector3
  \brief Class that defines point-3d control feature
*/
class SOTFEATUREVECTOR3_EXPORT FeatureVector3 : public FeatureAbstract {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  DECLARE_NO_REFERENCE;

 protected:
  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> vectorSIN;
  dynamicgraph::SignalPtr<MatrixHomogeneous, sigtime_t> positionSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Matrix, sigtime_t> articularJacobianSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, sigtime_t> positionRefSIN;

  using FeatureAbstract::errorSOUT;
  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::selectionSIN;

 public:
  FeatureVector3(const std::string &name);
  virtual ~FeatureVector3(void) {}

  virtual size_type &getDimension(size_type &dim, sigtime_t time);

  virtual dynamicgraph::Vector &computeError(dynamicgraph::Vector &res,
                                             sigtime_t time);
  virtual dynamicgraph::Matrix &computeJacobian(dynamicgraph::Matrix &res,
                                                sigtime_t time);

  virtual void display(std::ostream &os) const;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_FEATURE_VECTOR3_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
