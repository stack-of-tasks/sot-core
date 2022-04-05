/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FEATURE_POINT6D_HH__
#define __SOT_FEATURE_POINT6D_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/exception-feature.hh>
#include <sot/core/exception-task.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/matrix-geometry.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(feature_point6d_EXPORTS)
#define SOTFEATUREPOINT6D_EXPORT __declspec(dllexport)
#else
#define SOTFEATUREPOINT6D_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFEATUREPOINT6D_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/*!
  \class FeaturePoint6d
  \brief Class that defines point-6d control feature.
  \deprecated This class was replaced by FeaturePose.
*/
class [[deprecated(
    "replaced by FeaturePose")]] SOTFEATUREPOINT6D_EXPORT FeaturePoint6d
    : public FeatureAbstract,
      public FeatureReferenceHelper<FeaturePoint6d> {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  /* --- Frame type --------------------------------------------------------- */
 protected:
  enum ComputationFrameType { FRAME_DESIRED, FRAME_CURRENT };
  static const ComputationFrameType COMPUTATION_FRAME_DEFAULT;

 public:
  /// \brief Set computation frame
  void computationFrame(const std::string &inFrame);
  /// \brief Get computation frame
  std::string computationFrame() const;

 private:
  ComputationFrameType computationFrame_;

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dynamicgraph::SignalPtr<MatrixHomogeneous, int> positionSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> velocitySIN;
  dynamicgraph::SignalPtr<dynamicgraph::Matrix, int> articularJacobianSIN;

  using FeatureAbstract::errorSOUT;
  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::selectionSIN;

  /*! \name Dealing with the reference value to be reach with this feature.
    @{  */
  DECLARE_REFERENCE_FUNCTIONS(FeaturePoint6d);
  /*! @} */

 public:
  FeaturePoint6d(const std::string &name);
  virtual ~FeaturePoint6d(void) {}

  virtual unsigned int &getDimension(unsigned int &dim, int time);

  virtual dynamicgraph::Vector &computeError(dynamicgraph::Vector & res,
                                             int time);
  virtual dynamicgraph::Vector &computeErrordot(dynamicgraph::Vector & res,
                                                int time);
  virtual dynamicgraph::Matrix &computeJacobian(dynamicgraph::Matrix & res,
                                                int time);

  /** Static Feature selection. */
  inline static Flags selectX(void) { return Flags("100000"); }
  inline static Flags selectY(void) { return Flags("010000"); }
  inline static Flags selectZ(void) { return Flags("001000"); }
  inline static Flags selectRX(void) { return Flags("000100"); }
  inline static Flags selectRY(void) { return Flags("000010"); }
  inline static Flags selectRZ(void) { return Flags("000001"); }

  inline static Flags selectTranslation(void) { return Flags("111000"); }
  inline static Flags selectRotation(void) { return Flags("000111"); }

  virtual void display(std::ostream & os) const;

 public:
  void servoCurrentPosition(void);

 private:
  // Intermediate variables for internal computations
  Eigen::Vector3d v_, omega_, errordot_t_, errordot_th_, Rreftomega_, t_, tref_;
  VectorUTheta error_th_;
  MatrixRotation R_, Rref_, Rt_, Rreft_;
  Eigen::Matrix3d P_, Pinv_;
  double accuracy_;
  void inverseJacobianRodrigues();
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_FEATURE_POINT6D_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
