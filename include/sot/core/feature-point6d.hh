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
namespace dg = dynamicgraph;

/*!
  \class FeaturePoint6d
  \brief Class that defines point-6d control feature
*/
class SOTFEATUREPOINT6D_EXPORT FeaturePoint6d
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
  dg::SignalPtr<MatrixHomogeneous, int> positionSIN;
  dg::SignalPtr<dg::Vector, int> velocitySIN;
  dg::SignalPtr<dg::Matrix, int> articularJacobianSIN;

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

  virtual dg::Vector &computeError(dg::Vector &res, int time);
  virtual dg::Vector &computeErrordot(dg::Vector &res, int time);
  virtual dg::Matrix &computeJacobian(dg::Matrix &res, int time);

  /** Static Feature selection. */
  inline static Flags selectX(void) { return FLAG_LINE_1; }
  inline static Flags selectY(void) { return FLAG_LINE_2; }
  inline static Flags selectZ(void) { return FLAG_LINE_3; }
  inline static Flags selectRX(void) { return FLAG_LINE_4; }
  inline static Flags selectRY(void) { return FLAG_LINE_5; }
  inline static Flags selectRZ(void) { return FLAG_LINE_6; }

  inline static Flags selectTranslation(void) { return Flags(7); }
  inline static Flags selectRotation(void) { return Flags(56); }

  virtual void display(std::ostream &os) const;

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

#endif // #ifndef __SOT_FEATURE_POINT6D_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
