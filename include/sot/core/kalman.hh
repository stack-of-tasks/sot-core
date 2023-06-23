/*
 * Copyright 2010, 2011, 2012
 * Nicolas Mansard,
 * François Bleibel,
 * Olivier Stasse,
 * Florent Lamiraux
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_KALMAN_H
#define __SOT_KALMAN_H

/* -------------------------------------------------------------------------- */
/* --- INCLUDE -------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/linear-algebra.h>

#include <Eigen/LU>

/* -------------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------------ */
/* -------------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(kalman_EXPORTS)
#define SOT_KALMAN_EXPORT __declspec(dllexport)
#else
#define SOT_KALMAN_EXPORT __declspec(dllimport)
#endif
#else
#define SOT_KALMAN_EXPORT
#endif

/* -------------------------------------------------------------------------- */
/* --- CLASSE --------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

class SOT_KALMAN_EXPORT Kalman : public Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

 protected:
  std::size_t size_state;
  std::size_t size_measure;
  double dt;

 public:
  SignalPtr<Vector, sigtime_t> measureSIN;          // y
  SignalPtr<Matrix, sigtime_t> modelTransitionSIN;  // F
  SignalPtr<Matrix, sigtime_t> modelMeasureSIN;     // H
  SignalPtr<Matrix, sigtime_t> noiseTransitionSIN;  // Q
  SignalPtr<Matrix, sigtime_t> noiseMeasureSIN;     // R

  SignalPtr<Vector, sigtime_t> statePredictedSIN;        // x_{k|k-1}
  SignalPtr<Vector, sigtime_t> observationPredictedSIN;  // y_pred = h (x_{k|k-1})
  SignalTimeDependent<Matrix, sigtime_t> varianceUpdateSOUT;  // P
  SignalTimeDependent<Vector, sigtime_t> stateUpdateSOUT;     // X_est

  SignalTimeDependent<Matrix, sigtime_t> gainSINTERN;        // K
  SignalTimeDependent<Matrix, sigtime_t> innovationSINTERN;  // S

 public:
  virtual std::string getDocString() const {
    return "Implementation of extended Kalman filter     \n"
           "\n"
           "  Dynamics of the system:                    \n"
           "\n"
           "    x = f (x   , u   ) + w       (state)      \n"
           "     k      k-1   k-1     k-1                 \n"
           "\n"
           "    y = h (x ) + v               (observation)\n"
           "     k      k     k                           \n"
           "\n"
           "  Prediction:\n"
           "\n"
           "    ^          ^                       \n"
           "    x     = f (x       , u   )     (state) \n"
           "     k|k-1      k-1|k-1   k-1          \n"
           "\n"
           "                           T           \n"
           "    P     = F    P        F    + Q (covariance)\n"
           "     k|k-1   k-1  k-1|k-1  k-1         \n"
           "\n"
           "  with\n"
           "           \\                         \n"
           "           d f  ^                         \n"
           "    F    = --- (x       , u   )           \n"
           "     k-1   \\     k-1|k-1   k-1            \n"
           "           d x                         \n"
           "\n"
           "         \\                             \n"
           "         d h  ^                          \n"
           "    H  = --- (x       )                     \n"
           "     k   \\     k-1|k-1                      \n"
           "         d x                             \n"

           "  Update:\n"
           "\n"
           "                ^                            \n"
           "    z = y  - h (x     )             (innovation)\n"
           "     k   k       k|k-1                       \n"
           "                   T                          \n"
           "    S = H  P      H  + R            (innovation covariance)\n"
           "     k   k  k|k-1  k                          \n"
           "                T  -1                         \n"
           "    K = P      H  S                 (Kalman gain)\n"
           "     k   k|k-1  k  k                          \n"
           "    ^     ^                                   \n"
           "    x   = x      + K  z             (state)   \n"
           "     k|k   k|k-1    k  k                      \n"
           "\n"
           "    P   =(I - K  H ) P                        \n"
           "     k|k       k  k   k|k-1                   \n"
           "\n"
           "  Signals\n"
           "    - input(vector)::x_pred:  state prediction\n"
           "                                                         ^\n"
           "    - input(vector)::y_pred:  observation prediction: h (x     )\n"
           "                                                          k|k-1\n"
           "    - input(matrix)::F:       partial derivative wrt x of f\n"
           "    - input(vector)::y:       measure         \n"
           "    - input(matrix)::H:       partial derivative wrt x of h\n"
           "    - input(matrix)::Q:       variance of noise w\n"
           "                                                 k-1\n"
           "    - input(matrix)::R:       variance of noise v\n"
           "                                                 k\n"
           "    - output(matrix)::P_pred: variance of prediction\n"
           "                                               ^\n"
           "    - output(vector)::x_est:  state estimation x\n"
           "                                                k|k\n";
  }

 protected:
  Matrix &computeVarianceUpdate(Matrix &P_k_k, const sigtime_t &time);
  Vector &computeStateUpdate(Vector &x_est, const sigtime_t &time);

  void setStateEstimation(const Vector &x0) {
    stateEstimation_ = x0;
    stateUpdateSOUT.recompute(0);
  }

  void setStateVariance(const Matrix &P0) {
    stateVariance_ = P0;
    varianceUpdateSOUT.recompute(0);
  }
  // Current state estimation
  // ^
  // x
  //  k-1|k-1
  Vector stateEstimation_;
  // Variance of current state estimation
  // P
  //  k-1|k-1
  Matrix stateVariance_;

  //                          ^
  // Innovation: z  = y  - H  x
  //              k    k    k  k|k-1
  Vector z_;

  // F    P
  //  k-1  k-1|k-1
  Matrix FP_;

  // Variance prediction
  // P
  //  k|k-1
  Matrix Pk_k_1_;

  // Innovation covariance
  Matrix S_;

  // Kalman Gain
  Matrix K_;

 public:
  Kalman(const std::string &name);
  /* --- Entity --- */
  void display(std::ostream &os) const;
};

}  // namespace sot
}  // namespace dynamicgraph

/*!
  \file Kalman.h
  \brief  Extended kalman filter implementation
*/

#endif
