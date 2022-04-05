/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_GAIN_HYPERBOLIC_HH__
#define __SOT_GAIN_HYPERBOLIC_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(gain_hyperbolic_EXPORTS)
#define SOTGAINHYPERBOLIC_EXPORT __declspec(dllexport)
#else
#define SOTGAINHYPERBOLIC_EXPORT __declspec(dllimport)
#endif
#else
#define SOTGAINHYPERBOLIC_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/** \brief Hyperbolic gain.
 * It follows the law \f[ g(e) = a \frac{\tanh(-b(||e|| - d)) + 1}{2} + c \f]
 * The default coefficients are:
 * - \f$ a = 0   \f$,
 * - \f$ b = 0   \f$,
 * - \f$ c = 0.1 \f$,
 * - \f$ d = 0   \f$.
 */
class SOTGAINHYPERBOLIC_EXPORT GainHyperbolic : public dynamicgraph::Entity {
 public: /* --- CONSTANTS --- */
  /* Default values. */
  static const double ZERO_DEFAULT;   // = 0.1
  static const double INFTY_DEFAULT;  // = 0.1
  static const double TAN_DEFAULT;    // = 1.

 public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display(std::ostream &os) const;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

 protected:
  /* Parameters of the hyperbolic-gain function:
   * lambda (x) = a * exp (-b*x) + c. */
  double coeff_a;
  double coeff_b;
  double coeff_c;
  double coeff_d;

 public: /* --- CONSTRUCTORS ---- */
  GainHyperbolic(const std::string &name);
  GainHyperbolic(const std::string &name, const double &lambda);
  GainHyperbolic(const std::string &name, const double &valueAt0,
                 const double &valueAtInfty, const double &tanAt0,
                 const double &decal0);

 public: /* --- INIT --- */
  inline void init(void) { init(ZERO_DEFAULT, INFTY_DEFAULT, TAN_DEFAULT, 0); }
  inline void init(const double &lambda) { init(lambda, lambda, 1., 0); }
  /** Set the coefficients.
   * - \f$ a = valueAt0 - valueAtInfty \f$,
   * - \f$ b = \frac{tanAt0}{2*a} \f$, or \f$ b = 0 \f$ if \f$ a == 0 \f$,
   * - \f$ c = valueAtInfty \f$,
   * - \f$ d = decal0 \f$.
   */
  void init(const double &valueAt0, const double &valueAtInfty,
            const double &tanAt0, const double &decal0);
  void forceConstant(void);

 public: /* --- SIGNALS --- */
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> errorSIN;
  dynamicgraph::SignalTimeDependent<double, int> gainSOUT;

 protected:
  double &computeGain(double &res, int t);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_GAIN_HYPERBOLIC_HH__
