/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_GAIN_ADAPTATIVE_HH__
#define __SOT_GAIN_ADAPTATIVE_HH__

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
#if defined(gain_adaptive_EXPORTS)
#define SOTGAINADAPTATIVE_EXPORT __declspec(dllexport)
#else
#define SOTGAINADAPTATIVE_EXPORT __declspec(dllimport)
#endif
#else
#define SOTGAINADAPTATIVE_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/** Exponentially decreasing gain.
 * It follows the law \f[ g(e) = a \exp (-b ||e||) + c \f].
 *
 * The default values for
 * - \f$ a = 0   \f$,
 * - \f$ b = 0   \f$,
 * - \f$ c = 0.1 \f$.
 */
class SOTGAINADAPTATIVE_EXPORT GainAdaptive : public dynamicgraph::Entity {

public: /* --- CONSTANTS --- */
  /* Default values. */
  static const double ZERO_DEFAULT;  // = 0.1
  static const double INFTY_DEFAULT; // = 0.1
  static const double TAN_DEFAULT;   // = 1.

public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display(std::ostream &os) const;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

protected:
  /* Parameters of the adaptative-gain function:
   * lambda (x) = a * exp (-b*x) + c. */
  double coeff_a;
  double coeff_b;
  double coeff_c;

public: /* --- CONSTRUCTORS ---- */
  GainAdaptive(const std::string &name);
  GainAdaptive(const std::string &name, const double &lambda);
  GainAdaptive(const std::string &name, const double &valueAt0,
               const double &valueAtInfty, const double &tanAt0);

public: /* --- INIT --- */
  inline void init(void) { init(ZERO_DEFAULT, INFTY_DEFAULT, TAN_DEFAULT); }
  inline void init(const double &lambda) { init(lambda, lambda, 1.); }
  void init(const double &valueAt0, const double &valueAtInfty,
            const double &tanAt0);
  /** \brief Set the gain
   * by providing the value at 0, at \f$ \infty \f$ and the percentage of
   * accomplishment between both to be reached when the error is
   * \c errorReference.
   *
   * To visualize the curve of the gain versus the error, use
   * \code{.py}
   * from dynamic_graph.sot.core.gain_adaptive import GainAdaptive
   * import numpy, matplotlib.pyplot as plt
   * g = GainAdaptive('g')
   * g.setByPoint(4.9, 0.001, 0.01, 0.1)
   *
   * errors = numpy.linspace(0, 0.1, 1000)
   * def compute(e):
   *     t = g.error.time + 1
   *     g.error.value = (e,)
   *     g.error.time = t
   *     g.gain.recompute(t)
   *     return g.gain.value
   *
   * gains = [ compute(e) for e in errors ]
   *
   * lg = plt.plot(errors, gains, 'r', label="Gain")
   * ld = plt.twinx().plot(errors, [ g*e for e,g in zip(errors,gains) ], 'b',
   *   label="Derivative")
   * lines = lg + ld
   * plt.legend(lines, [l.get_label() for l in lines])
   * plt.show()
   * \endcode
   */
  void initFromPassingPoint(const double &valueAt0, const double &valueAtInfty,
                            const double &errorReference,
                            const double &percentage);
  void forceConstant(void);

public: /* --- SIGNALS --- */
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> errorSIN;
  dynamicgraph::SignalTimeDependent<double, int> gainSOUT;

protected:
  double &computeGain(double &res, int t);

private:
  void addCommands();
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif // #ifndef __SOT_GAIN_ADAPTATIVE_HH__
