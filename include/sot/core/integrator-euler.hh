/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_INTEGRATOR_EULER_H__
#define __SOT_INTEGRATOR_EULER_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <sot/core/integrator-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

namespace internal {
template <class coefT> bool integratorEulerCoeffIsIdentity(const coefT c) {
  return c == 1;
}

bool integratorEulerCoeffIsIdentity(const Vector c) { return c.isOnes(); }

bool integratorEulerCoeffIsIdentity(const Matrix c) { return c.isIdentity(); }
} // namespace internal

/*!
 * \class IntegratorEuler
 * \brief integrates an ODE using a naive Euler integration.
 * TODO: change the integration method. For the moment, the highest
 * derivative of the output signal is computed using the
 * previous values of the other derivatives and the input
 * signal, then integrated n times, which will most certainly
 * induce a huge drift for ODEs with a high order at the denominator.
 */
template <class sigT, class coefT>
class IntegratorEuler : public IntegratorAbstract<sigT, coefT> {

public:
  virtual const std::string &getClassName(void) const;
  static std::string getTypeName(void) { return "Unknown"; }
  static const std::string CLASS_NAME;

public:
  using IntegratorAbstract<sigT, coefT>::SIN;
  using IntegratorAbstract<sigT, coefT>::SOUT;
  using IntegratorAbstract<sigT, coefT>::numerator;
  using IntegratorAbstract<sigT, coefT>::denominator;

public:
  IntegratorEuler(const std::string &name)
      : IntegratorAbstract<sigT, coefT>(name),
        derivativeSOUT(boost::bind(&IntegratorEuler<sigT, coefT>::derivative,
                                   this, _1, _2),
                       SOUT,
                       "sotIntegratorEuler(" + name +
                           ")::output(vector)::derivativesout") {
    this->signalRegistration(derivativeSOUT);

    using namespace dynamicgraph::command;

    setSamplingPeriod(0.005);

    this->addCommand("setSamplingPeriod",
                     new Setter<IntegratorEuler, double>(
                         *this, &IntegratorEuler::setSamplingPeriod,
                         "Set the time during two sampling."));
    this->addCommand("getSamplingPeriod",
                     new Getter<IntegratorEuler, double>(
                         *this, &IntegratorEuler::getSamplingPeriod,
                         "Get the time during two sampling."));

    this->addCommand(
        "initialize",
        makeCommandVoid0(
            *this, &IntegratorEuler::initialize,
            docCommandVoid0(
                "Initialize internal memory from current value of input")));
  }

  virtual ~IntegratorEuler(void) {}

protected:
  std::vector<sigT> inputMemory;
  std::vector<sigT> outputMemory;

  dynamicgraph::SignalTimeDependent<sigT, int> derivativeSOUT;

  double dt;
  double invdt;

public:
  sigT &integrate(sigT &res, int time) {
    sotDEBUG(15) << "# In {" << std::endl;

    sigT sum;
    sigT tmp1, tmp2;
    const std::vector<coefT> &num = numerator;
    const std::vector<coefT> &denom = denominator;

    // Step 1
    tmp1 = inputMemory[0];
    inputMemory[0] = SIN.access(time);
    sum = num[0] * inputMemory[0];
    // End of step 1. Here, sum is b_0 X

    // Step 2
    int numsize = (int)num.size();
    for (int i = 1; i < numsize; ++i) {
      tmp2 = inputMemory[i - 1] - tmp1;
      tmp2 *= invdt;
      tmp1 = inputMemory[i];
      inputMemory[i] = tmp2;
      sum += (num[i] * inputMemory[i]);
    }
    // End of step 2. Here, sum is b_m * d(m)X / dt^m + ... + b_0 X

    // Step 3
    int denomsize = (int)denom.size() - 1;
    for (int i = 0; i < denomsize; ++i) {
      sum -= (denom[i] * outputMemory[i]);
    }
    // End of step 3. Here, sum is b_m * d(m)X / dt^m + ... + b_0 X
    //                           - a_0 Y - ... a_n-1 d(n-1)Y / dt^(n-1)

    // Step 4
    outputMemory[denomsize] = sum;
    for (int i = denomsize - 1; i >= 0; --i) {
      outputMemory[i] += (outputMemory[i + 1] * dt);
    }
    // End of step 4. The ODE is integrated

    inputMemory[0] = SIN.access(time);
    res = outputMemory[0];

    sotDEBUG(15) << "# Out }" << std::endl;
    return res;
  }

  sigT &derivative(sigT &res, int time) {
    if (outputMemory.size() < 2)
      throw dynamicgraph::ExceptionSignal(
          dynamicgraph::ExceptionSignal::GENERIC,
          "Integrator does not compute the derivative.");

    SOUT.recompute(time);
    res = outputMemory[1];
    return res;
  }

  void setSamplingPeriod(const double &period) {
    dt = period;
    invdt = 1 / period;
  }

  double getSamplingPeriod() const { return dt; }

  void initialize() {
    if (denominator.empty() || numerator.empty())
      throw dynamicgraph::ExceptionSignal(
          dynamicgraph::ExceptionSignal::GENERIC,
          "The numerator or the denominator is empty.");

    // Check that denominator.back is the identity
    if (!internal::integratorEulerCoeffIsIdentity(denominator.back()))
      throw dynamicgraph::ExceptionSignal(
          dynamicgraph::ExceptionSignal::GENERIC,
          "The coefficient of the highest order derivative of denominator "
          "should be 1 (the last pushDenomCoef should be the identity).");

    std::size_t numsize = numerator.size();
    inputMemory.resize(numsize);

    inputMemory[0] = SIN.accessCopy();
    for (std::size_t i = 1; i < numsize; ++i) {
      inputMemory[i] = inputMemory[0];
    }

    std::size_t denomsize = denominator.size();
    outputMemory.resize(denomsize);
    for (std::size_t i = 0; i < denomsize; ++i) {
      outputMemory[i] = inputMemory[0];
    }
  }
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif
