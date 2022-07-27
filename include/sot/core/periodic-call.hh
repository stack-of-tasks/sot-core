/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_PERIODICCALL_HH__
#define __SOT_PERIODICCALL_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-base.h>

#include <sot/core/api.hh>
/* STD */
#include <list>
#include <map>
#include <string>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/*!
  \class PeriodicCall
*/
class SOT_CORE_EXPORT PeriodicCall {
 protected:
  struct SignalToCall {
    dynamicgraph::SignalBase<int> *signal;
    unsigned int downsamplingFactor;

    SignalToCall() {
      signal = NULL;
      downsamplingFactor = 1;
    }

    SignalToCall(dynamicgraph::SignalBase<int> *s, unsigned int df = 1) {
      signal = s;
      downsamplingFactor = df;
    }
  };

  typedef std::map<std::string, SignalToCall> SignalMapType;
  SignalMapType signalMap;

  int innerTime;

  /* --- FUNCTIONS ------------------------------------------------------------
   */
 public:
  PeriodicCall(void);
  virtual ~PeriodicCall(void) {}

  void addDownsampledSignal(const std::string &name,
                            dynamicgraph::SignalBase<int> &sig,
                            const unsigned int &downsamplingFactor);
  void addDownsampledSignal(const std::string &sigpath,
                            const unsigned int &downsamplingFactor);

  void addSignal(const std::string &name, dynamicgraph::SignalBase<int> &sig);
  void addSignal(const std::string &args);
  void rmSignal(const std::string &name);

  void runSignals(const int &t);
  void run(const int &t);

  void clear(void) { signalMap.clear(); }

  void display(std::ostream &os) const;
};

}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __SOT_PERIODICCALL_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
