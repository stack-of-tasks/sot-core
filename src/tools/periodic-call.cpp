/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
#include <algorithm>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/exception-factory.h>
#include <dynamic-graph/pool.h>
#include <sot/core/debug.hh>
#include <sot/core/exception-tools.hh>
#include <sot/core/periodic-call.hh>

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

PeriodicCall::PeriodicCall(void) : signalMap(), innerTime(0) {}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void PeriodicCall::addSignal(const std::string &name, SignalBase<int> &sig) {
  signalMap[name] = SignalToCall(&sig);
  return;
}

void PeriodicCall::addSignal(const std::string &sigpath) {
  istringstream sigISS(sigpath);
  SignalBase<int> &signal =
      ::dynamicgraph::PoolStorage::getInstance()->getSignal(sigISS);
  addSignal(sigpath, signal);
  return;
}

void PeriodicCall::addDownsampledSignal(
    const std::string &name, SignalBase<int> &sig,
    const unsigned int &downsamplingFactor) {
  signalMap[name] = SignalToCall(&sig, downsamplingFactor);
  return;
}

void PeriodicCall::addDownsampledSignal(
    const std::string &sigpath, const unsigned int &downsamplingFactor) {
  istringstream sigISS(sigpath);
  SignalBase<int> &signal =
      ::dynamicgraph::PoolStorage::getInstance()->getSignal(sigISS);
  addDownsampledSignal(sigpath, signal, downsamplingFactor);
  return;
}

void PeriodicCall::rmSignal(const std::string &name) {
  signalMap.erase(name);
  return;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void PeriodicCall::runSignals(const int &t) {
  for (SignalMapType::iterator iter = signalMap.begin();
       signalMap.end() != iter; ++iter) {
    if (t % iter->second.downsamplingFactor == 0)
      (*iter).second.signal->recompute(t);
  }
  return;
}

void PeriodicCall::run(const int &t) {
  runSignals(t);
  return;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void PeriodicCall::display(std::ostream &os) const {
  os << "  (t=" << innerTime << ")" << endl;

  os << " -> SIGNALS:" << endl;
  for (SignalMapType::const_iterator iter = signalMap.begin();
       signalMap.end() != iter; ++iter) {
    os << " - " << (*iter).first << endl;
  }
}

/*
static std::string readLineStr( istringstream& args )
{
  stringbuf* pbuf=args.rdbuf();
  const std::streamsize size = pbuf->in_avail();
  char * buffer = new char[ size+1 ];
  pbuf->sgetn( buffer,size );

  buffer[size]='\0';
  std::string res( buffer );
  delete [] buffer;
  return res;
}
*/

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
