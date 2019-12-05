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
#define ADD_COMMAND(name, def)                                                 \
  if (commandMap.count(prefix + name) != 0) {                                  \
    DG_THROW ExceptionFactory(ExceptionFactory::OBJECT_CONFLICT,               \
                              "Command " + prefix + name +                     \
                                  " already registered in Entity.");           \
  }                                                                            \
  commandMap.insert(std::make_pair(prefix + name, def))

void PeriodicCall::addSpecificCommands(Entity &ent,
                                       Entity::CommandMap_t &commandMap,
                                       const std::string &prefix) {
  using namespace dynamicgraph::command;

  /* Explicit typage to help the compiler. */
  boost::function<void(const std::string &)>
      addSignal = boost::bind(&PeriodicCall::addSignal, this, _1),
      rmSignal = boost::bind(&PeriodicCall::rmSignal, this, _1);
  boost::function<void(const std::string &, const unsigned int &)>
      addDownsampledSignal =
          boost::bind(&PeriodicCall::addDownsampledSignal, this, _1, _2);
  boost::function<void(void)> clear = boost::bind(&PeriodicCall::clear, this);
  boost::function<void(std::ostream &)> disp =
      boost::bind(&PeriodicCall::display, this, _1);

  ADD_COMMAND(
      "addSignal",
      makeCommandVoid1(ent, addSignal,
                       docCommandVoid1("Add the signal to the refresh list",
                                       "string (sig name)")));
  ADD_COMMAND("addDownsampledSignal",
              makeCommandVoid2(
                  ent, addDownsampledSignal,
                  docCommandVoid2(
                      "Add the signal to the refresh list", "string (sig name)",
                      "unsigned int (downsampling factor, 1 means every time, "
                      "2 means every other time, etc...")));
  ADD_COMMAND(
      "rmSignal",
      makeCommandVoid1(ent, rmSignal,
                       docCommandVoid1("Remove the signal to the refresh list",
                                       "string (sig name)")));
  ADD_COMMAND(
      "clear",
      makeCommandVoid0(
          ent, clear,
          docCommandVoid0(
              "Clear all signals and commands from the refresh list.")));

  ADD_COMMAND("disp",
              makeCommandVerbose(
                  ent, disp,
                  docCommandVerbose(
                      "Print the list of to-refresh signals and commands.")));
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
