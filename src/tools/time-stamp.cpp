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

/* SOT */
#include <dynamic-graph/factory.h>
#include <sot/core/macros-signal.hh>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/time-stamp.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TimeStamp, "TimeStamp");

/* --- CONSTRUCTION ---------------------------------------------------- */
TimeStamp::TimeStamp(const std::string &name)
    : Entity(name), offsetValue(0), offsetSet(false),
      timeSOUT("TimeStamp(" + name + ")::output(vector2)::time"),
      timeDoubleSOUT("TimeStamp(" + name + ")::output(double)::timeDouble"),
      timeOnceSOUT(boost::bind(&TimeStamp::getTimeStamp, this, _1, _2),
                   sotNOSIGNAL,
                   "TimeStamp(" + name + ")::output(vector2)::synchro"),
      timeOnceDoubleSOUT(
          boost::bind(&TimeStamp::getTimeStampDouble, this,
                      SOT_CALL_SIG(timeSOUT, dynamicgraph::Vector), _1),
          timeSOUT, "TimeStamp(" + name + ")::output(double)::synchroDouble") {
  sotDEBUGIN(15);

  timeSOUT.setFunction(boost::bind(&TimeStamp::getTimeStamp, this, _1, _2));
  timeDoubleSOUT.setFunction(
      boost::bind(&TimeStamp::getTimeStampDouble, this,
                  SOT_CALL_SIG(timeSOUT, dynamicgraph::Vector), _1));
  timeOnceSOUT.setNeedUpdateFromAllChildren(true);
  timeOnceDoubleSOUT.setNeedUpdateFromAllChildren(true);
  signalRegistration(timeSOUT << timeDoubleSOUT << timeOnceSOUT
                              << timeOnceDoubleSOUT);

  gettimeofday(&val, NULL);

  sotDEBUGOUT(15);
}

/* --- DISPLAY --------------------------------------------------------- */
void TimeStamp::display(std::ostream &os) const {
  os << "TimeStamp <> : " << val.tv_sec << "s; " << val.tv_usec << "us."
     << std::endl;
}

/* --------------------------------------------------------------------- */
/* --- CONTROL --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

dynamicgraph::Vector &TimeStamp::getTimeStamp(dynamicgraph::Vector &res,
                                              const int & /*time*/) {
  sotDEBUGIN(15);
  gettimeofday(&val, NULL);
  if (res.size() != 2)
    res.resize(2);

  res(0) = static_cast<double>(val.tv_sec);
  res(1) = static_cast<double>(val.tv_usec);
  sotDEBUGOUT(15);
  return res;
}

double &TimeStamp::getTimeStampDouble(const dynamicgraph::Vector &vect,
                                      double &res) {
  sotDEBUGIN(15);

  if (offsetSet)
    res = (vect(0) - offsetValue) * 1000;
  else
    res = vect(0) * 1000;
  res += vect(1) / 1000;
  sotDEBUGOUT(15);
  return res;
}
