/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SOT_SMOOTHREACH_H_H
#define __SOT_SMOOTHREACH_H_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (com_freezer_EXPORTS)
#    define SOTSMOOTHREACH_EXPORT __declspec(dllexport)
#  else
#    define SOTSMOOTHREACH_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTSMOOTHREACH_EXPORT
#endif


namespace dynamicgraph { namespace sot {

namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTSMOOTHREACH_EXPORT SmoothReach
: public dg::Entity
{
  public:
    static const std::string CLASS_NAME;
    virtual const std::string & getClassName() const { return CLASS_NAME; }

  private:
    ml::Vector start,goal;
    int startTime, lengthTime;
    bool isStarted, isParam;
    int smoothMode; double smoothParam;

    double smoothFunction( double x );

  public: /* --- CONSTRUCTION --- */
    SmoothReach(const std::string & name);
    virtual ~SmoothReach(void) {};

  public: /* --- SIGNAL --- */
    dg::SignalPtr<ml::Vector, int> startSIN;
    dg::SignalTimeDependent<ml::Vector, int> goalSOUT;
    //dg::SignalTimeDependent<double, int> percentSOUT;

  public: /* --- FUNCTION --- */
    ml::Vector& goalSOUT_function(ml::Vector & goal, const int& time);

    void set( const ml::Vector & goal, const int & length );
    const ml::Vector & getGoal( void );
    const int & getLength( void );
    const int & getStart( void );

    void setSmoothing( const int & mode, const double & param );

  public: /* --- PARAMS --- */
    virtual void display(std::ostream & os) const;
    void initCommands( void );
};



} /* namespace sot */} /* namespace dynamicgraph */



#endif /* #ifndef __SOT_SMOOTHREACH_H_H */
