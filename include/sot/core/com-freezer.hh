/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SOTCOMFREEZER_H_H
#define __SOT_SOTCOMFREEZER_H_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (com_freezer_EXPORTS)
#    define SOTCOMFREEZER_EXPORT __declspec(dllexport)
#  else
#    define SOTCOMFREEZER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTCOMFREEZER_EXPORT
#endif


namespace dynamicgraph { namespace sot {

namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTCOMFREEZER_EXPORT CoMFreezer
: public dg::Entity
{
  public:
    static const std::string CLASS_NAME;
    virtual const std::string & getClassName() const { return CLASS_NAME; }

  private:
    dg::Vector m_lastCoM;
    bool m_previousPGInProcess;
    int m_lastStopTime;

  public: /* --- CONSTRUCTION --- */
    CoMFreezer(const std::string & name);
    virtual ~CoMFreezer(void);

  public: /* --- SIGNAL --- */
    dg::SignalPtr<dg::Vector, int> CoMRefSIN;
    dg::SignalPtr<unsigned, int>  PGInProcessSIN;
    dg::SignalTimeDependent<dg::Vector, int> freezedCoMSOUT;

  public: /* --- FUNCTION --- */
    dg::Vector& computeFreezedCoM(dg::Vector & freezedCoM, const int& time);

  public: /* --- PARAMS --- */
    virtual void display(std::ostream & os) const;
};



} /* namespace sot */} /* namespace dynamicgraph */



#endif /* #ifndef __SOT_SOTCOMFREEZER_H_H */
