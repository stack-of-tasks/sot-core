/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2009
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotCOMFreezer.h
 * Project:   SOT
 * Author:    Pierre Gergondet
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifndef __SOT_SOTCOMFREEZER_H_H
#define __SOT_SOTCOMFREEZER_H_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

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


namespace sot {

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
    ml::Vector m_lastCoM;
    bool m_previousPGInProcess;
    int m_lastStopTime;

  public: /* --- CONSTRUCTION --- */
    CoMFreezer(const std::string & name);
    virtual ~CoMFreezer(void);

  public: /* --- SIGNAL --- */
    dg::SignalPtr<ml::Vector, int> CoMRefSIN;
    dg::SignalPtr<unsigned, int>  PGInProcessSIN;
    dg::SignalTimeDependent<ml::Vector, int> freezedCoMSOUT;

  public: /* --- FUNCTION --- */
    ml::Vector& computeFreezedCoM(ml::Vector & freezedCoM, const int& time);

  public: /* --- PARAMS --- */
    virtual void display(std::ostream & os) const;
};



} // namespace sot



#endif /* #ifndef __SOT_SOTCOMFREEZER_H_H */
