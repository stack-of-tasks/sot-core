/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotWeightedSOT.h
 * Project:   SOT
 * Author:    Nicolas Mansard
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



#ifndef __SOT_WSOT_HH
#define __SOT_WSOT_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* Classes standards. */
#include <list>                    /* Classe std::list   */

/* SOT */
#include <sot-core/sotSOT.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (sotWeightedSOT_CORE_EXPORTS)
#    define SOTWEIGHTEDSOT_CORE_EXPORT __declspec(dllexport)
#  else
#    define SOTWEIGHTEDSOT_CORE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTWEIGHTEDSOT_CORE_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/*! @ingroup stackoftasks
  \brief This class implements the Stack of Task.
  It allows to deal with the priority of the controllers
  through the shell. The controllers can be either constraints
  either tasks.


*/
class SOTWEIGHTEDSOT_CORE_EXPORT sotWeightedSOT
:public sotSOT
{
 public:
  /*! \brief Specify the name of the class entity. */
  static const std::string CLASS_NAME;
  /*! \brief Returns the name of this class. */
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  /*! \brief Default constructor */
  sotWeightedSOT( const std::string& name );

 public: /* --- CONTROL --- */

  /*! \name Methods to compute the control law following the
   recursive definition of the stack of tasks.
   @{
  */

  /*! \brief Compute the control law using weighted inverse. */
  ml::Matrix& computeSquareRootInvWeight( ml::Matrix& res,const int& time );
  ml::Vector& computeWeightedControlLaw( ml::Vector& control,const int& time );
  ml::Matrix& computeConstrainedWeight( ml::Matrix& KAK,const int& time );

  /*! @} */

 public: /* --- SIGNALS --- */

  /*! \name Methods to handle signals
    @{
   */
  SignalPtr<ml::Matrix,int> weightSIN;
  SignalTimeDependant<ml::Matrix,int> constrainedWeightSOUT;
  SignalPtr<ml::Matrix,int> constrainedWeightSIN;
  SignalTimeDependant<ml::Matrix,int> squareRootInvWeightSOUT;
  SignalPtr<ml::Matrix,int> squareRootInvWeightSIN;
  /*! @} */

};





#endif /* #ifndef __SOT_SOT_HH */




