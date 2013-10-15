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

#ifndef __SOT_WSOT_HH
#define __SOT_WSOT_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* Classes standards. */
#include <list>                    /* Classe std::list   */

/* SOT */
#include <sot/core/sot.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (weighted_sot_EXPORTS)
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
  \class dynamicgraph::sot::WeightedSot
  \brief This class implements the Stack of Task.
  It allows to deal with the priority of the controllers
  through the shell. The controllers can be either constraints
  either tasks.


*/

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

class SOTWEIGHTEDSOT_CORE_EXPORT WeightedSot
:public Sot
{
 public:
  /*! \brief Specify the name of the class entity. */
  static const std::string CLASS_NAME;
  /*! \brief Returns the name of this class. */
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  /*! \brief Default constructor */
  WeightedSot( const std::string& name );

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
  dg::SignalPtr<ml::Matrix,int> weightSIN;
  dg::SignalTimeDependent<ml::Matrix,int> constrainedWeightSOUT;
  dg::SignalPtr<ml::Matrix,int> constrainedWeightSIN;
  dg::SignalTimeDependent<ml::Matrix,int> squareRootInvWeightSOUT;
  dg::SignalPtr<ml::Matrix,int> squareRootInvWeightSIN;
  /*! @} */

};

} /* namespace sot */} /* namespace dynamicgraph */



#endif /* #ifndef __SOT_SOT_HH */




