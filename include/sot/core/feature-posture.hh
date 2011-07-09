/*
 * Copyright 2010,
 * Florent Lamiraux
 * Thomas Moulard,
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

#ifndef SOT_CORE_FEATURE_POSTURE_HH
#define SOT_CORE_FEATURE_POSTURE_HH

#include "sot/core/api.hh"
#include "sot/core/feature-abstract.hh"
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_posture_EXPORTS)
#    define SOTFEATUREPOSTURE_EXPORT __declspec(dllexport)
#  else
#    define SOTFEATUREPOSTURE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFEATUREPOSTURE_EXPORT
#endif

namespace dynamicgraph {
  namespace sot {
    using command::Command;
    using command::Value;
      
    /* Feature that observes the posture of the robot, ie whose Jacobian is the
     * identity, or slices of the identity. This feature can be exactly
     * obtained with a generic posture, given the identity matrix as the input
     * Jacobian, the identity matrix. It is even prefereable, as the reference
     * value is then given by a signal, which can be reevalutated at each
     * iteration, for example to track a reference trajectory in the
     * configuration space. See for example the toFlag python function in the
     * sot-dyninv module to nicely selec the posture DOF.
     */

    class SOTFEATUREPOSTURE_EXPORT FeaturePosture
      : public FeatureAbstract
    {
      class SelectDof;	
      friend class SelectDof;

      DYNAMIC_GRAPH_ENTITY_DECL();

    public:
      typedef dynamicgraph::SignalPtr<ml::Vector, int> signalIn_t;
      typedef dynamicgraph::SignalTimeDependent<ml::Vector, int> signalOut_t;
	
      DECLARE_NO_REFERENCE;
	
      explicit FeaturePosture (const std::string& name);
      virtual ~FeaturePosture ();
      virtual unsigned int& getDimension( unsigned int& res,int );
      void setPosture (const ml::Vector& posture);
      void selectDof (unsigned dofId, bool control);

    protected:

      virtual ml::Vector& computeError( ml::Vector& res, int );
      virtual ml::Matrix& computeJacobian( ml::Matrix& res, int );
      virtual ml::Vector& computeActivation( ml::Vector& res, int );

      signalIn_t state_;
      signalOut_t error_;
      ml::Vector posture_;
      ml::Matrix jacobian_;
    }; // class FeaturePosture
  } // namespace sot
} // namespace dynamicgraph

#endif //SOT_CORE_FEATURE_POSTURE_HH
