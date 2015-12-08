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

#ifndef SOT_FEATURE_JOINTLIMITS_HH
# define SOT_FEATURE_JOINTLIMITS_HH
// Matrix
# include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

// SOT
# include <dynamic-graph/entity.h>
# include <sot/core/exception-task.hh>
# include <dynamic-graph/all-signals.h>

#if defined (WIN32)
#  if defined (joint_limitator_EXPORTS)
#    define SOTJOINTLIMITATOR_EXPORT __declspec(dllexport)
#  else
#    define SOTJOINTLIMITATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTJOINTLIMITATOR_EXPORT
#endif

namespace dynamicgraph {
  namespace sot {
    namespace dg = dynamicgraph;

    /// \brief Filter control vector to avoid exceeding joint maximum values.
    ///
    /// This must be plugged between the entity producing the command
    /// (i.e. usually the sot) and the entity executing it (the device).
    class SOTJOINTLIMITATOR_EXPORT JointLimitator
      : public dg::Entity
    {
      DYNAMIC_GRAPH_ENTITY_DECL ();

    public:
      JointLimitator (const std::string& name);
      virtual ~JointLimitator ()
      {}

      virtual dg::Vector& computeControl (dg::Vector& res, int time);
      dg::Vector& computeWidthJl (dg::Vector& res, const int& time);

      virtual void display (std::ostream& os) const;

      /// \name Signals
      /// \{
      dg::SignalPtr< dg::Vector,int > jointSIN;
      dg::SignalPtr< dg::Vector,int > upperJlSIN;
      dg::SignalPtr< dg::Vector,int > lowerJlSIN;
      dg::SignalPtr< dg::Vector,int > controlSIN;
      dg::SignalTimeDependent< dg::Vector,int > controlSOUT;
      dg::SignalTimeDependent< dg::Vector,int > widthJlSINTERN;
      /// \}
    };
  } // end of namespace sot.
} // end of namespace dynamic-graph.

#endif //! SOT_FEATURE_JOINTLIMITS_HH
