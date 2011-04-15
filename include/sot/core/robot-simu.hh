/*
 * Copyright 2010,
 * Nicolas Mansard, Olivier Stasse, François Bleibel, Florent Lamiraux
 *
 * CNRS
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

#ifndef DYNAMICGRAPH_SOT_ROBOT_SIMU_HH
#define DYNAMICGRAPH_SOT_ROBOT_SIMU_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* -- MaaL --- */
#include <jrl/mal/boost.hh>
namespace ml= maal::boost;
/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include "sot/core/device.hh"
#include "sot/core/api.hh"

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (robot_simu_EXPORTS)
#    define SOT_ROBOT_SIMU_EXPORT __declspec(dllexport)
#  else
#    define SOT_ROBOT_SIMU_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOT_ROBOT_SIMU_EXPORT
#endif

namespace dynamicgraph {
  namespace sot {

    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    class SOT_ROBOT_SIMU_EXPORT RobotSimu
      :public Device
    {
    public:
      RobotSimu(const std::string& inName);
      static const std::string CLASS_NAME;
      virtual const std::string& getClassName(void) const {
	return CLASS_NAME;
      }
    };
  } // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef DYNAMICGRAPH_SOT_ROBOT_SIMU_HH */




