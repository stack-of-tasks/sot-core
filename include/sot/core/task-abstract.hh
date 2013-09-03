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

#ifndef __SOT_TASKABSTRACT_H__
#define __SOT_TASKABSTRACT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* STD */
#include <string>

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/all-signals.h>
#include <sot/core/multi-bound.hh>
#include "sot/core/api.hh"
#include <dynamic-graph/linear-algebra.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
  namespace sot {
    namespace dg = dynamicgraph;

    /// Hierarchical element of the stack of tasks.
    ///
    /// A task computes a value and a Jacobian as output signals.
    /// Once stacked into a solver, the solver will compute the control
    /// vector that makes the task values converge toward zero in the 
    /// order defined by the priority levels.
    ///
    /// \image html pictures/task.png "Task diagram: Task types derive from TaskAbstract. The value and Jacobian of a Task are computed from the features that are stored in the task.

    class SOT_CORE_EXPORT TaskAbstract
      : public dg::Entity
    {
    public:

      /* Use a derivative of this class to store computational memory. */
      class MemoryTaskAbstract
      {
      public:
	int timeLastChange;
      public:
	MemoryTaskAbstract( void ) : timeLastChange(0) {};
	virtual ~MemoryTaskAbstract( void ) {};
      public:
	virtual void commandLine( const std::string& cmdLine
				  ,std::istringstream& cmdArgs
				  ,std::ostream& os ) = 0;
	virtual void display( std::ostream& os ) const = 0;
	friend std::ostream& operator<<( std::ostream& os,
					 const MemoryTaskAbstract& tcm )
	{tcm.display(os); return os;}
      };

    public:
      MemoryTaskAbstract * memoryInternal;

    protected:
      void taskRegistration( void );

    public:
      TaskAbstract( const std::string& n );

    public: /* --- SIGNALS --- */

      dg::SignalTimeDependent< VectorMultiBound,int > taskSOUT;
      dg::SignalTimeDependent< dg::Matrix,int > jacobianSOUT;

    public: /* --- PARAMS --- */
      virtual void commandLine( const std::string& cmdLine
				,std::istringstream& cmdArgs
				,std::ostream& os ) ;
    };

  } /* namespace sot */
} /* namespace dynamicgraph */


#endif /* #ifndef __SOT_TASKABSTRACT_H__ */


