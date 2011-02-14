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

/* Matrix */
#include <jrl/mal/boost.hh>
#include <jrl/mal/boostmatrixsvd.hh>
namespace ml = maal::boost;

/* STD */
#include <string>

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/multi-bound.h>
#include "sot/core/api.hh"

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

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
    friend std::ostream&
      operator<<( std::ostream& os,const MemoryTaskAbstract& tcm )
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
  dg::SignalTimeDependent< ml::Matrix,int > jacobianSOUT;
  dg::SignalTimeDependent< ml::Vector,int > featureActivationSOUT;

 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine
			    ,std::istringstream& cmdArgs
			    ,std::ostream& os ) ;
 public:
};

} /* namespace sot */} /* namespace dynamicgraph */





#endif /* #ifndef __SOT_TASKABSTRACT_H__ */


