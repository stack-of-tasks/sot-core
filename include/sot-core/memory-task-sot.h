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

#ifndef __SOT_MEMORY_TASK_HH
#define __SOT_MEMORY_TASK_HH


#include <sot-core/task-abstract.h>
#include "sot/core/api.hh"

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

class SOT_CORE_EXPORT MemoryTaskSOT
: public TaskAbstract::MemoryTaskAbstract, public dg::Entity
{
 public://   protected:
  /* Internal memory to reduce the dynamic allocation at task resolution. */
  ml::MatrixSvd Jt;  //( nJ,mJ );
  ml::Matrix Jp;
  ml::Matrix PJp;

  ml::Matrix Jff; //( nJ,FF_SIZE ); // Free-floating part
  ml::Matrix Jact; //( nJ,mJ );     // Activated part
  ml::Matrix JK; //(nJ,mJ);

  ml::Matrix U,V;
  ml::Vector S;

 public:
  /* mJ is the number of actuated joints, nJ the number of feature in the task,
   * and ffsize the number of unactuated DOF. */
  MemoryTaskSOT( const std::string & name,const unsigned int nJ=0,
                    const unsigned int mJ=0,const unsigned int ffsize =0 );

  virtual void initMemory( const unsigned int nJ,
                           const unsigned int mJ,
                           const unsigned int ffsize );

 public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display( std::ostream& os ) const;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public: /* --- SIGNALS --- */
  dg::Signal< ml::Matrix,int > jacobianInvSINOUT;
  dg::Signal< ml::Matrix,int > jacobianConstrainedSINOUT;
  dg::Signal< ml::Matrix,int > jacobianProjectedSINOUT;
  dg::Signal< ml::Matrix,int > singularBaseImageSINOUT;
  dg::Signal< unsigned int,int > rankSINOUT;

 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
			    std::ostream& os );
};

} /* namespace sot */} /* namespace dynamicgraph */

#endif // __SOT_MEMORY_TASK_HH
