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


#include <sot/core/task-abstract.hh>
#include "sot/core/api.hh"

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { 
  namespace sot {
    namespace dg = dynamicgraph;
    
    class SOT_CORE_EXPORT MemoryTaskSOT
      : public TaskAbstract::MemoryTaskAbstract, public dg::Entity
    {
    public://   protected:
      /* Internal memory to reduce the dynamic allocation at task resolution. */
      dg::Matrix Jt;  //( nJ,mJ );
      dg::Matrix Jp;
      dg::Matrix PJp;
      
      dg::Matrix Jff; //( nJ,FF_SIZE ); // Free-floating part
      dg::Matrix Jact; //( nJ,mJ );     // Activated part
      dg::Matrix JK; //(nJ,mJ);
      
      dg::Matrix U,V;

      typedef Eigen::JacobiSVD<dg::Matrix> SVD_t;
      SVD_t svd;
      
    public:
      /* mJ is the number of actuated joints, nJ the number of feature in the task,
       * and ffsize the number of unactuated DOF. */
      MemoryTaskSOT( const std::string & name,const unsigned int nJ=0,
		     const unsigned int mJ=0,const unsigned int ffsize =0 );
      
      virtual void initMemory( const unsigned int nJ,
			       const unsigned int mJ,
			       const unsigned int ffsize,
			       bool atConstruction = false);
      
    public: /* --- ENTITY INHERITANCE --- */
      static const std::string CLASS_NAME;
      virtual void display( std::ostream& os ) const;
      virtual const std::string& getClassName( void ) const { return CLASS_NAME; }
      
    public: /* --- SIGNALS --- */
      dg::Signal< dg::Matrix,int > jacobianInvSINOUT;
      dg::Signal< dg::Matrix,int > jacobianConstrainedSINOUT;
      dg::Signal< dg::Matrix,int > jacobianProjectedSINOUT;
      dg::Signal< dg::Matrix,int > singularBaseImageSINOUT;
      dg::Signal< unsigned int,int > rankSINOUT;
      
    public: /* --- PARAMS --- */
      virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
				std::ostream& os );
    };
    
  } /* namespace sot */
} /* namespace dynamicgraph */

#endif // __SOT_MEMORY_TASK_HH
