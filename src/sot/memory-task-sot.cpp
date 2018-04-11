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

#include <sot/core/memory-task-sot.hh>
#include <sot/core/debug.hh>
#include <sot/core/matrix-svd.hh>
using namespace dynamicgraph::sot;
using namespace dynamicgraph;


const std::string MemoryTaskSOT::CLASS_NAME = "MemoryTaskSOT";


MemoryTaskSOT::
MemoryTaskSOT( const std::string & name
                  ,const unsigned int nJ
                  ,const unsigned int mJ
                  ,const unsigned int ffsize )
    :
  Entity( name )
  ,jacobianInvSINOUT( "sotTaskAbstract("+name+")::inout(matrix)::Jinv" )
  ,jacobianConstrainedSINOUT( "sotTaskAbstract("+name+")::inout(matrix)::JK" )
  ,jacobianProjectedSINOUT( "sotTaskAbstract("+name+")::inout(matrix)::Jt" )
  ,singularBaseImageSINOUT( "sotTaskAbstract("+name+")::inout(matrix)::V" )
  ,rankSINOUT( "sotTaskAbstract("+name+")::inout(matrix)::rank" )
{
  signalRegistration(jacobianInvSINOUT
                     <<singularBaseImageSINOUT<<rankSINOUT
                     <<jacobianConstrainedSINOUT<<jacobianProjectedSINOUT);
  initMemory( nJ,mJ,ffsize,true );
}


void MemoryTaskSOT::
initMemory( const unsigned int nJ,const unsigned int mJ,const unsigned int ffsize,
	    bool atConstruction )
{
   sotDEBUG(15) << "Task-mermory " << getName() << ": resize " 
                << nJ << "x" << mJ << std::endl;

   Jt.resize( nJ,mJ );
   Jp.resize( mJ,nJ );
   PJp.resize( mJ,nJ );

   Jff.resize( nJ,ffsize );
   Jact.resize( nJ,mJ );

   JK.resize( nJ,mJ );
   U.resize( nJ,nJ );
   V.resize( mJ,mJ );
   S.resize( std::min( nJ,mJ ) );

   svd = SVD_t (nJ, mJ, Eigen::ComputeThinU | Eigen::ComputeThinV);

   JK.fill(0);
   if (atConstruction) {
     Jt.setZero();
     Jp.setZero();
     PJp.setZero();
     Jff.setZero();
     Jact.setZero();
     JK.setZero();
     U.setZero();
     V.setZero();
     S.setZero();
   } else {
     Eigen::pseudoInverse(Jt,Jp);
   }
 }


 void MemoryTaskSOT::
 commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
              std::ostream& os )
 {
   if( cmdLine=="help" )
     {
       os << "TaskAbstract: " << std::endl
          << " - initMemory <nbJoints> <dimTask> [<ffsize>=6]. " << std::endl;
       Entity::commandLine( cmdLine,cmdArgs,os );
     }
   else if ( "initMemory"==cmdLine )
     {
       unsigned int nJ,mJ; cmdArgs >> nJ >> mJ;
       unsigned int ffsize = 6;
       cmdArgs >> std::ws; if( cmdArgs.good() ) cmdArgs >> ffsize;
       initMemory( nJ,mJ,ffsize );
     }
   else
     {
       Entity::commandLine( cmdLine,cmdArgs,os );
     }
 }

void MemoryTaskSOT::
display( std::ostream& /*os*/ ) const {} //TODO

