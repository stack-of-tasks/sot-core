/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Gepetto, LAAS, CNRS, 2009
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotMemoryTaskSOT.cpp
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


#include <sot-core/memory-task-sot.h>
#include <sot-core/sotDebug.h>


const std::string sotMemoryTaskSOT::CLASS_NAME = "MemoryTaskSOT";


sotMemoryTaskSOT::
sotMemoryTaskSOT( const std::string & name
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
  initMemory( nJ,mJ,ffsize );
}


void sotMemoryTaskSOT::
initMemory( const unsigned int nJ,const unsigned int mJ,const unsigned int ffsize )
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

   JK.fill(0);
   Jt.pseudoInverse(Jp);
 }


 void sotMemoryTaskSOT::
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

void sotMemoryTaskSOT::
display( std::ostream& os ) const {} //TODO

