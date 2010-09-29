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

/* SOT */
#define WITH_CHRONO
//#define VP_DEBUG
//#define VP_DEBUG_MODE 45
#include <sot-core/debug.h>
#include <sot-core/sot-h.h>
#include <sot-core/pool.h>
#include <sot-core/task.h>
#include <sot-core/task-unilateral.h>
#include <sot-core/factory.h>

using namespace std;
using namespace sot;
using namespace dynamicgraph;

#ifdef VP_DEBUG
class sotSOTH__INIT
{
public:sotSOTH__INIT( void ) { sot::DebugTrace::openFile(); }
};

sotSOTH__INIT sotSOTH_initiator;
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SotH,"SOTH");


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

SotH::
SotH( const std::string& name )
  :Sot(name),Qh(nbJoints),Rh(nbJoints,nbJoints),constraintH()
  ,solverNorm(nbJoints,Qh,Rh,constraintH),solverPrec(NULL)
  ,fillMemorySignal(false)
{
}


SotH::
~SotH( void )
{



}


void SotH::
defineNbDof( const unsigned int& nbDof )
{
  sotDEBUGIN(15);
  Sot::defineNbDof(nbDof);
  for(StackType::iterator iter=stack.begin();stack.end()!=iter;++iter )
    {
      MemoryTaskSOTH * mem
        = dynamic_cast<MemoryTaskSOTH *>( (*iter)->memoryInternal );
      if(NULL!=mem) mem->solver.setNbDof(nbDof);
    }
  solverNorm.setNbDof(nbDof-(ffJointIdLast-ffJointIdFirst));
  sotDEBUGOUT(15);
}

void SotH::
commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
	     std::ostream& os )
{
  sotDEBUGIN(15);

  if( cmdLine == "help")
    {
      os << "Stack of Tasks Inequality: "<<endl
	 << " - fillMemorySignal [boolean] "<<endl;
      Sot::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine == "fillMemorySignal")
    {
      cmdArgs >> std::ws;
      if( cmdArgs.good() )
	{  cmdArgs >> fillMemorySignal; }
      else
	{ os << "fillMemorySignal = " << ((fillMemorySignal)?"true":"false"); }
    }
 else
   {
     Sot::commandLine( cmdLine,cmdArgs,os );
   }


  sotDEBUGOUT(15);
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void buildTaskVectors( const VectorMultiBound& err,
                       const ml::Matrix & JK,
                       bubVector & ee,bubVector & eiinf,bubVector & eisup,
                       ConstraintMem::BoundSideVector& bounds,
                       bubMatrix & Je,bubMatrix & Ji )
{
  const unsigned int nJ = JK.nbCols();

  sotDEBUG(25) << "/* Compute the task sizes. */"<< std::endl;
  unsigned int sizei=0,sizee=0;
  for( VectorMultiBound::const_iterator iter=err.begin();
       err.end()!=iter;++iter )
    { if( iter->getMode()==MultiBound::MODE_SINGLE )
        sizee++; else sizei++; }

  Ji.resize(sizei,nJ); eiinf.resize(sizei),eisup.resize(sizei);
  bounds.resize(sizei);
  Je.resize(sizee,nJ); ee.resize(sizee);
  sotDEBUG(25) << "Size e=" << sizee << " i=" << sizei << std::endl;

  sotDEBUG(25) << "/* Build the task vectors. */"<< std::endl;
  for( int i=err.size()-1;i>=0;--i )
    {
    sotDEBUG(25) << "i = "<<i<< std::endl;
    switch( err[i].getMode() )
        {
        case MultiBound::MODE_SINGLE:
          {
            ee(--sizee) = err[i].getSingleBound();
            for( unsigned int j=0;j<nJ;++j )
              { Je(sizee,j) = JK(i,j); }
            break;
          }
        case MultiBound::MODE_DOUBLE:
          {
            --sizei;
            bounds[sizei] = ConstraintMem::BOUND_VOID;
            if( err[i].getDoubleBoundSetup( MultiBound::BOUND_INF ) )
              {
                eiinf(sizei) = err[i].getDoubleBound( MultiBound::BOUND_INF );
                bounds[sizei] = ConstraintMem::BOUND_INF;
              }
            if( err[i].getDoubleBoundSetup( MultiBound::BOUND_SUP ) )
              {
                eisup(sizei) = err[i].getDoubleBound( MultiBound::BOUND_SUP );
                if( bounds[sizei]==ConstraintMem::BOUND_INF )
                  bounds[sizei]= ConstraintMem::BOUND_BOTH;
                else bounds[sizei]= ConstraintMem::BOUND_SUP;
              }
            for( unsigned int j=0;j<nJ;++j )
              { Ji(sizei,j) = JK(i,j); }
            break;
          }
        }
    }
  sotDEBUG(25) << "ee = "<<(MATLAB)ee<< std::endl;
  sotDEBUG(25) << "eii = "<<(MATLAB)eiinf<< std::endl;
  sotDEBUG(25) << "eis = "<<(MATLAB)eisup<< std::endl;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#ifdef  WITH_CHRONO
	#ifndef WIN32
	#include <sys/time.h>
	#else /*WIN32*/
	#include <sot-core/utils-windows.h>
	#endif /*WIN32*/
#endif /*WITH_CHRONO*/

#ifdef WITH_CHRONO
#  define SOT_DEFINE_CHRONO                     \
  struct timeval t0,t1;                         \
  double dtsolver;                              \
  std::ofstream foutdt("/tmp/dtsoth.dat")
#  define SOT_INIT_CHRONO                       \
  gettimeofday(&t0,NULL); foutdt<<std::endl<<iterTime<<" "
#  define SOT_CHRONO                                                  \
  gettimeofday(&t1,NULL);                                               \
  dtsolver = (t1.tv_sec-t0.tv_sec) * 1000. + (t1.tv_usec-t0.tv_usec+0.)/1000. ; \
  foutdt << dtsolver <<" "; \
  gettimeofday(&t0,NULL)
#else // ifdef WITH_CHRONO
#  define SOT_DEFINE_CHRONO
#  define SOT_INIT_CHRONO
#  define SOT_CHRONO()
#endif // ifdef WITH_CHRONO


SOT_DEFINE_CHRONO;

ml::Vector& SotH::
computeControlLaw( ml::Vector& control,const int& iterTime )
{
  sotDEBUGIN(15);

  SOT_INIT_CHRONO;

  SolverHierarchicalInequalities::THRESHOLD_ZERO = inversionThresholdSIN(iterTime);
  const ml::Matrix &K = constraintSOUT(iterTime);
  const unsigned int nJ = K.nbCols();

  try
    {
      control = q0SIN( iterTime );
      sotDEBUG(15) << "initial velocity q0 = " << control << endl;
      if( nJ!=control.size() ) { control.resize( nJ ); control.fill(.0); }
    }
  catch (...)
    {
      if( nJ!=control.size() ) { control.resize( nJ ); }
      control.fill(0.);
      sotDEBUG(25) << "No initial velocity." <<endl;
    }
  SOT_CHRONO;

  /* --- Affect solvers --- */
//   if( solvers.size()+1!=stack.size() )
//     {
//       solvers.resize(stack.size()+1,NULL); // TODO: resize affect to default value?
//       for( SolversList::iterator iter = solvers.begin();solvers.end()!=iter;++iter )
//         {
//           if(*iter==NULL)
//             {
//               (*iter)=new SolverHierarchicalInequalities(nJ,Qh,Rh,constraintH);
//             }
//           /* TODO: free memory */
//         }
//     }

  if(Qh.getSize()!=nJ) { Qh.clear(nJ); }
  if((Rh.size1()!=nJ)||(Rh.size2()!=nJ)) Rh.resize(nJ,nJ,false);
  Rh*=0;
  constraintH.clear();
  solverPrec=NULL;
  SOT_CHRONO;

  sotDEBUGF(5, " --- Time %d -------------------", iterTime );
  unsigned int iterTask = 0;
  for( StackType::iterator iter = stack.begin(); iter!=stack.end();++iter,++iterTask )
    {
      sotDEBUGF(5,"Rank %d.",iterTask);
      TaskAbstract & task = **iter;
      sotDEBUG(15) << "Task: e_" << task.getName() << std::endl;
      const ml::Matrix &Jac = task.jacobianSOUT(iterTime);
      const VectorMultiBound &err = task.taskSOUT(iterTime);
      unsigned int ntask=err.size();

      sotDEBUG(25) << "/* Init memory. */" << std::endl;
      MemoryTaskSOTH * mem = dynamic_cast<MemoryTaskSOTH *>( task.memoryInternal );
      if( (NULL==mem)||(mem->referenceKey!=this) )
        {
          if( NULL!=task.memoryInternal ) delete task.memoryInternal;
          mem = new MemoryTaskSOTH(task.getName()+"_memSOTH",
                                      this,nJ,Qh,Rh,constraintH);
          task.memoryInternal = mem;
        }

      sotDEBUG(25) << "/* --- Resize J --- */" << std::endl;
      mem->JK.resize( ntask,nJ );
      mem->Jff.resize( ntask,Jac.nbCols()-nJ );
      mem->Jact.resize( ntask,nJ );

      sotDEBUG(25) << "/* --- COMPUTE JK --- */" << std::endl;
      Sot::computeJacobianConstrained( Jac,K,mem->JK,mem->Jff,mem->Jact );
      if( fillMemorySignal )
        {
          mem->jacobianConstrainedSINOUT = mem->JK;
          mem->jacobianConstrainedSINOUT.setTime( iterTime );
        }

      sotDEBUG(25) << "/* Set initial conditions. */" << std::endl;
      SolverHierarchicalInequalities & solver = mem->solver;
      if( NULL==solverPrec ) solver.setInitialConditionVoid();
      else
        { solver.setInitialCondition( solverPrec->u0,solverPrec->rankh ); }
      solverPrec = &solver;
      SOT_CHRONO;

#ifdef VP_DEBUG
      solver.printDifferentialCondition(sotDEBUGFLOW.outputbuffer);
#endif

      sotDEBUG(25) << "/* Record warm start. */" << std::endl;
      solver.recordInitialConditions();

      sotDEBUG(25) << "/* Effective warm start. */ " << std::endl;
      solver.warmStart();
      SOT_CHRONO;

      sotDEBUG(25) << "/* Build the task vectors. */"<< std::endl;
      bubMatrix Ji; bubVector eiinf,eisup;
      ConstraintMem::BoundSideVector bounds;
      bubMatrix Je; bubVector ee;
      buildTaskVectors(err,mem->JK,ee,eiinf,eisup,bounds,Je,Ji);
      sotDEBUG(15) << "ee" << iterTask << " = " << ee << std::endl;
      sotDEBUG(15) << "eiinf" << iterTask << " = " << eiinf << std::endl;
      sotDEBUG(15) << "eisup" << iterTask << " = " << eisup << std::endl;
      SOT_CHRONO;

      sotDEBUG(25) << "/* Solve level=" <<iterTask<<". */"<< std::endl;
      solver.solve( Je,ee,Ji,eiinf,eisup,bounds,solver.getSlackActiveSet() );
      sotDEBUG(1) << "u" << iterTask << " = " << (MATLAB)solver.u0 << endl;
      SOT_CHRONO;

      solver.computeDifferentialCondition();
#ifdef VP_DEBUG
      solver.printDifferentialCondition(sotDEBUGFLOW.outputbuffer);
#endif
      SOT_CHRONO;
    }

  /* --- Minimize norm --- */
  {
    sotDEBUG(25) << "Rank top." << std::endl;
    sotDEBUG(25) << "/* Set initial conditions. */" << std::endl;
    sotDEBUG(25) << solverPrec->u0 << std::endl;
    sotDEBUG(25) << solverNorm.u0 << std::endl;
    sotDEBUG(25) << solverNorm.nJ << std::endl;
    sotDEBUG(25) << nJ << std::endl;
    solverNorm.setInitialCondition( solverPrec->u0,solverPrec->rankh );
    SOT_CHRONO;

    sotDEBUG(25) << "/* Record warm start. */" << std::endl;
    solverNorm.recordInitialConditions();
    sotDEBUG(25) << "/* Effective warm start. */ " << std::endl;
    solverNorm.warmStart();
    SOT_CHRONO;

    sotDEBUG(25) << "/* Solve. */"<< std::endl;
    bubMatrix Idnj( bub::identity_matrix<double>(nJ,nJ));
    bubVector ee(nJ); ee.assign( bub::zero_vector<double>(nJ));
    bubMatrix Ji(0,0);    bubVector ei(0);  ConstraintMem::BoundSideVector bounds(0);
    solverNorm.solve( Idnj,ee,Ji,ei,ei,bounds,solverNorm.getSlackActiveSet() );
    sotDEBUG(1) << "utop = " << (MATLAB)solverNorm.u0 << endl;
    SOT_CHRONO;

    solverNorm.computeDifferentialCondition();
#ifdef VP_DEBUG
    solverNorm.printDifferentialCondition(sotDEBUGFLOW.outputbuffer);
#endif
    SOT_CHRONO;
  }

  /* Passing the result to the control. */
  control.resize(nJ);
  control.accessToMotherLib().assign( solverNorm.u0 );

  if( fillMemorySignal )
    for( StackType::iterator iter = stack.begin(); iter!=stack.end();++iter,++iterTask )
      {
        TaskAbstract & task = **iter;
        MemoryTaskSOTH * mem = dynamic_cast<MemoryTaskSOTH *>( task.memoryInternal );
        if( mem == NULL ) continue;
        VectorMultiBound taskVector = task.taskSOUT(iterTime);
        const ml::Matrix JK = mem->jacobianConstrainedSINOUT.accessCopy();
        ml::Vector JKu(taskVector.size()); JKu = JK*control;
        ml::Vector diff(taskVector.size());

        for( unsigned int i=0;i<taskVector.size();++i )
          {
            const MultiBound & Xi = taskVector[i];
            switch( Xi.getMode() )
              {
              case MultiBound::MODE_SINGLE:
                diff(i) = Xi.getSingleBound()-JKu(i);
                break;
              case MultiBound::MODE_DOUBLE:
                diff(i) = std::min( JKu(i)-Xi.getDoubleBound(MultiBound::BOUND_INF ),
                                    Xi.getDoubleBound(MultiBound::BOUND_SUP)-JKu(i) );
                break;
              }
          }
        mem->diffErrorSINOUT = diff;
        mem->diffErrorSINOUT.setTime( iterTime );

      }

  sotDEBUGOUT(15);
  return control;
}

/* --------------------------------------------------------------------- */
/* --- MEMORY ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */


const std::string SotH::MemoryTaskSOTH::CLASS_NAME = "MemoryTaskSOTH";

void SotH::MemoryTaskSOTH::
commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
             std::ostream& os )
{
  Entity::commandLine( cmdLine,cmdArgs,os );
}

void SotH::MemoryTaskSOTH::
display( std::ostream& os ) const {} //TODO


SotH::MemoryTaskSOTH::
MemoryTaskSOTH( const std::string & name,
                   const SotH * ref,
                   unsigned int nJ,
                   sotRotationComposedInExtenso& Qh,
                   bubMatrix &Rh,
                   SolverHierarchicalInequalities::ConstraintList &cH )
  :Entity(name),referenceKey(ref),solver(nJ,Qh,Rh,cH)
  ,jacobianConstrainedSINOUT( "sotTaskAbstract("+name+")::inout(matrix)::JK" )
  ,diffErrorSINOUT( "sotTaskAbstract("+name+")::inout(vector)::diff" )
{   signalRegistration(jacobianConstrainedSINOUT<<diffErrorSINOUT);  }

