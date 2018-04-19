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

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

//#define VP_DEBUG
//#define VP_DEBUG_MODE 45
#include <sot/core/debug.hh>

/* SOT */
#ifdef VP_DEBUG
 class sotSOT__INIT
 {
 public:sotSOT__INIT( void ) { dynamicgraph::sot::DebugTrace::openFile(); }
 };
 sotSOT__INIT sotSOT_initiator;
#endif //#ifdef VP_DEBUG

#include <sot/core/sot.hh>
#include <sot/core/pool.hh>
#include <sot/core/task.hh>
#include <sot/core/memory-task-sot.hh>
#include <sot/core/matrix-svd.hh>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/factory.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#include "../src/sot/sot-command.h"

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Sot,"SOT");


const double Sot::INVERSION_THRESHOLD_DEFAULT = 1e-4;
const unsigned int Sot::NB_JOINTS_DEFAULT = 46;

/* --------------------------------------------------------------------- */
/* --- CONSTRUCTION ---------------------------------------------------- */
/* --------------------------------------------------------------------- */
Sot::
Sot( const std::string& name )
  :Entity(name)
  ,stack()
  ,constraintList()
  ,ffJointIdFirst( FF_JOINT_ID_DEFAULT )
  ,ffJointIdLast( FF_JOINT_ID_DEFAULT+6 )

  ,nbJoints( NB_JOINTS_DEFAULT )
  ,taskGradient(0)
  ,recomputeEachTime(true)
  ,q0SIN( NULL,"sotSOT("+name+")::input(double)::q0" )
  ,inversionThresholdSIN( NULL,"sotSOT("+name+")::input(double)::damping" )
  ,constraintSOUT( boost::bind(&Sot::computeConstraintProjector,this,_1,_2),
		   sotNOSIGNAL,
		    "sotSOT("+name+")::output(matrix)::constraint" )
  ,controlSOUT( boost::bind(&Sot::computeControlLaw,this,_1,_2),
		constraintSOUT<<inversionThresholdSIN<<q0SIN,
		"sotSOT("+name+")::output(vector)::control" )
{
  inversionThresholdSIN = INVERSION_THRESHOLD_DEFAULT;

  signalRegistration( inversionThresholdSIN<<controlSOUT<<constraintSOUT<<q0SIN );

  // Commands
  //
  std::string docstring;
  // addConstraint
  docstring ="    \n"
    "    AddConstraint\n"
    "    \n"
    "      Input:\n"
    "        - a string: name of the constraint object\n"
    "    \n";
  addCommand("addConstraint",
	     new command::classSot::AddConstraint(*this, docstring));

  docstring ="    \n"
    "    setNumberDofs.\n"
    "    \n"
    "      Input:\n"
    "        - a positive integer : number of degrees of freedom of the robot.\n"
    "    \n";
  addCommand("setSize",
	     new dynamicgraph::command::Setter<Sot, unsigned int>
	     (*this, &Sot::defineNbDof, docstring));

  docstring ="    \n"
    "    push a task into the stack.\n"
    "    \n"
    "      Input:\n"
    "        - a string : Name of the task.\n"
    "    \n";
  addCommand("push",
	     new command::classSot::Push(*this, docstring));

  docstring ="    \n"
    "    remove a task into the stack.\n"
    "    \n"
    "      Input:\n"
    "        - a string : Name of the task.\n"
    "    \n";
  addCommand("remove",
	     new command::classSot::Remove(*this, docstring));

  docstring ="    \n"
    "    up a task into the stack.\n"
    "    \n"
    "      Input:\n"
    "        - a string : Name of the task.\n"
    "    \n";
  addCommand("up",
	     new command::classSot::Up(*this, docstring));

  docstring ="    \n"
    "    down a task into the stack.\n"
    "    \n"
    "      Input:\n"
    "        - a string : Name of the task.\n"
    "    \n";
  addCommand("down",
	     new command::classSot::Down(*this, docstring));

  // Display
  docstring ="    \n"
    "    display the list of tasks pushed inside the stack.\n"
    "    \n";
  addCommand("display",
	     new command::classSot::Display(*this, docstring));


  // Clear
  docstring ="    \n"
    "    clear the list of tasks pushed inside the stack.\n"
    "    \n";
  addCommand("clear",
	     new command::classSot::Clear(*this, docstring));

  // List
  docstring ="    \n"
    "    returns the list of tasks pushed inside the stack.\n"
    "    \n";
  addCommand("list",
	     new command::classSot::List(*this, docstring));


}

/* --------------------------------------------------------------------- */
/* --- STACK MANIPULATION --- */
/* --------------------------------------------------------------------- */
void Sot::
push( TaskAbstract& task )
{
  stack.push_back( &task );
  controlSOUT.addDependency( task.taskSOUT );
  controlSOUT.addDependency( task.jacobianSOUT );
  controlSOUT.setReady();
}
TaskAbstract& Sot::
pop( void )
{
  TaskAbstract* res = stack.back();
  stack.pop_back();
  controlSOUT.removeDependency( res->taskSOUT );
  controlSOUT.removeDependency( res->jacobianSOUT );
  controlSOUT.setReady();
  return *res;
}
bool Sot::
exist( const TaskAbstract& key )
{
  std::list<TaskAbstract*>::iterator it;
  for ( it=stack.begin();stack.end()!=it;++it )
    {
      if( *it == &key ) { return true; }
    }
  return false;
}
void Sot::
remove( const TaskAbstract& key )
{
  bool find =false; std::list<TaskAbstract*>::iterator it;
  for ( it=stack.begin();stack.end()!=it;++it )
    {
      if( *it == &key ) { find=true; break; }
    }
  if(! find ){ return; }

  stack.erase( it );
  removeDependency( key );
}

void Sot::
removeDependency( const TaskAbstract& key )
{
  controlSOUT.removeDependency( key.taskSOUT );
  controlSOUT.removeDependency( key.jacobianSOUT );
  controlSOUT.setReady();
}

void Sot::
up( const TaskAbstract& key )
{
  bool find =false; std::list<TaskAbstract*>::iterator it;
  for ( it=stack.begin();stack.end()!=it;++it )
    {
      if( *it == &key ) { find=true; break; }
    }
  if( stack.begin()==it ) { return; }
  if(! find ){ return; }

  std::list<TaskAbstract*>::iterator pos=it; pos--;
  TaskAbstract * task = *it;
  stack.erase( it );
  stack.insert( pos,task );
  controlSOUT.setReady();
}
void Sot::
down( const TaskAbstract& key )
{
  bool find =false; std::list<TaskAbstract*>::iterator it;
  for ( it=stack.begin();stack.end()!=it;++it )
    {
      if( *it == &key ) { find=true; break; }
    }
  if( stack.end()==it ) { return; }
  if(! find ){ return; }

  std::list<TaskAbstract*>::iterator pos=it; pos++;
  TaskAbstract* task=*it;
  stack.erase( it );
  if( stack.end()==pos ){ stack.push_back(task); }
  else
    {
      pos++;
      stack.insert( pos,task );
    }
  controlSOUT.setReady();
}

void Sot::
clear( void )
{
  for (  std::list<TaskAbstract*>::iterator it=stack.begin();stack.end()!=it;++it )
    {
      removeDependency( **it );
    }
  stack.clear();
  controlSOUT.setReady();
}

/* --------------------------------------------------------------------- */
/* --- CONSTRAINTS ----------------------------------------------------- */
/* --------------------------------------------------------------------- */

void Sot::
addConstraint( Constraint& constraint )
{
  constraintList.push_back( &constraint );
  constraintSOUT.addDependency( constraint.jacobianSOUT );
}

void Sot::
removeConstraint( const Constraint& key )
{
  bool find =false; ConstraintListType::iterator it;
  for ( it=constraintList.begin();constraintList.end()!=it;++it )
    {
      if( *it == &key ) { find=true; break; }
    }
  if(! find ){ return; }

  constraintList.erase( it );

  constraintSOUT.removeDependency( key.jacobianSOUT );
}
void Sot::
clearConstraint( void )
{
  for (  ConstraintListType::iterator it=constraintList.begin();
	 constraintList.end()!=it;++it )
    {
      constraintSOUT.removeDependency( (*it)->jacobianSOUT );
    }
  constraintList.clear();
}

void Sot::
defineFreeFloatingJoints( const unsigned int& first,const unsigned int& last )
{
  ffJointIdFirst = first ;
  if( last>0 ) ffJointIdLast=last ;
  else ffJointIdLast=ffJointIdFirst+6;
}

void Sot::
defineNbDof( const unsigned int& nbDof )
{
  nbJoints = nbDof;
  constraintSOUT.setReady();
  controlSOUT.setReady();
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

dynamicgraph::Matrix & Sot::
computeJacobianConstrained( const dynamicgraph::Matrix& Jac,
                            const dynamicgraph::Matrix& K,
                            dynamicgraph::Matrix& JK)
{
  const Matrix::Index nJ = Jac.rows();
  const Matrix::Index mJ = K.cols();
  const Matrix::Index nbConstraints = Jac.cols() - mJ;

  if (nbConstraints == 0) {
    JK = Jac;
    return JK;
  }
  JK.resize(nJ, mJ);
  JK.noalias() = Jac.leftCols (nbConstraints) * K;
  JK.noalias() += Jac.rightCols (Jac.cols() - nbConstraints);

  return JK;
}


dynamicgraph::Matrix & Sot::
computeJacobianConstrained( const TaskAbstract& task,
                            const dynamicgraph::Matrix& K )
{
  const dynamicgraph::Matrix &Jac = task.jacobianSOUT.accessCopy ();
  MemoryTaskSOT * mem = dynamic_cast<MemoryTaskSOT *>( task.memoryInternal );
  if( NULL==mem ) throw; // TODO
  dynamicgraph::Matrix &JK = mem->JK;
  return computeJacobianConstrained(Jac,K,JK);
}

static void computeJacobianActivated( Task* taskSpec,
				      dynamicgraph::Matrix& Jt,
				      const int& iterTime )
{
  if( NULL!=taskSpec )
    {
      const Flags& controlSelec = taskSpec->controlSelectionSIN( iterTime );
      sotDEBUG(25) << "Control selection = " << controlSelec <<endl;
      if( controlSelec )
	{
	  if(!controlSelec)
	    {
	      sotDEBUG(15) << "Control selection."<<endl;
	      for( int i=0;i<Jt.cols();++i )
		{
		  if(! controlSelec(i) )
		    { Jt.col (i).setZero(); }
		}
	    }
	  else
	    {
	      sotDEBUG(15) << "S is equal to Id."<<endl;
	    }
	}
      else
	{
	  sotDEBUG(15) << "Task not activated."<<endl;
	  Jt *= 0;
	}
    }
  else { /* No selection specification: nothing to do. */ }
}


/* --------------------------------------------------------------------- */
/* --- CONTROL --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


//#define WITH_CHRONO


#ifdef  WITH_CHRONO
	#ifndef WIN32
	#include <sys/time.h>
	#else /*WIN32*/
	#include <sot/core/utils-windows.hh>
	#endif /*WIN32*/
#endif /*WITH_CHRONO*/


#ifdef  WITH_CHRONO
#   define sotINIT_CHRONO1 struct timeval t0,t1; double dt
#   define sotSTART_CHRONO1  gettimeofday(&t0,NULL)
#   define sotCHRONO1 \
      gettimeofday(&t1,NULL);\
      dt = ( (double)(t1.tv_sec-t0.tv_sec) * 1000.* 1000.\
	     + (double)(t1.tv_usec-t0.tv_usec)  );\
      sotDEBUG(1) << "dt: "<< dt / 1000. << std::endl
#   define sotINITPARTCOUNTERS  struct timeval tpart0
#   define sotSTARTPARTCOUNTERS  gettimeofday(&tpart0,NULL)
#   define sotCOUNTER(nbc1,nbc2) \
	  gettimeofday(&tpart##nbc2,NULL); \
	  dt##nbc2 += ( (double)(tpart##nbc2.tv_sec-tpart##nbc1.tv_sec) * 1000.* 1000. \
		   +    (double)(tpart##nbc2.tv_usec-tpart##nbc1.tv_usec)  )
#   define sotINITCOUNTER(nbc1) \
   struct timeval tpart##nbc1; double dt##nbc1=0;
#   define sotPRINTCOUNTER(nbc1)  sotDEBUG(1) << "dt" << nbc1 << " = " << dt##nbc1 << std::endl
#else // #ifdef  WITH_CHRONO
#   define sotINIT_CHRONO1
#   define sotSTART_CHRONO1
#   define sotCHRONO1
#   define sotINITPARTCOUNTERS
#   define sotSTARTPARTCOUNTERS
#   define sotCOUNTER(nbc1,nbc2)
#   define sotINITCOUNTER(nbc1)
#   define sotPRINTCOUNTER(nbc1)
#endif // #ifdef  WITH_CHRONO

void Sot::
taskVectorToMlVector( const VectorMultiBound& taskVector, Vector& res )
{
  res.resize(taskVector.size());
  unsigned int i=0;
  
  for( VectorMultiBound::const_iterator iter=taskVector.begin();
       iter!=taskVector.end();++iter,++i ) {
    res(i)=iter->getSingleBound();
  }
}

dynamicgraph::Vector& Sot::
computeControlLaw( dynamicgraph::Vector& control,const int& iterTime )
{
  sotDEBUGIN(15);

  sotINIT_CHRONO1; sotINITPARTCOUNTERS;
  sotINITCOUNTER(1); sotINITCOUNTER(2);sotINITCOUNTER(3);sotINITCOUNTER(4);
  sotINITCOUNTER(5); sotINITCOUNTER(6);sotINITCOUNTER(7);sotINITCOUNTER(8);

  sotSTART_CHRONO1;
  sotSTARTPARTCOUNTERS;

  const double &th = inversionThresholdSIN(iterTime);
  const Matrix &K = constraintSOUT(iterTime);
  const Matrix::Index mJ = K.cols(); // number dofs - number constraints

  try {
    control = q0SIN( iterTime );
    sotDEBUG(15) << "initial velocity q0 = " << control << endl;
    if( mJ!=control.size() ) { control.resize( mJ ); control.setZero(); }
  }
  catch (...)
    {
      if( mJ!=control.size() ) { control.resize( mJ );}
      control.setZero();
      sotDEBUG(25) << "No initial velocity." <<endl;
    }

  sotDEBUGF(5, " --- Time %d -------------------", iterTime );
  unsigned int iterTask = 0;
  const Matrix* PrevProj = NULL;
  for( StackType::iterator iter = stack.begin(); iter!=stack.end();++iter )
    {
      sotDEBUGF(5,"Rank %d.",iterTask);
      TaskAbstract & task = **iter;
      sotDEBUG(15) << "Task: e_" << task.getName() << std::endl;
      const dynamicgraph::Matrix &Jac = task.jacobianSOUT(iterTime);
      sotCOUNTER(0,1); // Direct Dynamic

      unsigned int rankJ;
      const Matrix::Index nJ = Jac.rows(); // number dofs

      /* Init memory. */
      MemoryTaskSOT * mem = dynamic_cast<MemoryTaskSOT *>( task.memoryInternal );
      if( NULL==mem )
        {
          if( NULL!=task.memoryInternal ) delete task.memoryInternal;
          mem = new MemoryTaskSOT( task.getName()+"_memSOT",nJ,mJ );
          task.memoryInternal = mem;
        }

      Matrix &Jp = mem->Jp;
      Matrix &JK = mem->JK;
      Matrix &Jt = mem->Jt;
      Matrix &Proj = mem->Proj;
      MemoryTaskSOT::SVD_t& svd = mem->svd;

      taskVectorToMlVector(task.taskSOUT(iterTime), mem->err);
      const dynamicgraph::Vector &err = mem->err;

      Jp.resize( mJ,nJ );
      Jt.resize( nJ,mJ );
      JK.resize( nJ,mJ );

      if( (recomputeEachTime)
          ||(task.jacobianSOUT.getTime()>mem->jacobianInvSINOUT.getTime())
          ||(mem->jacobianInvSINOUT.accessCopy().rows()!=mJ)
          ||(mem->jacobianInvSINOUT.accessCopy().cols()!=nJ)
          ||(task.jacobianSOUT.getTime()>mem->jacobianConstrainedSINOUT.getTime())
          ||(task.jacobianSOUT.getTime()>mem->rankSINOUT.getTime())
          ||(task.jacobianSOUT.getTime()>mem->singularBaseImageSINOUT.getTime()) )
    {
	sotDEBUG(2) <<"Recompute inverse."<<endl;

	/* --- FIRST ALLOCS --- */
	sotDEBUG(1) << "Size = "
		    << std::min(nJ, mJ) + mem->Jff.cols()*mem->Jff.rows()
	  + mem->Jact.cols()*mem->Jact.rows() << std::endl;

	sotDEBUG(1) << std::endl;
	sotDEBUG(1) << "nJ=" << nJ << " " << "Jac.cols()=" << Jac.cols()
		    <<" "<< "mJ=" << mJ<<std::endl;
	mem->Jff.resize( nJ,Jac.cols()-mJ ); // number dofs, number constraints
	sotDEBUG(1) << std::endl;
	mem->Jact.resize( nJ,mJ );
	sotDEBUG(1) << std::endl;
	sotDEBUG(1) << "Size = "
		    << std::min(nJ, mJ) + mem->Jff.cols()*mem->Jff.rows()
	  + mem->Jact.cols()*mem->Jact.rows() << std::endl;

	/***/sotCOUNTER(1,2); // first allocs

	/* --- COMPUTE JK --- */
	computeJacobianConstrained( task,K );
	/***/sotCOUNTER(2,3); // compute JK

	/* --- COMPUTE Jt --- */
	if( 0<iterTask ) Jt.noalias() = JK*(*PrevProj); else { Jt = JK; }
	/***/sotCOUNTER(3,4); // compute Jt

	/* --- COMPUTE S --- */
	computeJacobianActivated( dynamic_cast<Task*>( &task ),Jt,iterTime );
	/***/sotCOUNTER(4,5); // Jt*S
	
	/* --- PINV --- */
        svd.compute (Jt);
        Eigen::dampedInverse (svd, Jp, th);
	/***/sotCOUNTER(5,6); // PINV
	sotDEBUG(20) << "V after dampedInverse." << svd.matrixV() <<endl;
	/* --- RANK --- */
	{
          rankJ = 0;
          while ( rankJ < svd.singularValues().size()
              &&  th    < svd.singularValues()[rankJ])
          { ++rankJ; }
	}

	sotDEBUG(45) << "control"<<iterTask<<" = "<<control<<endl;
	sotDEBUG(25) << "J"<<iterTask<<" = "<<Jac<<endl;
	sotDEBUG(25) << "JK"<<iterTask<<" = "<<JK<<endl;
	sotDEBUG(25) << "Jt"<<iterTask<<" = "<<Jt<<endl;
	sotDEBUG(15) << "Jp"<<iterTask<<" = "<<Jp<<endl;
	sotDEBUG(15) << "e"<<iterTask<<" = "<<err<<endl;
	sotDEBUG(45) << "JJp"<<iterTask<<" = "<< JK*Jp <<endl;
	//sotDEBUG(45) << "U"<<iterTask<<" = "<< U<<endl;
	sotDEBUG(45) << "S"<<iterTask<<" = "<< svd.singularValues()<<endl;
	sotDEBUG(45) << "V"<<iterTask<<" = "<< svd.matrixV()<<endl;
	sotDEBUG(45) << "U"<<iterTask<<" = "<< svd.matrixU()<<endl;

	mem->jacobianInvSINOUT = Jp;
	mem->jacobianInvSINOUT.setTime( iterTime );
	mem->jacobianConstrainedSINOUT = JK;
	mem->jacobianConstrainedSINOUT.setTime( iterTime );
	mem->jacobianProjectedSINOUT = Jt;
	mem->jacobianProjectedSINOUT.setTime( iterTime );
	mem->singularBaseImageSINOUT = svd.matrixV().leftCols(rankJ);
	mem->singularBaseImageSINOUT.setTime( iterTime );
	mem->rankSINOUT = rankJ;
	mem->rankSINOUT.setTime( iterTime );

	sotDEBUG(25)<<"Inverse recomputed."<<endl;
      } else {
	sotDEBUG(2)<<"Inverse not recomputed."<<endl;
	rankJ = mem->rankSINOUT.accessCopy();
	Jp = mem->jacobianInvSINOUT.accessCopy();
	JK = mem->jacobianConstrainedSINOUT.accessCopy ();
	Jt = mem->jacobianProjectedSINOUT.accessCopy ();
      }
      /***/sotCOUNTER(6,7); // TRACE

      /* --- COMPUTE QDOT AND P --- */
      /*DEBUG: normally, the first iter (ie the test below)
      * is the same than the other, starting with control_0 = q0SIN. */
      if( iterTask==0 ) control.noalias() += Jp*err;
      else              control           += *PrevProj * (Jp*(err - JK*control));
      /***/sotCOUNTER(7,8); // QDOT

      /* --- OPTIMAL FORM: To debug. --- */
      if( 0==iterTask ) {
        Proj.noalias() = svd.matrixV().rightCols(svd.matrixV().cols()-rankJ);
      } else {
        Proj.noalias() = *PrevProj * svd.matrixV().rightCols(svd.matrixV().cols()-rankJ);
      }

      /* --- OLIVIER START  --- */
      // Update by Joseph Mirabel to match Eigen API

      sotDEBUG(2) << "Proj non optimal (rankJ= " <<rankJ
		  << ", iterTask ="  << iterTask
		  << ")";
      sotDEBUG(20) << "V = " << svd.matrixV();
      sotDEBUG(20) << "Jt = " << Jt;
      sotDEBUG(20) << "JpxJt = " << Jp*Jt;
      sotDEBUG(25) << "Proj-Jp*Jt"<<iterTask<<" = "<< (Proj-Jp*Jt) <<endl;

       /* --- OLIVIER END --- */

       sotDEBUG(15) << "q"<<iterTask<<" = "<<control<<std::endl;
       sotDEBUG(25) << "P"<<iterTask<<" = "<< Proj <<endl;
       iterTask++;
       PrevProj = &Proj;

       sotPRINTCOUNTER(1);     sotPRINTCOUNTER(2);    sotPRINTCOUNTER(3);
       sotPRINTCOUNTER(4);     sotPRINTCOUNTER(5);    sotPRINTCOUNTER(6);
       sotPRINTCOUNTER(7);     sotPRINTCOUNTER(8);

    }

  sotCHRONO1;

  if( 0!=taskGradient )
    {
      const dynamicgraph::Matrix & Jac = taskGradient->jacobianSOUT.access(iterTime);

      const Matrix::Index nJ = Jac.rows();

      MemoryTaskSOT * mem
        = dynamic_cast<MemoryTaskSOT *>( taskGradient->memoryInternal );
      if( NULL==mem )
        {
          if( NULL!=taskGradient->memoryInternal )
            { delete taskGradient->memoryInternal; }
          mem = new MemoryTaskSOT( taskGradient->getName()+"_memSOT",nJ,mJ );
          taskGradient->memoryInternal = mem;
        }

      taskVectorToMlVector(taskGradient->taskSOUT.access(iterTime), mem->err);
      const dynamicgraph::Vector& err = mem->err;

      sotDEBUG(45) << "K = " << K <<endl;
      sotDEBUG(45) << "Jff = " << Jac <<endl;

      /* --- MEMORY INIT --- */
      dynamicgraph::Matrix &Jp = mem->Jp;
      dynamicgraph::Matrix &PJp = mem->PJp;
      dynamicgraph::Matrix &Jt = mem->Jt;
      MemoryTaskSOT::SVD_t& svd = mem->svd;

      mem->JK.resize( nJ,mJ );
      mem->Jt.resize( nJ,mJ );
      mem->Jff.resize( nJ,Jac.cols()-mJ );
      mem->Jact.resize( nJ,mJ );
      Jp.resize( mJ,nJ );
      PJp.resize( nJ,mJ );

     /* --- COMPUTE JK --- */
      dynamicgraph::Matrix &JK = computeJacobianConstrained( *taskGradient,K);

      /* --- COMPUTE Jinv --- */
      sotDEBUG(35) << "grad = " << err <<endl;
      sotDEBUG(35) << "Jgrad = " << JK <<endl;

      // Use optimized-memory Jt to do the p-inverse.
      Jt=JK;
      svd.compute (Jt);
      // TODO the two next lines could be replaced by
      Eigen::dampedInverse( svd, Jp,th );
      if (PrevProj != NULL)
        PJp.noalias() = (*PrevProj)*Jp;
      else
        PJp.noalias() = Jp;

      /* --- COMPUTE ERR --- */
      const dynamicgraph::Vector& Herr( err );

      /* --- COMPUTE CONTROL --- */
      control.noalias() += PJp*Herr;

      /* ---  TRACE  --- */
      sotDEBUG(45) << "Pgrad = " << (PJp*Herr) <<endl;
      if (PrevProj != NULL) { sotDEBUG(45) << "P = " << *PrevProj <<endl; }
      sotDEBUG(45) << "Jp = " << Jp <<endl;
      sotDEBUG(45) << "PJp = " << PJp <<endl;
    }

  sotDEBUGOUT(15);
  return control;
}



dynamicgraph::Matrix& Sot::
computeConstraintProjector( dynamicgraph::Matrix& ProjK, const int& time )
{
  sotDEBUGIN(15);
  const dynamicgraph::Matrix *Jptr;
  if( 0==constraintList.size() )
    {
      ProjK.resize( 0, nbJoints );
      sotDEBUGOUT(15);
      return ProjK;
    }
  else if( 1==constraintList.size() )
    { Jptr = &(*constraintList.begin())->jacobianSOUT(time); }
  else
    {
      SOT_THROW ExceptionTask( ExceptionTask::EMPTY_LIST,
				  "Not implemented yet." );
    }

  const dynamicgraph::Matrix &J = *Jptr;
  sotDEBUG(12) << "J = "<< J;

  const Matrix::Index nJc = J.cols();
  dynamicgraph::Matrix Jff( J.rows(),ffJointIdLast-ffJointIdFirst );
  dynamicgraph::Matrix Jc( J.rows(),nJc-ffJointIdLast+ffJointIdFirst );

  Jc.leftCols (ffJointIdFirst)    = J.leftCols  (ffJointIdFirst);
  Jc.rightCols(nJc-ffJointIdLast) = J.rightCols (nJc-ffJointIdLast);
  Jff = J.middleCols(ffJointIdFirst, ffJointIdLast);

  sotDEBUG(25) << "Jc = "<< Jc;
  sotDEBUG(25) << "Jff = "<< Jff;

  dynamicgraph::Matrix Jffinv( Jff.cols(),Jff.rows() );
  Eigen::pseudoInverse(Jff, Jffinv);
  
  dynamicgraph::Matrix& Jffc = ProjK;
  Jffc.resize( Jffinv.rows(),Jc.cols() );
  Jffc = -Jffinv*Jc;
  sotDEBUG(15) << "Jffc = "<< Jffc;

  sotDEBUGOUT(15);
  return ProjK;
}


/* --------------------------------------------------------------------- */
/* --- DISPLAY --------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void Sot::
display( std::ostream& os ) const
{

  os << "+-----------------"<<std::endl<<"+   SOT     "
     << std::endl<< "+-----------------"<<std::endl;
  for ( std::list<TaskAbstract*>::const_iterator it=this->stack.begin();
	this->stack.end()!=it;++it )
    {
      os << "| " << (*it)->getName() <<std::endl;
    }
  os<< "+-----------------"<<std::endl;
  if( taskGradient )
    {
      os << "| {Grad} " <<taskGradient->getName() << std::endl;
      os<< "+-----------------"<<std::endl;
    }
  for( ConstraintListType::const_iterator it = constraintList.begin();
       it!=constraintList.end();++it )
    { os<< "| K: "<< (*it)->getName() << endl; }


}

std::ostream&
operator<< ( std::ostream& os,const Sot& sot )
{
  sot.display(os);
  return os;
}

/* --------------------------------------------------------------------- */
/* --- COMMAND --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void Sot::
commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
	     std::ostream& os )
{
  sotDEBUGIN(15);

  if( cmdLine == "help")
    {
      os << "Stack of Tasks: "<<endl
	 << " - push <task>"<< endl
	 << " - pop"<< endl
	 << " - down <task>"<< endl
	 << " - up <task>"<< endl
	 << " - rm <task>"<< endl
	 << " - clear"<< endl
	 << " - display"<< endl

	 << " - addConstraint <constraint> "<<endl
	 << " - rmConstraint <constraint> "<<endl
	 << " - clearConstraint"<<endl
	 << " - printConstraint "<<endl

	 << " - nbJoints <nb> "<<endl
         << " - recomputeEachTime [true|false]" << endl;
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine == "recomputeEachTime")
    {
      std::string tname; cmdArgs >> std::ws;
      if( cmdArgs.good() ) cmdArgs >> recomputeEachTime;
      else os << "recomputeEachTime = " << ((recomputeEachTime)?"true":"false") << std::endl;
    }
  else if( cmdLine == "clear")
    {
      clear();
    }
  else if( cmdLine == "push")
    {
    }
  else if( cmdLine == "gradient")
    {
      std::string tname; cmdArgs >> std::ws;
      if( cmdArgs.good() )
	{
	  cmdArgs >> tname;
	  if( ( "0"==tname )||( "rm"==tname ) )
	    { taskGradient = 0; }
	  else
	    {
	    }
	}
      else
	{
	  os << "gradient = ";
	  if( taskGradient ) os << (*taskGradient) << std::endl;
	  else os << "undef. " << std::endl;
	}
      controlSOUT.setReady();
    }
  else if( cmdLine == "up")
    {
    }
  else if( cmdLine == "down")
    {
    }
  else if( cmdLine == "rm")
    {
    }
  else if( cmdLine == "pop")
    {
      TaskAbstract& task = pop();
      os << "Remove : "<< task << std::endl;
    }

  else if( cmdLine == "addConstraint" )
    {
    }
  else if( cmdLine == "rmConstraint" )
    {
    }
  else if( cmdLine == "clearConstraint" )
    {
      clearConstraint( );
      constraintSOUT.setReady();
    }
  else if( cmdLine == "printConstraint" )
    {
      os<< "Constraints: "<<std::endl;
      for( ConstraintListType::iterator it = constraintList.begin();
	   it!=constraintList.end();++it )
	{ os<< "  - "<< (*it)->getName() << endl; }
    }
  else if( cmdLine == "nbJoints")
    {
      cmdArgs>>ws;
      if( cmdArgs.good() )
	{
          unsigned int nbDof;
	  cmdArgs >> nbDof;
          defineNbDof(nbDof);
	} else { os << "nbJoints = "<< nbJoints <<endl; }
    }
  else if( cmdLine == "display")
    {
      display(os);
    }
  else
    Entity::commandLine( cmdLine,cmdArgs,os );


  sotDEBUGOUT(15);
}

std::ostream& Sot::
writeGraph( std::ostream& os ) const
{
  std::list<TaskAbstract *>::const_iterator iter;
  for(  iter = stack.begin(); iter!=stack.end();++iter )
    {
      const TaskAbstract & task = **iter;
      std::list<TaskAbstract *>::const_iterator nextiter =iter;
      nextiter++;

      if (nextiter!=stack.end())
	{
	  TaskAbstract & nexttask = **nextiter;
	  os << "\t\t\t\"" << task.getName() << "\" -> \"" << nexttask.getName() << "\" [color=red]" << endl;
	}

    }

  os << "\t\tsubgraph cluster_Tasks {" <<endl;
  os << "\t\t\tsubgraph \"cluster_" << getName() << "\" {" << std::endl;
  os << "\t\t\t\tcolor=lightsteelblue1; label=\"" << getName() <<"\"; style=filled;" << std::endl;
  for(  iter = stack.begin(); iter!=stack.end();++iter )
    {
      const TaskAbstract & task = **iter;
      os << "\t\t\t\t\"" << task.getName()
		<<"\" [ label = \"" << task.getName() << "\" ," << std::endl
		<<"\t\t\t\t   fontcolor = black, color = black, fillcolor = magenta, style=filled, shape=box ]" << std::endl;

    }
  os << "\t\t\t}" << std::endl;
  os << "\t\t\t}" <<endl;
  return os;
}
