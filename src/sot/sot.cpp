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
 public:sotSOT__INIT( void ) { sot::DebugTrace::openFile(); }
 };
 sotSOT__INIT sotSOT_initiator;
#endif //#ifdef VP_DEBUG

#include <sot/core/sot.hh>
#include <sot/core/pool.hh>
#include <sot/core/task.hh>
#include <sot/core/memory-task-sot.hh>
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
  addCommand("setNumberDofs",
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
  //controlSOUT.addDependency( task.featureActivationSOUT );
  controlSOUT.setReady();
}
TaskAbstract& Sot::
pop( void )
{
  TaskAbstract* res = stack.back();
  stack.pop_back();
  controlSOUT.removeDependency( res->taskSOUT );
  controlSOUT.removeDependency( res->jacobianSOUT );
  controlSOUT.removeDependency( res->featureActivationSOUT );
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
  //controlSOUT.removeDependency( key.featureActivationSOUT );
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

ml::Matrix & Sot::
computeJacobianConstrained( const ml::Matrix& Jac,
                            const ml::Matrix& K,
                            ml::Matrix& JK,
                            ml::Matrix& Jff,
                            ml::Matrix& Jact )
{
  const unsigned int nJ = Jac.nbRows();
  const unsigned int mJ = K.nbCols();
  const unsigned int nbConstraints = Jac.nbCols() - mJ;

  if (nbConstraints == 0) {
    JK = Jac;
    return JK;
  }
  for( unsigned int i=0;i<nJ;++i )
    {
      for( unsigned int j=0;j<nbConstraints;++j ) Jff(i,j)=Jac(i,j);
      for( unsigned int j=nbConstraints;j<Jac.nbCols();++j )
	Jact(i,j-nbConstraints)=Jac(i,j);
    }
  Jff.multiply(K,JK);
  JK+=Jact;

  return JK;
}


ml::Matrix & Sot::
computeJacobianConstrained( const TaskAbstract& task,
                            const ml::Matrix& K )
{
  const ml::Matrix &Jac = task.jacobianSOUT;
  MemoryTaskSOT * mem = dynamic_cast<MemoryTaskSOT *>( task.memoryInternal );
  if( NULL==mem ) throw; // TODO
  ml::Matrix &Jff = mem->Jff;
  ml::Matrix &Jact = mem->Jact;
  ml::Matrix &JK = mem->JK;
  return computeJacobianConstrained(Jac,K,JK,Jff,Jact);
}

static void computeJacobianActivated( Task* taskSpec,
				      ml::Matrix& Jt,
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
	      for( unsigned int i=0;i<Jt.nbCols();++i )
		{
		  if(! controlSelec(i) )
		    {for( unsigned int j=0;j<Jt.nbRows();++j ) { Jt(j,i)=0.; }}
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
      dt = ( (t1.tv_sec-t0.tv_sec) * 1000.* 1000.\
	     + (t1.tv_usec-t0.tv_usec+0.)  );\
      sotDEBUG(1) << "dt: "<< dt / 1000. << std::endl
#   define sotINITPARTCOUNTERS  struct timeval tpart0
#   define sotSTARTPARTCOUNTERS  gettimeofday(&tpart0,NULL)
#   define sotCOUNTER(nbc1,nbc2) \
	  gettimeofday(&tpart##nbc2,NULL); \
	  dt##nbc2 += ( (tpart##nbc2.tv_sec-tpart##nbc1.tv_sec) * 1000.* 1000. \
		   + (tpart##nbc2.tv_usec-tpart##nbc1.tv_usec+0.)  )
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

ml::Vector Sot::
taskVectorToMlVector( const VectorMultiBound& taskVector )
{
  ml::Vector res(taskVector.size()); unsigned int i=0;
  for( VectorMultiBound::const_iterator iter=taskVector.begin();
       iter!=taskVector.end();++iter,++i )
    {
      res(i)=iter->getSingleBound();
    }
  return res;
}

ml::Vector& Sot::
computeControlLaw( ml::Vector& control,const int& iterTime )
{
  sotDEBUGIN(15);

  sotINIT_CHRONO1; sotINITPARTCOUNTERS;
  sotINITCOUNTER(1); sotINITCOUNTER(2);sotINITCOUNTER(3);sotINITCOUNTER(4);
  sotINITCOUNTER(5); sotINITCOUNTER(6);sotINITCOUNTER(7);sotINITCOUNTER(8);

  sotSTART_CHRONO1;
  sotSTARTPARTCOUNTERS;

  const double &th = inversionThresholdSIN(iterTime);
  const ml::Matrix &K = constraintSOUT(iterTime);
  const unsigned int mJ = K.nbCols(); // number dofs - number constraints

  try {
    control = q0SIN( iterTime );
    sotDEBUG(15) << "initial velocity q0 = " << control << endl;
    if( mJ!=control.size() ) { control.resize( mJ ); control.fill(.0); }
  }
  catch (...)
    {
      if( mJ!=control.size() ) { control.resize( mJ ); }
      control.fill(0.);
      sotDEBUG(25) << "No initial velocity." <<endl;
    }

  sotDEBUGF(5, " --- Time %d -------------------", iterTime );
  unsigned int iterTask = 0;
  for( StackType::iterator iter = stack.begin(); iter!=stack.end();++iter )
    {
      sotDEBUGF(5,"Rank %d.",iterTask);
      TaskAbstract & task = **iter;
      sotDEBUG(15) << "Task: e_" << task.getName() << std::endl;
      const ml::Matrix &Jac = task.jacobianSOUT(iterTime);
      const ml::Vector err = taskVectorToMlVector(task.taskSOUT(iterTime));
      sotCOUNTER(0,1); // Direct Dynamic

      unsigned int rankJ;
      const unsigned int nJ = Jac.nbRows(); // number dofs

      /* Init memory. */
      MemoryTaskSOT * mem = dynamic_cast<MemoryTaskSOT *>( task.memoryInternal );
      if( NULL==mem )
        {
          if( NULL!=task.memoryInternal ) delete task.memoryInternal;
          mem = new MemoryTaskSOT( task.getName()+"_memSOT",nJ,mJ );
          task.memoryInternal = mem;
        }

      ml::Matrix &Jp = mem->Jp;
      ml::Matrix &V = mem->V;
      ml::Matrix &JK = mem->JK;
      ml::Matrix &Jt = mem->Jt;

      Jp.resize( mJ,nJ );
      V.resize( mJ,mJ );
      Jt.resize( nJ,mJ );
      JK.resize( nJ,mJ );

      if( (recomputeEachTime)
          ||(task.jacobianSOUT.getTime()>mem->jacobianInvSINOUT.getTime())
          ||(mem->jacobianInvSINOUT.accessCopy().nbRows()!=mJ)
          ||(mem->jacobianInvSINOUT.accessCopy().nbCols()!=nJ)
          ||(task.jacobianSOUT.getTime()>mem->jacobianConstrainedSINOUT.getTime())
          ||(task.jacobianSOUT.getTime()>mem->rankSINOUT.getTime())
          ||(task.jacobianSOUT.getTime()>mem->singularBaseImageSINOUT.getTime()) )
    {
	sotDEBUG(2) <<"Recompute inverse."<<endl;

	/* --- FIRST ALLOCS --- */
	ml::Vector &S = mem->S;

	sotDEBUG(1) << "Size = "
		    << S.size() + mem->Jff.nbCols()*mem->Jff.nbRows()
	  + mem->Jact.nbCols()*mem->Jact.nbRows() << std::endl;

sotDEBUG(1) << std::endl;
	S.resize( min(nJ,mJ) );
sotDEBUG(1) << nJ << " " << Jac.nbCols() <<" "<<mJ<<std::endl;
 mem->Jff.resize( nJ,Jac.nbCols()-mJ ); // number dofs, number constraints
sotDEBUG(1) << std::endl;
	mem->Jact.resize( nJ,mJ );
sotDEBUG(1) << std::endl;
	sotDEBUG(1) << "Size = "
		    << S.size() + mem->Jff.nbCols()*mem->Jff.nbRows()
	  + mem->Jact.nbCols()*mem->Jact.nbRows() << std::endl;

	/***/sotCOUNTER(1,2); // first allocs
	
	/* --- COMPUTE JK --- */
	computeJacobianConstrained( task,K );
	/***/sotCOUNTER(2,3); // compute JK
	
	/* --- COMPUTE Jt --- */
	if( 0<iterTask ) JK.multiply(Proj,Jt); else { Jt = JK; }
	/***/sotCOUNTER(3,4); // compute Jt
	
	/* --- COMPUTE S --- */
	computeJacobianActivated( dynamic_cast<Task*>( &task ),Jt,iterTime );
	/***/sotCOUNTER(4,5); // Jt*S
	
	/* --- PINV --- */
	Jt.dampedInverse( Jp,th,NULL,&S,&V );
	/***/sotCOUNTER(5,6); // PINV
	sotDEBUG(2) << "V after dampedInverse." << V <<endl;
	/* --- RANK --- */
	{
	  const unsigned int Jmax = S.size(); rankJ=0;
	  for( unsigned i=0;i<Jmax;++i ) { if( S(i)>th ) rankJ++; }
	}
	
	sotDEBUG(45) << "control"<<iterTask<<" = "<<control<<endl;
	sotDEBUG(25) << "J"<<iterTask<<" = "<<Jac<<endl;
	sotDEBUG(25) << "JK"<<iterTask<<" = "<<JK<<endl;
	sotDEBUG(25) << "Jt"<<iterTask<<" = "<<Jt<<endl;
	sotDEBUG(15) << "Jp"<<iterTask<<" = "<<Jp<<endl;
	sotDEBUG(15) << "e"<<iterTask<<" = "<<err<<endl;
	sotDEBUG(45) << "JJp"<<iterTask<<" = "<< JK*Jp <<endl;
	//sotDEBUG(45) << "U"<<iterTask<<" = "<< U<<endl;
	sotDEBUG(45) << "S"<<iterTask<<" = "<< S<<endl;
	sotDEBUG(45) << "V"<<iterTask<<" = "<< V<<endl;

	mem->jacobianInvSINOUT = Jp;
	mem->jacobianInvSINOUT.setTime( iterTime );
	mem->jacobianConstrainedSINOUT = JK;
	mem->jacobianConstrainedSINOUT.setTime( iterTime );
	mem->jacobianProjectedSINOUT = Jt;
	mem->jacobianProjectedSINOUT.setTime( iterTime );
	mem->singularBaseImageSINOUT = V;
	mem->singularBaseImageSINOUT.setTime( iterTime );
	mem->rankSINOUT = rankJ;
	mem->rankSINOUT.setTime( iterTime );

	sotDEBUG(25)<<"Inverse recomputed."<<endl;
      } else {
	sotDEBUG(2)<<"Inverse not recomputed."<<endl;
	rankJ = mem->rankSINOUT.accessCopy();
	Jp = mem->jacobianInvSINOUT.accessCopy();
	V = mem->singularBaseImageSINOUT.accessCopy();
	JK = mem->jacobianConstrainedSINOUT;
	Jt = mem->jacobianProjectedSINOUT;
      }
      /***/sotCOUNTER(6,7); // TRACE


      /* --- COMPUTE QDOT AND P --- */
      /*DEBUG: normally, the first iter (ie the test below)
      * is the same than the other, starting with control_0 = q0SIN. */
      if( iterTask==0 ) control += Jp*err; else
	control += Jp*(err - JK*control);
      /***/sotCOUNTER(7,8); // QDOT


      /* --- OPTIMAL FORM: To debug. --- */
      if( 0==iterTask )
	{ Proj.resize( mJ,mJ ); Proj.setIdentity(); }

//        {
// 	double *p,*v1,*v2,*vtmp1,*vtmp2;
// 	p = traits::matrix_storage(Proj.matrix);
// 	v1 = traits::matrix_storage(V.matrix);
// 	v2 = traits::matrix_storage(V.matrix);
// 	vtmp1 = traits::matrix_storage(V.matrix);
// 	/***/sotCOUNTER(6,7); // Ppre
 	
// 	for( unsigned int i=0;i<mJ;++i )
// 	  {
// 	    vtmp2 = traits::matrix_storage(V.matrix);
// 	    for( unsigned int j=0;j<mJ;++j )
// 	      {
// 		v1 = vtmp1;   v2 = vtmp2;
// 		for( unsigned int k=0;k<rankJ;++k )
// 		  {
// 		    (*p) -=( *v1) * (*v2);
// 		    v2++;v1++;
// 		  }
// 		p++;   vtmp2 += mJ;
// 	      }
// 	    vtmp1 += mJ;
// 	/***/sotCOUNTER(7,8); // P
// 	  }
//       }
       /* NON OPTIMAL FORM: to be replaced after debug. */
      //       Proj-=Jp*Jt;

      /* --- OLIVIER START  --- */
      sotDEBUG(2) << "Proj non optimal (rankJ= " <<rankJ
		  << ", iterTask ="  << iterTask
		  << ")";
      sotDEBUG(2) << "V = " << V;
      sotDEBUG(2) << "Jt = " << Jt;
      sotDEBUG(2) << "JpxJt = " << Jp*Jt;
      sotDEBUG(2) << "Ptmp" << iterTask <<" = " << Jp*Jt;

      /* NON OPTIMAL FORM: to be replaced after debug. */
      if (1)
       {
	 double *p,*v1,*v2,*vtmp1,*vtmp2;
	 p = MRAWDATA(Proj.matrix);
	 v1 = MRAWDATA(V.matrix);
	 v2 = MRAWDATA(V.matrix);
	 vtmp1 = MRAWDATA(V.matrix);
	 /***/sotCOUNTER(6,7); // Ppre
	
	 for( unsigned int i=0;i<mJ;++i )
	   {
	     vtmp2 = MRAWDATA(V.matrix);
	     for( unsigned int j=0;j<mJ;++j )
	       {
		 v1 = vtmp1;   v2 =vtmp2;
		 for( unsigned int k=0;k<rankJ;++k )
		   //for( unsigned int k=0;k<mJ;++k )
		   {
		     (*p) -=( *v1) * (*v2);
		     v2+=mJ;v1+=mJ;
		   }
		 p++; vtmp2 ++;
	       }
	     vtmp1++;
	   }
	 /***/sotCOUNTER(7,8); // P
       }
      else
	{ Proj-=Jp*Jt;}

       /* --- OLIVIER END --- */

       sotDEBUG(15) << "q"<<iterTask<<" = "<<control<<std::endl;
       sotDEBUG(25) << "P"<<iterTask<<" = "<< Proj <<endl;
       iterTask++;

       sotPRINTCOUNTER(1);     sotPRINTCOUNTER(2);    sotPRINTCOUNTER(3);
       sotPRINTCOUNTER(4);     sotPRINTCOUNTER(5);    sotPRINTCOUNTER(6);
       sotPRINTCOUNTER(7);     sotPRINTCOUNTER(8);

    }

  sotCHRONO1;

  if( 0!=taskGradient )
    {
      const ml::Vector err
        = taskVectorToMlVector(taskGradient->taskSOUT.access(iterTime));
      const ml::Vector & h = taskGradient->featureActivationSOUT.access(iterTime);
      const ml::Matrix & Jac = taskGradient->jacobianSOUT.access(iterTime);

      const unsigned int nJ = Jac.nbRows();

      MemoryTaskSOT * mem
        = dynamic_cast<MemoryTaskSOT *>( taskGradient->memoryInternal );
      if( NULL==mem )
        {
          if( NULL!=taskGradient->memoryInternal )
            { delete taskGradient->memoryInternal; }
          mem = new MemoryTaskSOT( taskGradient->getName()+"_memSOT",nJ,mJ );
          taskGradient->memoryInternal = mem;
        }

      sotDEBUG(45) << "K = " << K <<endl;
      sotDEBUG(45) << "Jff = " << Jac <<endl;

      /* --- MEMORY INIT --- */
      ml::Matrix &Jp = mem->Jp;
      ml::Matrix &PJp = mem->PJp;
      ml::Matrix &Jt = mem->Jt;

      mem->JK.resize( nJ,mJ );
      mem->Jt.resize( nJ,mJ );
      mem->Jff.resize( nJ,Jac.nbCols()-mJ );
      mem->Jact.resize( nJ,mJ );
      Jp.resize( mJ,nJ );
      PJp.resize( nJ,mJ );

     /* --- COMPUTE JK --- */
      ml::Matrix &JK = computeJacobianConstrained( *taskGradient,K);

      /* --- COMPUTE Jinv --- */
      sotDEBUG(35) << "grad = " << err <<endl;
      sotDEBUG(35) << "h = " << h <<endl;
      sotDEBUG(35) << "Jgrad = " << JK <<endl;

      // Use optimized-memory Jt to do the p-inverse.
      Jt=JK; Jt.dampedInverse( Jp,th );
      Proj.multiply( Jp,PJp );

      /* --- COMPUTE ERR --- */
      ml::Vector Herr( err.size() );
      for( unsigned int i=0;i<err.size(); ++i )
	{
	  if( h(i)<=0 ) Herr(i) = 0.;
	  else if( h(i) >= 1. ) Herr(i) = err(i);
	  else Herr(i) = err(i) * h(i) ;
	}

      /* --- COMPUTE CONTROL --- */
      control += PJp*Herr;

      /* ---  TRACE  --- */
      sotDEBUG(45) << "Pgrad = " << (PJp*Herr) <<endl;
      sotDEBUG(45) << "P = " << Proj <<endl;
      sotDEBUG(45) << "Jp = " << Jp <<endl;
      sotDEBUG(45) << "PJp = " << PJp <<endl;
    }

  sotDEBUGOUT(15);
  return control;
}



ml::Matrix& Sot::
computeConstraintProjector( ml::Matrix& ProjK, const int& time )
{
  sotDEBUGIN(15);
  const ml::Matrix *Jptr;
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

  const ml::Matrix &J = *Jptr;
  sotDEBUG(12) << "J = "<< J;

  const unsigned int nJc = J.nbCols();
  ml::Matrix Jff( J.nbRows(),ffJointIdLast-ffJointIdFirst );
  ml::Matrix Jc( J.nbRows(),nJc-ffJointIdLast+ffJointIdFirst );

  for( unsigned int i=0;i<J.nbRows();++i )
    {
      if( ffJointIdFirst )
	for( unsigned int j=0;j<ffJointIdFirst;++j )
	  Jc(i,j)=J(i,j);
      if( ffJointIdLast<nJc )
	for( unsigned int j=ffJointIdLast;j<nJc;++j )
	    Jc(i,j+ffJointIdFirst-ffJointIdLast)=J(i,j);
      for( unsigned int j=ffJointIdFirst;j<ffJointIdLast;++j )
	Jff( i,j-ffJointIdFirst )=J(i,j);
    }
  sotDEBUG(25) << "Jc = "<< Jc;
  sotDEBUG(25) << "Jff = "<< Jff;

  ml::Matrix Jffinv( Jff.nbCols(),Jff.nbRows() );
  Jff.pseudoInverse( Jffinv );   Jffinv *= -1;

  ml::Matrix& Jffc = ProjK;
  Jffc.resize( Jffinv.nbRows(),Jc.nbCols() );
  Jffinv.multiply( Jc,Jffc );
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
      std::string tname; cmdArgs >> tname;
      TaskAbstract & task = sotPool.getTask( tname );
      push(task);
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
	      TaskAbstract & task = sotPool.getTask( tname );
	      taskGradient = &task;
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
      std::string tname; cmdArgs >> tname;
      TaskAbstract & task = sotPool.getTask( tname );
      up(task);
    }
  else if( cmdLine == "down")
    {
      std::string tname; cmdArgs >> tname;
      TaskAbstract & task = sotPool.getTask( tname );
      down(task);
    }
  else if( cmdLine == "rm")
    {
      std::string tname; cmdArgs >> tname;
      TaskAbstract & task = sotPool.getTask( tname );
      remove(task);
    }
  else if( cmdLine == "pop")
    {
      TaskAbstract& task = pop();
      os << "Remove : "<< task << std::endl;
    }

  else if( cmdLine == "addConstraint" )
    {
      std::string cstname; cmdArgs >> cstname;
      Constraint &cs = dynamic_cast<Constraint&>(sotPool.getTask( cstname ));
      addConstraint( cs );
      constraintSOUT.setReady();
    }
  else if( cmdLine == "rmConstraint" )
    {
      std::string cstname; cmdArgs >> cstname;
      Constraint &cs = dynamic_cast<Constraint&>(sotPool.getTask( cstname ));
      removeConstraint( cs );
      constraintSOUT.setReady();
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
	  os << "\t\t\t" << task.getName() << " -> " << nexttask.getName() << " [color=red]" << endl;
	}

    }

  os << "\t\tsubgraph cluster_Tasks {" <<endl;
  os << "\t\t\tsubgraph cluster_" << getName() << " {" << std::endl;
  os << "\t\t\t\tcolor=lightsteelblue1; label=\"" << getName() <<"\"; style=filled;" << std::endl;
  for(  iter = stack.begin(); iter!=stack.end();++iter )
    {
      const TaskAbstract & task = **iter;
      os << "\t\t\t\t" << task.getName()
		<<" [ label = \"" << task.getName() << "\" ," << std::endl
		<<"\t\t\t\t   fontcolor = black, color = black, fillcolor = magenta, style=filled, shape=box ]" << std::endl;

    }
  os << "\t\t\t}" << std::endl;
  os << "\t\t\t}" <<endl;
  return os;
}
