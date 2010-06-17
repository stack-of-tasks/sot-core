/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotSot.cpp
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

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

//#define VP_DEBUG
#define VP_DEBUG_MODE 45
#include <sot-core/sotDebug.h>

/* SOT */
#ifdef VP_DEBUG
 class sotSOT__INIT
 {
 public:sotSOT__INIT( void ) { sotDebugTrace::openFile(); }
 };
 sotSOT__INIT sotSOT_initiator;
#endif //#ifdef VP_DEBUG

#include <sot-core/sot.h>
#include <sot-core/pool.h>
#include <sot-core/task.h>
#include <sot-core/memory-task-sot.h>

using namespace std;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot-core/factory.h>
SOT_FACTORY_ENTITY_PLUGIN(sotSOT,"SOT");


const double sotSOT::INVERSION_THRESHOLD_DEFAULT = 1e-4;
const unsigned int sotSOT::NB_JOINTS_DEFAULT = 46;

/* --------------------------------------------------------------------- */
/* --- CONSTRUCTION ---------------------------------------------------- */
/* --------------------------------------------------------------------- */
sotSOT::
sotSOT( const std::string& name )
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
   ,constraintSOUT( boost::bind(&sotSOT::computeConstraintProjector,this,_1,_2),
		   sotNOSIGNAL,
		    "sotSOT("+name+")::output(matrix)::constraint" )
  ,controlSOUT( boost::bind(&sotSOT::computeControlLaw,this,_1,_2),
		constraintSOUT<<inversionThresholdSIN<<q0SIN,
		"sotSOT("+name+")::output(vector)::control" )
{
  inversionThresholdSIN = INVERSION_THRESHOLD_DEFAULT;

  signalRegistration( inversionThresholdSIN<<controlSOUT<<constraintSOUT<<q0SIN );
}

/* --------------------------------------------------------------------- */
/* --- STACK MANIPULATION --- */
/* --------------------------------------------------------------------- */
void sotSOT::
push( sotTaskAbstract& task )
{
  stack.push_back( &task );
  controlSOUT.addDependancy( task.taskSOUT );
  controlSOUT.addDependancy( task.jacobianSOUT );
  //controlSOUT.addDependancy( task.featureActivationSOUT );
  controlSOUT.setReady();
}
sotTaskAbstract& sotSOT::
pop( void )
{
  sotTaskAbstract* res = stack.back();
  stack.pop_back();
  controlSOUT.removeDependancy( res->taskSOUT );
  controlSOUT.removeDependancy( res->jacobianSOUT );
  controlSOUT.removeDependancy( res->featureActivationSOUT );
  controlSOUT.setReady();
  return *res;
}
bool sotSOT::
exist( const sotTaskAbstract& key )
{
  std::list<sotTaskAbstract*>::iterator it;
  for ( it=stack.begin();stack.end()!=it;++it )
    {
      if( *it == &key ) { return true; }
    }
  return false;
}
void sotSOT::
remove( const sotTaskAbstract& key )
{
  bool find =false; std::list<sotTaskAbstract*>::iterator it;
  for ( it=stack.begin();stack.end()!=it;++it )
    {
      if( *it == &key ) { find=true; break; }
    }
  if(! find ){ return; }

  stack.erase( it );
  removeDependancy( key );
}

void sotSOT::
removeDependancy( const sotTaskAbstract& key )
{
  controlSOUT.removeDependancy( key.taskSOUT );
  controlSOUT.removeDependancy( key.jacobianSOUT );
  //controlSOUT.removeDependancy( key.featureActivationSOUT );
  controlSOUT.setReady();
}

void sotSOT::
up( const sotTaskAbstract& key )
{
  bool find =false; std::list<sotTaskAbstract*>::iterator it;
  for ( it=stack.begin();stack.end()!=it;++it )
    {
      if( *it == &key ) { find=true; break; }
    }
  if( stack.begin()==it ) { return; }
  if(! find ){ return; }

  std::list<sotTaskAbstract*>::iterator pos=it; pos--;
  sotTaskAbstract * task = *it;
  stack.erase( it );
  stack.insert( pos,task );
  controlSOUT.setReady();
}
void sotSOT::
down( const sotTaskAbstract& key )
{
  bool find =false; std::list<sotTaskAbstract*>::iterator it;
  for ( it=stack.begin();stack.end()!=it;++it )
    {
      if( *it == &key ) { find=true; break; }
    }
  if( stack.end()==it ) { return; }
  if(! find ){ return; }

  std::list<sotTaskAbstract*>::iterator pos=it; pos++;
  sotTaskAbstract* task=*it;
  stack.erase( it );
  if( stack.end()==pos ){ stack.push_back(task); }
  else
    {
      pos++;
      stack.insert( pos,task );
    }
  controlSOUT.setReady();
}

void sotSOT::
clear( void )
{
  for (  std::list<sotTaskAbstract*>::iterator it=stack.begin();stack.end()!=it;++it )
    {
      removeDependancy( **it );
    }
  stack.clear();
  controlSOUT.setReady();
}

/* --------------------------------------------------------------------- */
/* --- CONSTRAINTS ----------------------------------------------------- */
/* --------------------------------------------------------------------- */

void sotSOT::
addConstraint( sotConstraint& constraint )
{
  constraintList.push_back( &constraint );
  constraintSOUT.addDependancy( constraint.jacobianSOUT );
}

void sotSOT::
removeConstraint( const sotConstraint& key )
{
  bool find =false; ConstraintListType::iterator it;
  for ( it=constraintList.begin();constraintList.end()!=it;++it )
    {
      if( *it == &key ) { find=true; break; }
    }
  if(! find ){ return; }

  constraintList.erase( it );

  constraintSOUT.removeDependancy( key.jacobianSOUT );
}
void sotSOT::
clearConstraint( void )
{
  for (  ConstraintListType::iterator it=constraintList.begin();
	 constraintList.end()!=it;++it )
    {
      constraintSOUT.removeDependancy( (*it)->jacobianSOUT );
    }
  constraintList.clear();
}

void sotSOT::
defineFreeFloatingJoints( const unsigned int& first,const unsigned int& last )
{
  ffJointIdFirst = first ;
  if( last>0 ) ffJointIdLast=last ;
  else ffJointIdLast=ffJointIdFirst+6;
}

void sotSOT::
defineNbDof( const unsigned int& nbDof )
{
  nbJoints = nbDof;
  constraintSOUT.setReady();
  controlSOUT.setReady();
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

ml::Matrix & sotSOT::
computeJacobianConstrained( const ml::Matrix& Jac,
                            const ml::Matrix& K,
                            ml::Matrix& JK,
                            ml::Matrix& Jff,
                            ml::Matrix& Jact )
{
  const unsigned int nJ = Jac.nbRows();
  const unsigned int mJ = K.nbCols();
  const unsigned int FF_SIZE = Jac.nbCols() - mJ;

  for( unsigned int i=0;i<nJ;++i )
    {
      for( unsigned int j=0;j<FF_SIZE;++j ) Jff(i,j)=Jac(i,j);
      for( unsigned int j=FF_SIZE;j<Jac.nbCols();++j )
	Jact(i,j-FF_SIZE)=Jac(i,j);
    }
  Jff.multiply(K,JK);
  JK+=Jact;

  return JK;
}


ml::Matrix & sotSOT::
computeJacobianConstrained( const sotTaskAbstract& task,
                            const ml::Matrix& K )
{
  const ml::Matrix &Jac = task.jacobianSOUT;
  sotMemoryTaskSOT * mem = dynamic_cast<sotMemoryTaskSOT *>( task.memoryInternal );
  if( NULL==mem ) throw; // TODO
  ml::Matrix &Jff = mem->Jff;
  ml::Matrix &Jact = mem->Jact;
  ml::Matrix &JK = mem->JK;
  return computeJacobianConstrained(Jac,K,JK,Jff,Jact);
}

static void computeJacobianActivated( sotTask* taskSpec,
				      ml::Matrix& Jt,
				      const int& iterTime )
{
  if( NULL!=taskSpec )
    {
      const sotFlags& controlSelec = taskSpec->controlSelectionSIN( iterTime );
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
	// When including Winsock2.h, the MAL must be included first
	//#include <MatrixAbstractLayer/boost.h>
	#include <sot-core/sotUtilsWindows.h>
	#include <Winsock2.h>
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

ml::Vector sotSOT::
taskVectorToMlVector( const sotVectorMultiBound& taskVector )
{
  ml::Vector res(taskVector.size()); unsigned int i=0;
  for( sotVectorMultiBound::const_iterator iter=taskVector.begin();
       iter!=taskVector.end();++iter,++i )
    {
      res(i)=iter->getSingleBound();
    }
  return res;
}

ml::Vector& sotSOT::
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
  const unsigned int mJ = K.nbCols();

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
      sotTaskAbstract & task = **iter;
      sotDEBUG(15) << "Task: e_" << task.getName() << std::endl;
      const ml::Matrix &Jac = task.jacobianSOUT(iterTime);
      const ml::Vector err = taskVectorToMlVector(task.taskSOUT(iterTime));
      sotCOUNTER(0,1); // Direct Dynamic

      unsigned int rankJ;
      const unsigned int nJ = Jac.nbRows();

      /* Init memory. */
      sotMemoryTaskSOT * mem = dynamic_cast<sotMemoryTaskSOT *>( task.memoryInternal );
      if( NULL==mem )
        {
          if( NULL!=task.memoryInternal ) delete task.memoryInternal;
          mem = new sotMemoryTaskSOT( task.getName()+"_memSOT",nJ,mJ );
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
	mem->Jff.resize( nJ,Jac.nbCols()-mJ );
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
	computeJacobianActivated( dynamic_cast<sotTask*>( &task ),Jt,iterTime );
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

      sotMemoryTaskSOT * mem
        = dynamic_cast<sotMemoryTaskSOT *>( taskGradient->memoryInternal );
      if( NULL==mem )
        {
          if( NULL!=taskGradient->memoryInternal )
            { delete taskGradient->memoryInternal; }
          mem = new sotMemoryTaskSOT( taskGradient->getName()+"_memSOT",nJ,mJ );
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



ml::Matrix& sotSOT::
computeConstraintProjector( ml::Matrix& ProjK, const int& time )
{
  sotDEBUGIN(15);
  const ml::Matrix *Jptr;
  if( 0==constraintList.size() )
    {
      const unsigned int FF_SIZE = ffJointIdLast-ffJointIdFirst;
      ProjK.resize( FF_SIZE,nbJoints-FF_SIZE ); ProjK.fill(.0);
      sotDEBUGOUT(15);
      return ProjK;
    }
  else if( 1==constraintList.size() )
    { Jptr = &(*constraintList.begin())->jacobianSOUT(time); }
  else
    {
      SOT_THROW sotExceptionTask( sotExceptionTask::EMPTY_LIST,
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
void sotSOT::
display( std::ostream& os ) const
{

  os << "+-----------------"<<std::endl<<"+   SOT     "
     << std::endl<< "+-----------------"<<std::endl;
  for ( std::list<sotTaskAbstract*>::const_iterator it=this->stack.begin();
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
operator<< ( std::ostream& os,const sotSOT& sot )
{
  sot.display(os);
  return os;
}

/* --------------------------------------------------------------------- */
/* --- COMMAND --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void sotSOT::
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
      sotTaskAbstract & task = sotPool.getTask( tname );
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
	      sotTaskAbstract & task = sotPool.getTask( tname );
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
      sotTaskAbstract & task = sotPool.getTask( tname );
      up(task);
    }
  else if( cmdLine == "down")
    {
      std::string tname; cmdArgs >> tname;
      sotTaskAbstract & task = sotPool.getTask( tname );
      down(task);
    }
  else if( cmdLine == "rm")
    {
      std::string tname; cmdArgs >> tname;
      sotTaskAbstract & task = sotPool.getTask( tname );
      remove(task);
    }
  else if( cmdLine == "pop")
    {
      sotTaskAbstract& task = pop();
      os << "Remove : "<< task << std::endl;
    }

  else if( cmdLine == "addConstraint" )
    {
      std::string cstname; cmdArgs >> cstname;
      sotConstraint &cs = dynamic_cast<sotConstraint&>(sotPool.getTask( cstname ));
      addConstraint( cs );
      constraintSOUT.setReady();
    }
  else if( cmdLine == "rmConstraint" )
    {
      std::string cstname; cmdArgs >> cstname;
      sotConstraint &cs = dynamic_cast<sotConstraint&>(sotPool.getTask( cstname ));
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

std::ostream& sotSOT::
writeGraph( std::ostream& os ) const
{
  std::list<sotTaskAbstract *>::const_iterator iter;
  for(  iter = stack.begin(); iter!=stack.end();++iter )
    {
      const sotTaskAbstract & task = **iter;
      std::list<sotTaskAbstract *>::const_iterator nextiter =iter;
      nextiter++;

      if (nextiter!=stack.end())
	{
	  sotTaskAbstract & nexttask = **nextiter;
	  os << "\t\t\t" << task.getName() << " -> " << nexttask.getName() << " [color=red]" << endl;
	}

    }

  os << "\t\tsubgraph cluster_Tasks {" <<endl;
  os << "\t\t\tsubgraph cluster_" << getName() << " {" << std::endl;
  os << "\t\t\t\tcolor=lightsteelblue1; label=\"" << getName() <<"\"; style=filled;" << std::endl;
  for(  iter = stack.begin(); iter!=stack.end();++iter )
    {
      const sotTaskAbstract & task = **iter;
      os << "\t\t\t\t" << task.getName()
		<<" [ label = \"" << task.getName() << "\" ," << std::endl
		<<"\t\t\t\t   fontcolor = black, color = black, fillcolor = magenta, style=filled, shape=box ]" << std::endl;

    }
  os << "\t\t\t}" << std::endl;
  os << "\t\t\t}" <<endl;
  return os;
}
