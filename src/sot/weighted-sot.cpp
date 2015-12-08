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

/* SOT */
#include <sot/core/weighted-sot.hh>
#include <sot/core/memory-task-sot.hh>
#include <sot/core/pool.hh>
#include <sot/core/task.hh>
#include <sot/core/debug.hh>
using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot/core/factory.hh>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(WeightedSot,"WSOT");

/* --------------------------------------------------------------------- */
/* --- CONSTRUCTION ---------------------------------------------------- */
/* --------------------------------------------------------------------- */
WeightedSot::
WeightedSot( const std::string& name )
  :Sot(name)
   ,weightSIN( NULL,"sotWeightedSOT("+name+")::input(matrix)::weight" )
   ,constrainedWeightSOUT( boost::bind(&WeightedSot::computeConstrainedWeight,
				       this,_1,_2),
			   weightSIN<<constraintSOUT,
			   "sotWeightedSOT("+name+")::input(matrix)::KweightOUT" )
   ,constrainedWeightSIN( NULL,"sotWeightedSOT("+name+")::input(matrix)::Kweight" )
   ,squareRootInvWeightSOUT( boost::bind(&WeightedSot::computeSquareRootInvWeight,this,_1,_2),
			     weightSIN<<constrainedWeightSIN,
			     "sotWeightedSOT("+name+")::output(matrix)::sqweight" )
   ,squareRootInvWeightSIN( &squareRootInvWeightSOUT,
			    "sotWeightedSOT("+name+")::input(matrix)::sqweightIN" )
{
  sotDEBUGIN(25);
  signalRegistration( weightSIN<<constrainedWeightSIN<<squareRootInvWeightSOUT
		      << squareRootInvWeightSIN );

  controlSOUT.setFunction( boost::bind(&WeightedSot::computeWeightedControlLaw,
				       this,_1,_2) );
  controlSOUT.addDependency( squareRootInvWeightSIN );

  constrainedWeightSIN.plug(&constrainedWeightSOUT);
  sotDEBUGOUT(25);
}


/* --------------------------------------------------------------------- */
/* --- A inv ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/linear-algebra.h>

dynamicgraph::Matrix& WeightedSot::
computeSquareRootInvWeight( dynamicgraph::Matrix& S5i,const int& time )
{
  sotDEBUGIN(15);

  const dynamicgraph::Matrix& A = constrainedWeightSIN( time );
  if( A.cols()!= A.rows() )
    {
      SOT_THROW ExceptionTask( ExceptionTask::MATRIX_SIZE,
				  "Weight matrix should be square.","" );
    }
  sotDEBUG(25) << "KA = " << A << endl;

  /* Decomposition of Choleski:
   *  (1) Diag of S5 is the sqrt of diag of A minus the sum-square
   * of the begining of colum i of S5: s_ii = sqrt(a_ii - sum(k=0:i-1) s_ki^2).
   *  (2) For each element of the line, the component s_ij is the cor. element
   * of A minus the scalar product of column i and j of s, divided by the diag
   * of S: s_ij = (a_ij - sum(k=0:i) s_ik.s_jk ) / s_ii .
   */
  dynamicgraph::Matrix S5( A.rows(),A.rows() ); S5.setZero();
  //S5.resize( A.rows(),A.rows() ); S5.setZero();
  for( unsigned int i=0;i<A.cols();++i )
    {
      double x=A(i,i);
      for( unsigned int k=0;k<i;++k )
	x-=S5(k,i)*S5(k,i);
      double sq=sqrt(x);
      S5(i,i)=sq;

      for( unsigned int j=i+1;j<A.cols();++j )
	{
	  x=A(i,j);
	  for( unsigned int k=0;k<i;++k )
	    x-=S5(k,i)*S5(k,j);
	  S5(i,j)=x/sq;
	}
    }
  sotDEBUG(20) << "S = " <<S5  << endl;

  /* S=Inversion of a upper-triangular matrix A.
   *  (1) the diag of the inverse is the inverse of the diag: s_ii = 1/a_ii.
   *
   *  (2) Then cover the matrix by writting the lines from the diag.
   * For each element i,j, knowing all elements i,k (k<j) of the line i,
   * the element s_ij is obtained by developing the term p_ij of the product
   * S*A. p_ik = [S*A]_ik = 0 = sum(k=i:j) s_ik a_kj = 0. Then
   * s_ij = - [ sum(k=i:j-1) s_ik a_kj ] / a_ii
   */
  S5i.resize( A.rows(),A.rows() ); S5i.setZero();
  for( unsigned int l=0;l<A.rows();++l )
    for( unsigned int i=0;i<A.cols();++i )
      {
 	unsigned int j=i+l; if(j>=A.cols()) continue;
 	if( j==i ) S5i(j,j)=1/S5(j,j);
 	else
 	  {
 	    S5i(i,j)=0;
 	    for( unsigned int k=i;k<j;k++ )
 	      {	S5i(i,j) -= S5(k,j)*S5i(i,k);    }
 	    S5i(i,j)/=S5(j,j);
 	  }
      }
  sotDEBUG(15) << "Si = " <<S5i  << endl;

  sotDEBUGOUT(15);
  return S5i;
}


/* --------------------------------------------------------------------- */
/* --- CONTROL --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

dynamicgraph::Matrix& WeightedSot::
computeConstrainedWeight( dynamicgraph::Matrix& KAK,const int& iterTime )
{
  sotDEBUGIN(15);

  const dynamicgraph::Matrix &K = constraintSOUT(iterTime);
  const dynamicgraph::Matrix &A = weightSIN(iterTime);
  const unsigned int mJ = K.cols();

//   const unsigned int rhand=28, lhand=40;
//   for( unsigned int i=0;i<6;++i )
//     for( unsigned int j=0;j<mJ+6;++j )
//       {
//  	if( i+rhand==j ) A(j,j)=1; else
// 	  A(i+rhand,j) = A(j,rhand+i) = 0.;
//  	if( i+lhand==j ) A(j,j)=1; else
// 	  A(i+lhand,j) = A(j,lhand+i) = 0.;
//       }

  sotDEBUG(35) << "Kff = " << K  << endl;
  sotDEBUG(35) << "A = " << A  << endl;

  KAK.resize( mJ,mJ ); KAK.fill(0);
  {
    dynamicgraph::Matrix KA( mJ,6 ); KA.fill( 0. );
    for( unsigned int i=0;i<mJ;++i )
      for( unsigned int j=0;j<6;++j )
	for( unsigned int k=0;k<6;++k )
	  KA(i,j) += K(k,i)*A(k,j);

    for( unsigned int i=0;i<mJ;++i )
      for( unsigned int j=0;j<mJ;++j )
	{
	  for( unsigned int k=0;k<6;++k )
 	    {
	      KAK(i,j) += KA(i,k)*K(k,j);
	      const double x = A(i+6,k)*K(k,j);
	      KAK(i,j) += x;
	      KAK(j,i) += x;
	    }
	  KAK(i,j) += A(i+6,j+6);
	}
  }
  sotDEBUG(35) << "KAK = " << KAK  << endl;
  return KAK;
}
/* --------------------------------------------------------------------- */
/* --- CONTROL --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#ifndef WIN32
#include <sys/time.h>
#endif /*WIN32*/

//#define WITH_CHRONO

#ifdef  WITH_CHRONO
#   define sotINIT_CHRONO1 struct timeval t0,t1; double dt
#   define sotSTART_CHRONO1  gettimeofday(&t0,NULL)
#   define sotCHRONO1 \
      gettimeofday(&t1,NULL);\
      dt = ( (t1.tv_sec-t0.tv_sec) * 1000.\
	     + (t1.tv_usec-t0.tv_usec+0.) / 1000. );\
      sotDEBUG(1) << "dt: "<< dt
#   define sotINITPARTCOUNTERS  struct timeval tpart0
#   define sotSTARTPARTCOUNTERS  gettimeofday(&tpart0,NULL)
#   define sotCOUNTER(nbc1,nbc2) \
	  gettimeofday(&tpart##nbc2,NULL); \
	  dt##nbc2 += ( (tpart##nbc2.tv_sec-tpart##nbc1.tv_sec) * 1000. \
		   + (tpart##nbc2.tv_usec-tpart##nbc1.tv_usec+0.) / 1000. )
#   define sotINITCOUNTER(nbc1) \
   struct timeval tpart##nbc1; double dt##nbc1=0;
#else // #ifdef  WITH_CHRONO
#   define sotINIT_CHRONO1
#   define sotSTART_CHRONO1
#   define sotCHRONO1 if(1) ; else std::cout
#   define sotINITPARTCOUNTERS
#   define sotSTARTPARTCOUNTERS
#   define sotCOUNTER(nbc1,nbc2)
#   define sotINITCOUNTER(nbc1)
#endif // #ifdef  WITH_CHRONO



dynamicgraph::Vector& WeightedSot::
computeWeightedControlLaw( dynamicgraph::Vector& control,const int& iterTime )
{
  sotDEBUGIN(15);
  sotDEBUGF(5, " --- Time %d -------------------", iterTime );

  sotINIT_CHRONO1; sotINITPARTCOUNTERS;
  sotINITCOUNTER(0b);
  sotINITCOUNTER(1); sotINITCOUNTER(2);sotINITCOUNTER(3);sotINITCOUNTER(4);
  sotINITCOUNTER(5); sotINITCOUNTER(6);sotINITCOUNTER(7);
  sotINITCOUNTER(8); sotINITCOUNTER(9);
  sotINITCOUNTER(4a); sotINITCOUNTER(4b); sotINITCOUNTER(4c); sotINITCOUNTER(4d);sotINITCOUNTER(4e);

  sotSTART_CHRONO1;
  sotSTARTPARTCOUNTERS;

  dynamicgraph::Matrix P;
  const double &th = inversionThresholdSIN(iterTime);
  const dynamicgraph::Matrix &K = constraintSOUT(iterTime);
  const unsigned int mJ = K.cols();
  //dynamicgraph::Matrix A = weightSIN(iterTime);
  const dynamicgraph::Matrix &S5i = squareRootInvWeightSIN(iterTime);
  dynamicgraph::Matrix Ai(mJ,mJ);
  Ai = S5i.multiply*(S5i.transpose()); // TODO: Optimize by considering the triangular shape!
  sotDEBUG(35) << "Ai = " << Ai << endl;

  /* --- Q0 --- */
  /* --- Q0 --- */
  try {
    control = q0SIN( iterTime );
    sotDEBUG(15) << "initial velocity q0 = " << control << endl;
    if( mJ!=control.size() ) { control.resize( mJ ); control.fill(.0); }
  }
  catch (...)
    {
      if( mJ!=control.size() ) { control.resize( mJ ); }
      control.setZero();
      sotDEBUG(25) << "No initial velocity." <<endl;
    }


  /* --- Main Loop --- */
  /* --- Main Loop --- */
  unsigned int iterTask = 0;
  for( StackType::iterator iter = stack.begin(); iter!=stack.end();++iter )
    {
      sotCOUNTER(0,0b); // Direct Dynamic
      sotDEBUGF(5,"Rank %d.",iterTask);
      TaskAbstract & task = **iter;
      const dynamicgraph::Matrix &JacRO = task.jacobianSOUT(iterTime);
      const dynamicgraph::Vector err = Sot::taskVectorToMlVector(task.taskSOUT(iterTime));
      const unsigned int nJ = JacRO.rows();
      sotCOUNTER(0b,1); // Direct Dynamic

      /* Init memory. */
      MemoryTaskSOT * mem = dynamic_cast<MemoryTaskSOT *>( task.memoryInternal );
      if( NULL==mem )
        {
          if( NULL!=task.memoryInternal ) delete task.memoryInternal;
          mem = new MemoryTaskSOT( task.getName()+"_memSOT",nJ,mJ );
          task.memoryInternal = mem;
        }

      dynamicgraph::Matrix Jp,V; unsigned int rankJ;
      dynamicgraph::Matrix Jac(nJ,mJ);
      dynamicgraph::Matrix Jt(nJ,mJ);

      if( (task.jacobianSOUT.getTime()>mem->jacobianInvSINOUT.getTime())
	  ||(mem->jacobianInvSINOUT.accessCopy().rows()!=mJ)
	  ||(mem->jacobianInvSINOUT.accessCopy().cols()!=nJ)
	  ||(task.jacobianSOUT.getTime()>mem->jacobianConstrainedSINOUT.getTime())
	  ||(task.jacobianSOUT.getTime()>mem->rankSINOUT.getTime())
	  ||(task.jacobianSOUT.getTime()>mem->singularBaseImageSINOUT.getTime()) )
      {
	sotDEBUG(15) <<"Recompute inverse."<<endl;

	/* --- FIRST ALLOCS --- */
	const unsigned int FF_SIZE=JacRO.cols()-mJ;
	dynamicgraph::Matrix Jff( nJ,FF_SIZE );
	dynamicgraph::Matrix Jact( nJ,mJ );
	Jp.resize( mJ,nJ );
	/***/sotCOUNTER(1,2); // first allocs

	/* --- COMPUTE JK --- */
	for( unsigned int i=0;i<nJ;++i )
	  {
	    for( unsigned int j=0;j<FF_SIZE;++j ) Jff(i,j)=JacRO(i,j);
	    for( unsigned int j=FF_SIZE;j<JacRO.cols();++j )
	      Jact(i,j-FF_SIZE)=JacRO(i,j);
	  }
	Jac = Jff*K;
	Jac+=Jact;
	/***/sotCOUNTER(2,3); // compute JK
	
	/* --- COMPUTE Jt --- */
	if( mJ==P.cols() ) Jt = Jac*P;
	else { Jt = Jac; }
	/***/sotCOUNTER(3,4); // compute Jt
	
	/* --- COMPUTE S --- */	
	dynamicgraph::Matrix Kact(mJ,mJ); Kact=S5i;
	Task* taskSpec = dynamic_cast<Task*>( &task );
	if( NULL!=taskSpec )
	  {
	    const Flags& controlSelec = taskSpec->controlSelectionSIN( iterTime );
	    sotDEBUG(25) << "Control selection = " << controlSelec <<endl;
	    if( controlSelec )
	      {
		if(!controlSelec)
		  {
		    /***/sotCOUNTER(4,4a); // compute Kh
		    sotDEBUG(15) << "Control selection."<<endl;
		    std::vector<unsigned int> unactiveList;
		    sotDEBUG(25) << "selec"<<iterTask<<" = [ ";
		    for( unsigned int i=0;i<mJ;++i )
		      {
			if(! controlSelec(i) )
			  {
			    if(sotDEBUG_ENABLE(25)) sotDEBUGFLOW.outputbuffer << i <<" ";
			    unactiveList.push_back(i);
			  }
		      }
		    if(sotDEBUG_ENABLE(25)) sotDEBUGFLOW.outputbuffer << "]" << endl ;
		
		    const unsigned int unactiveSize = unactiveList.size();
		    /***/sotCOUNTER(4a,4b); // compute Kh
	
		    /* Q = H*Ai*H, H being the unactivation matrix. */
		    dynamicgraph::Matrix Q(unactiveSize,unactiveSize);
		    dynamicgraph::Matrix Sir(unactiveSize,mJ);
		    for( unsigned int i=0;i<unactiveSize;++i )
		     {
		       for( unsigned int j=0;j<unactiveSize;++j )
			 Q(i,j)=Ai(unactiveList[i],unactiveList[j]);
		
		       for( unsigned int j=0;j<mJ;++j )
			 Sir(i,j) = S5i(unactiveList[i],j);
		     }
		    sotDEBUG(25) << "Q"<<iterTask<<" = " << Q << endl;
		    sotDEBUG(25) << "Sr"<<iterTask<<" = " << Sir << endl;
		    /***/sotCOUNTER(4b,4c); // compute Kh

		    /* Qi = inv(Q), always positive. */
		    dynamicgraph::Matrix Qi(unactiveSize,unactiveSize);
		    Q.inverse(Qi);
		    sotDEBUG(25) << "Qi"<<iterTask<<" = " << Qi << endl;
		    /***/sotCOUNTER(4c,4d); // compute Kh

		    /* Kact = (HSi)^+ HSi = Si_red^T Qi Si_red. */
		    dynamicgraph::Matrix SrQi(mJ,unactiveSize);
		    dynamicgraph::Matrix SrQiSr(mJ,mJ);
		    dynamicgraph::Matrix ArQiSr(mJ,mJ);
		    SrQi = (Sir.transpose())*Qi;
		    SrQiSr = SrQi*Sir;
		    ArQiSr = S5i*SrQiSr;
		    Kact-=ArQiSr; // TODO: optimizer ce merdier! Kact est sparse!

		    sotDEBUG(15) << "Kact"<<iterTask<<" = "<<Kact<<endl;
		    /***/sotCOUNTER(4d,4e); // compute Kh
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
	/***/sotCOUNTER(4,5); // compute Kh

	/* --- PINV --- */
	dynamicgraph::Matrix U; dynamicgraph::Vector S;
	dynamicgraph::Matrix JtA( Jt.rows(),Jt.cols() );
	dynamicgraph::Matrix JtAp( Jt.cols(),Jt.rows() );

	JtA = Jt * Kact;
	JtA.pseudoInverse( JtAp,th,&U,&S,&V );
	Jp = S5i * JtAp; // TODO: S5i*JtAp = Kact*JtAp, and Kact is sparse.

	if(  mJ==P.cols() ) Jp = P*Jp; // P is not idempotent
	/***/sotCOUNTER(5,6); // PINV

	/* --- RANK --- */
	{
	  const unsigned int Jmax = S.size(); rankJ=0;
	  for( unsigned i=0;i<Jmax;++i ) { if( S(i)>th ) rankJ++; }
	}

	sotDEBUG(45) << "control"<<iterTask<<" = "<<control<<endl;
	sotDEBUG(25) << "J"<<iterTask<<" = "<<JacRO<<endl;
	sotDEBUG(25) << "JK"<<iterTask<<" = "<<Jac<<endl;
	sotDEBUG(25) << "Jt"<<iterTask<<" = "<<Jt<<endl;
	sotDEBUG(25) << "JtA"<<iterTask<<" = "<<JtA<<endl;
	sotDEBUG(15) << "Jp"<<iterTask<<" = "<<Jp<<endl;
	sotDEBUG(15) << "e"<<iterTask<<" = "<<err<<endl;
	sotDEBUG(45) << "JJp"<<iterTask<<" = "<< Jac*Jp <<endl;
	sotDEBUG(45) << "U"<<iterTask<<" = "<< U<<endl;
	sotDEBUG(45) << "Svd"<<iterTask<<" = "<< S<<endl;
	sotDEBUG(45) << "V"<<iterTask<<" = "<< V<<endl;

	mem->jacobianInvSINOUT = Jp;
	mem->jacobianInvSINOUT.setTime( iterTime );
	mem->jacobianConstrainedSINOUT = Jac;
	mem->jacobianConstrainedSINOUT.setTime( iterTime );
	mem->singularBaseImageSINOUT = V;
	mem->singularBaseImageSINOUT.setTime( iterTime );
	mem->rankSINOUT = rankJ;
	mem->rankSINOUT.setTime( iterTime );

	sotDEBUG(25)<<"Inverse recomputed."<<endl;

      } else {
	sotDEBUG(15)<<"Inverse not recomputed."<<endl;
	rankJ = mem->rankSINOUT.accessCopy();
	Jac = mem->jacobianConstrainedSINOUT.accessCopy();
	Jp = mem->jacobianInvSINOUT.accessCopy();
	V = mem->singularBaseImageSINOUT.accessCopy();
      }

	/***/sotCOUNTER(6,7); // TRACE

      /* --- COMPUTE QDOT AND P --- */
      /*DEBUG*/// if( iterTask==0 ) control += Jp*err; else
      //Jp*err;
      sotDEBUG(15) << "G = " << Jp<<endl;
      sotDEBUG(15) << "e = " << err<<endl;
      sotDEBUG(15) << "Ge = " << Jp*err<<endl;
      sotDEBUG(15) << "d = " << Jac*control<<endl;
      control += Jp*(err - Jac*control);
      /***/sotCOUNTER(7,8); // QDOT


      if( mJ!=P.cols() ) { P.resize( mJ,mJ ); P.setIdentity(); }
      {
	dynamicgraph::Matrix Kp( mJ,mJ );
	Kp = Jp*Jt;
	P -= Kp;
      }
      sotCOUNTER(8,9); // Projo

      sotDEBUG(15) << "q"<<iterTask<<" = "<<control<<std::endl;
      sotDEBUG(25) << "P"<<iterTask<<" = "<< P <<endl;
#ifdef WITH_CH
      sotDEBUG(1) << "Chrono"<<iterTask<<"_"<<iterTime<<" = [ " << endl
		  <<" "<<dt0b<<" "<<dt1 <<" "<< dt2 <<" "<< dt3 <<" "
		  << dt4 <<" "<<dt5 <<", "<< dt6 <<" "<< dt7
		  <<" "<< dt8 <<" "<< dt9 << " ] " << endl;
      sotDEBUG(1) << "ChronoKh"<<iterTask<<"_"<<iterTime<<" = [ " << endl
		  <<" "<< dt4a<<" "<< dt4b<<" "<< dt4c<<" "
		  << dt4d<<" "<< dt4e<<" " << " ] " << endl;
#endif // #ifdef WITH_CHRONO
      iterTask++;
    }

  sotCHRONO1 << endl;


  sotDEBUGOUT(15);
  return control;
}
