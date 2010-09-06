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

/* SOT */
#include <sot-core/debug.h>
//class sotSOTQr__INIT
//{
//public:sotSOTQr__INIT( void ) { DebugTrace::openFile(); }
//};
//sotSOTQr__INIT sotSOTQr_initiator;


#include <sot-core/sot-qr.h>
#include <sot-core/pool.h>
#include <sot-core/task.h>
#include <sot-core/sot.h>
#include <sot-core/memory-task-sot.h>
#include <sot-core/debug.h>

#define FORTRAN_ID( id ) id##_
using namespace std;
using namespace sot;
using namespace dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot-core/factory.h>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SotQr,"SOTQr");


const double SotQr::INVERSION_THRESHOLD_DEFAULT = 1e-4;
const unsigned int SotQr::NB_JOINTS_DEFAULT = 46;

/* --------------------------------------------------------------------- */
/* --- CONSTRUCTION ---------------------------------------------------- */
/* --------------------------------------------------------------------- */
SotQr::
SotQr( const std::string& name )
  :Entity(name)
  ,stack()
  ,constraintList()
  ,ffJointIdFirst( FF_JOINT_ID_DEFAULT )
  ,ffJointIdLast( FF_JOINT_ID_DEFAULT+6 )

  ,nbJoints( NB_JOINTS_DEFAULT )
  ,taskGradient(0)
  ,q0SIN( NULL,"sotSOTQr("+name+")::input(double)::q0" )
   ,inversionThresholdSIN( NULL,"sotSOTQr("+name+")::input(double)::damping" )
   ,constraintSOUT( boost::bind(&SotQr::computeConstraintProjector,this,_1,_2),
		   sotNOSIGNAL,
		    "sotSOTQr("+name+")::output(matrix)::constraint" )
  ,controlSOUT( boost::bind(&SotQr::computeControlLaw,this,_1,_2),
		constraintSOUT<<inversionThresholdSIN<<q0SIN,
		"sotSOTQr("+name+")::output(vector)::control" )
{
  inversionThresholdSIN = INVERSION_THRESHOLD_DEFAULT;

  signalRegistration( inversionThresholdSIN<<controlSOUT<<constraintSOUT<<q0SIN );
}

/* --------------------------------------------------------------------- */
/* --- STACK MANIPULATION --- */
/* --------------------------------------------------------------------- */
void SotQr::
push( TaskAbstract& task )
{
  stack.push_back( &task );
  controlSOUT.addDependency( task.taskSOUT );
  controlSOUT.addDependency( task.jacobianSOUT );
  //controlSOUT.addDependency( task.featureActivationSOUT );
  controlSOUT.setReady();
}
TaskAbstract& SotQr::
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
bool SotQr::
exist( const TaskAbstract& key )
{
  std::list<TaskAbstract*>::iterator it;
  for ( it=stack.begin();stack.end()!=it;++it )
    {
      if( *it == &key ) { return true; }
    }
  return false;
}
void SotQr::
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

void SotQr::
removeDependency( const TaskAbstract& key )
{
  controlSOUT.removeDependency( key.taskSOUT );
  controlSOUT.removeDependency( key.jacobianSOUT );
  //controlSOUT.removeDependency( key.featureActivationSOUT );
  controlSOUT.setReady();
}

void SotQr::
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
void SotQr::
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

void SotQr::
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

void SotQr::
addConstraint( Constraint& constraint )
{
  constraintList.push_back( &constraint );
  constraintSOUT.addDependency( constraint.jacobianSOUT );
}

void SotQr::
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
void SotQr::
clearConstraint( void )
{
  for (  ConstraintListType::iterator it=constraintList.begin();
	 constraintList.end()!=it;++it )
    {
      constraintSOUT.removeDependency( (*it)->jacobianSOUT );
    }
  constraintList.clear();
}

void SotQr::
defineFreeFloatingJoints( const unsigned int& first,const unsigned int& last )
{
  ffJointIdFirst = first ;
  if( last>0 ) ffJointIdLast=last ;
  else ffJointIdLast=ffJointIdFirst+6;
}


/* --------------------------------------------------------------------- */
/* --- CONTROL --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


namespace bub = boost::numeric::ublas;
typedef bub::vector<double> bubVector;
typedef bub::matrix<double> bubMatrix;

//std::ostringstream MATLAB;
class MATLAB
{
public:
  std::string str;
  friend std::ostream & operator << (std::ostream & os, const MATLAB & m )
  {return os << m.str; }

  MATLAB( const bubVector& v1 )
  {
    std::ostringstream os; os << "[ ";
    for( unsigned int i=0;i<v1.size();++i )
      {
	os <<  v1(i);
	if( v1.size()!=i+1 ) { os << ", "; }
	else { os << "]"; }
      }
    str = os.str();
  }
  MATLAB( const bubMatrix& m1)
  {
    std::ostringstream os; os << "[ ";
    for( unsigned int i=0;i<m1.size1();++i )
      {
	for( unsigned int j=0;j<m1.size2();++j )
	  {
	    os <<  m1(i,j);
	    if( m1.size2()!=j+1 ) os << ", ";
	  }
	if( m1.size1()!=i+1 ) { os << " ;" << std::endl; }
	else { os << "  ]"; }
      }
    str = os.str();
  }
};

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
#define LAPACK_DGEQP3 FORTRAN_ID( dgeqp3 )
#define LAPACK_DGEQPF FORTRAN_ID( dgeqpf )
extern "C" {
  void LAPACK_DGEQP3( const int* m, const int* n, double* a, const int* lda, int * jpvt,
                      double* tau, double* work, const int* lwork, int* info );
  void LAPACK_DGEQPF( const int* m, const int* n, double* a, const int* lda, int * jpvt,
                      double* tau, double* work, int* info );
};

namespace boost { namespace numeric { namespace bindings { namespace lapack
{

  inline int geqp (bub::matrix<double,bub::column_major> &A,
		   bub::vector< int >& jp, bubVector& tau)
  {
    const int m = A.size1();
    const int n = A.size2();
    ::boost::numeric::ublas::vector<double> work(std::max(1, n*32));

    assert (std::min(m,n) <= tau.size());
    assert (n <= work.size());

    int const mF= A.size1();
    int const nF= A.size2();
    double* aF = MRAWDATA(A);
    int const ldaF = traits::leading_dimension (A);
    int * jpvtF = VRAWDATA(jp);
    double * tauF = VRAWDATA(tau);
    double * workF = VRAWDATA(work);
    int const lworkF =  work.size();
    int infoF;

    //     std::cout<<"lda = " << ldaF << std::endl;
    //     std::cout << "mF = " << mF << std::endl;
    //     std::cout << "nF = " << nF << std::endl;
    //     //std::cout << "aF  = " << aF  << std::endl;
    //     std::cout << "ldaF  = " << ldaF  << std::endl;
    //     //std::cout << "jpvtF  = " << jpvtF  << std::endl;
    //     //std::cout << "tauF  = " << tauF  << std::endl;
    //     //std::cout << "workF  = " << workF  << std::endl;
    //     std::cout << "lworkF  = " << lworkF  << std::endl;
    //     std::cout << "infoF  " << infoF << std::endl;


    LAPACK_DGEQP3(&mF, &nF, aF, &ldaF, jpvtF, tauF, workF, &lworkF, &infoF);
    //LAPACK_DGEQPF(&mF, &nF, aF, &ldaF, jpvtF, tauF, workF, &infoF);
    return infoF;
  }

      }}}}

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */

/* InvChowleski update. */
/* We have to compute a full decomposition of Rx Rx', where
 * Rx = [R x ], R being triangular up, and x being any vector.
 * The output should be T triangular up so that Rx Rx' = [ TT' 0; 0 0 ].
 * This is done by nullifying the last line of Rx', by successive
 * given rotation.
 */


/* Xn gives the range of the vector to consider when Rx = [ R X ] = [ R Y x Z ].
 * In that case, x=X(:,xn), starting from 0 (xn=0 -> x is the first
 * column of X).
 * TODO: optimize, considering that R is triangular!
 */
template< typename bubTemplateMatrix >
void invCholeskiUpdate( bubTemplateMatrix & Rx, const unsigned int xn=0 )
{
  const unsigned int n = Rx.size1();

  double c,s,tau;
  for( unsigned int iter =0; iter<n;iter++ )
    {
      //std::cout << "Rx" << iter << " = " << (MATLAB)Rx << std::endl << std::endl;

      const unsigned int i=n-iter-1;
      const unsigned int j=n-iter-1;
      const unsigned int k=n+xn;

      const double &a=Rx(j,i);
      const double &b=Rx(j,k);
      if( fabs(b)>fabs(a) )
	{ tau=-a/b; s=1/sqrt(1+tau*tau); c=s*tau; }
      else
	{ tau=-b/a; c=1/sqrt(1+tau*tau); s=c*tau; }

      for( unsigned int row=0;row<n-iter;++row )
	{
	  const double ri = Rx(row,i);
	  const double rk = Rx(row,k);
	  Rx(row,i) = ri*c-rk*s;
	  Rx(row,k) = ri*s+rk*c;
	}
    }
  //std::cout << "Rt = " << (MATLAB)Rx << std::endl << std::endl;
}

/* Same, but with a collection of vector instead of only one. */
template< typename bubTemplateMatrix >
void invGeneralizeCholeskiUpdate( bubTemplateMatrix & Rx )
{

  const unsigned int n = Rx.size1();
  const unsigned int xn = Rx.size2()-n;
  //std::cout << "Cho init " << Rx<< std::endl;
  //std::cout << "Cho init " << n << ":" << xn << std::endl;
  for( unsigned int i=0;i<xn;++i )
    {
      invCholeskiUpdate(Rx,i);
    }
}

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
class sotHouseholdMatrix
{
public: // protected:
  bubVector v;
  double beta;

public:
  sotHouseholdMatrix( void ) { v.resize(0);}
  sotHouseholdMatrix( const unsigned int n )
  { v.resize(n); std::fill(v.begin(),v.end(),0); beta=0;}
  sotHouseholdMatrix( const bubVector& _v, const double& _beta )
  { v= _v; beta=_beta; }
  sotHouseholdMatrix( const bubMatrix& _v, const double& _beta )
  {
    v.resize(_v.size1());
    for( unsigned int i=0;i<_v.size1();++i )
      {v(i)= _v(i,0);}
    beta=_beta;
  }

public:

  /* TODO: vstar is computed with many 0. same for vstar^T.
   * The computation should be optimized using only A22 instead of
   * A = [A11 A12; A21 A22]; */
  void multiplyRight( bubVector& a ) const // P*a
  {
    const unsigned int m=a.size();
    const unsigned int kv = v.size();
    const unsigned int m_kv = m-kv;

    double w; w=0;
    // w <- b v' A = b [ 0 ... 0 1 v' ] a
    for( unsigned int i=0;i<kv;++i )
      { w += v(i)*a(i+m_kv); }
    w*=beta;
    // a <- a - v w = [ 0; ... ; 0;  1; v ] w
    for( unsigned int i=0;i<kv;++i )
      { a(i+m_kv)-=v(i)*w; }
  }

  void multiplyRight( bubMatrix& A ) const // P*A
  {
    const unsigned int m=A.size1(), n=A.size2();
    const unsigned int kv = v.size();
    const unsigned int m_kv = m-kv;

    bubVector w(n); w*=0;
    // W <- b v' A = b [ 0 ... 0 1 v' ] A
    for( unsigned int j=0;j<n;++j )
      {
	for( unsigned int i=0;i<kv;++i )
	  { w(j) += v(i)*A(i+m_kv,j); }
	w(j)*=beta;
      }
    // A <- A - v w = [ 0; ... ; 0;  1; v ] w
    for( unsigned int i=0;i<kv;++i )
      for( unsigned int j=0;j<n;++j )
	{ A(i+m_kv,j)-=v(i)*w(j); }
  }


  friend std::ostream & operator << ( std::ostream & os,const sotHouseholdMatrix & P )
  {
    return os << "[b=" << P.beta << "] " << (MATLAB)P.v;
  }
};

class sotHouseholdMatrices
{
public: // protected:
  typedef std::list<sotHouseholdMatrix> HouseholdList;
  HouseholdList matrices;
  static const double THRESHOLD;

public:
  sotHouseholdMatrices( void ) { matrices.resize(0);}
  sotHouseholdMatrices( const bubMatrix & RQ, const bubVector & betas )
  { this->push_back(RQ,betas);  }

  void push_back( const sotHouseholdMatrix& P )
  { matrices.push_back( P ); }

  /* <nbVector> is the number of vector to consider. If nbVector<0, all
   * the householder vectors are added. */
  void push_back( const bub::matrix<double> & RQ,
		  const bubVector & betas,
		  const int nbVector=-1 )
  {
    const unsigned int nToProceed
      =((nbVector<0)
	?std::min(RQ.size1(),RQ.size2())
	:(unsigned int)nbVector);
    const unsigned int m=RQ.size1();

    //unsigned int i=0;
    //while( (i<n)&&(fabs(betas(i))>THRESHOLD) )

    for( unsigned int i=0;i<nToProceed;++i )

      {
	bubVector v(m-i);  v(0)=1;
	for( unsigned int j=1;j<m-i;++j ) v(j)=RQ(j+i,i);
	sotHouseholdMatrix(v,betas(i));
	this->push_back( sotHouseholdMatrix(v,betas(i)) );
      }
  }
  void push_back( const sotHouseholdMatrices& Q2 )
  {
    for( HouseholdList::const_iterator Pi = Q2.matrices.begin();
	 Pi!=Q2.matrices.end(); ++Pi )
      {   matrices.push_back( *Pi );   }
  }

  /* --- Vectors --- */
  virtual void multiplyRight( bubVector& a ) const // Q*a = P1*P2*...*Pn*a
  {
    for( HouseholdList::const_reverse_iterator Pi = matrices.rbegin();
	 Pi!=matrices.rend(); ++Pi )
      {	Pi->multiplyRight(a);      }
  }
  virtual void multiplyTransposeRight( bubVector& a ) const // Q*a = Pn*...*P2*P1*a
  {
    for( HouseholdList::const_iterator Pi=matrices.begin();
	 Pi!=matrices.end(); ++Pi )
      {	Pi->multiplyRight(a);      }
  }

  /* For both sub range product, I tried to optimize a little bit
   * the amount of computation that is done. However, when writing Q or Q'
   * as a product of P's, it is only possible to save computations on
   * the very first product. All the other P's have to be multiply in full.
   * Consequently, I will not waste any more time to save some 4 or 5
   * double product. */
  virtual void multiplyRangeRight( const bubVector& a, // N*A = Q*[0;I]*A = P1*...*Pn*[0;A]
				   bubVector& res,
				   const unsigned int zeroBefore,
				   const unsigned int zeroAfter ) const
  {
    const unsigned int m=a.size(); //n =1;
    const unsigned int mpp=m+zeroBefore+zeroAfter;
    bubVector &app = res; app.resize( mpp ); app.assign(bub::zero_vector<double>(mpp));
    bub::vector_range< bubVector > acopy(app,bub::range(zeroBefore,zeroBefore+m));
    acopy.assign(a);
    multiplyRight(app);
  }

  /* Same as before, no optimization done here. */
  // N'*a = [0 I]*Q'*a = subrange( P1*...*Pn*A )
  virtual void multiplyTransposeRangeRight( const bubVector& a,
					    bubVector& res,
					    const unsigned int zeroBefore,
					    const unsigned int zeroAfter ) const
  {
    bubVector acopy = a; multiplyTransposeRight(acopy);
    const unsigned int mres = a.size() - zeroBefore - zeroAfter;
    //bub::matrix_range< bubMatrix > asub (acopy,bub::range(zeroBefore,mres));
    res.resize(mres); res.assign( bub::project( acopy,bub::range(zeroBefore,mres)) );
  }

  /* --- Matrices --- */
  virtual void multiplyRight( bubMatrix& A ) const // Q*A = P1*P2*...*Pn*A
  {
    for( HouseholdList::const_reverse_iterator Pi = matrices.rbegin();
	 Pi!=matrices.rend(); ++Pi )
      {	Pi->multiplyRight(A);      }
  }

  virtual void multiplyTransposeRight( bubMatrix& A ) const // Q*A = Pn*...*P2*P1*A
  {
    for( HouseholdList::const_iterator Pi = matrices.begin();
	 Pi!=matrices.end(); ++Pi )
      {	Pi->multiplyRight(A);      }
  }

  friend std::ostream & operator << ( std::ostream & os,const sotHouseholdMatrices & Ps )
  {
    for( HouseholdList::const_iterator Pi = Ps.matrices.begin();
	 Pi!=Ps.matrices.end(); ++Pi )
      {
	os << (*Pi) << std::endl;
      }
    if( Ps.matrices.empty() ) os << "Identity" << std::endl;
    return os;
  }

};



const double sotHouseholdMatrices::THRESHOLD = 1e-9;

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */

typedef bub::triangular_adaptor< bub::matrix_range< bub::matrix<double,boost::numeric::ublas::column_major> >, bub::upper> bubMatrixTriSquare;

/* Assuming a diagonal-ordered triangular matrix. */
unsigned int rankDetermination( const bubMatrix& A,
				const double threshold = 1e-6 )
{
  const unsigned int n = traits::leading_dimension (A);
  unsigned int res = 0;

  for( unsigned int i=0;i<n;++i )
    { if( fabs(A(i,i))>threshold ) res++; else break; }

  return res;
}
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */

#define WITH_CHRONO


#ifdef  WITH_CHRONO
	#ifndef WIN32
	#include <sys/time.h>
	#else /*WIN32*/
	#include <sot-core/utils-windows.h>
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

ml::Vector& SotQr::
computeControlLaw( ml::Vector& control,const int& iterTime )
{
  sotDEBUGIN(15);

  //sotINIT_CHRONO1;
  sotINITPARTCOUNTERS; sotINITCOUNTER(1); sotINITCOUNTER(2);sotINITCOUNTER(3);

  //sotSTART_CHRONO1;
  sotSTARTPARTCOUNTERS;

  //const double &th = inversionThresholdSIN(iterTime);
  const ml::Matrix &K = constraintSOUT(iterTime);
  const unsigned int nJ = K.nbCols();

  sotHouseholdMatrices Q;
  unsigned int freeRank = nJ;
  bubVector u(nJ); u.assign( bub::zero_vector<double>(nJ) );

  sotDEBUGF(5, " --- Time %d -------------------", iterTime );
  /* First stage. */
  {
    TaskAbstract & task = **(stack.begin());
    const ml::Matrix &Jac = task.jacobianSOUT(iterTime);
    const ml::Vector err = Sot::taskVectorToMlVector(task.taskSOUT(iterTime));
    const unsigned int mJ = Jac.nbRows();
    /***/sotCOUNTER(0,1); // Direct Dynamic

    /* --- INIT MEMORY --- */
    MemoryTaskSOT * mem = dynamic_cast<MemoryTaskSOT *>( task.memoryInternal );
    if( NULL==mem )
      {
        if( NULL!=task.memoryInternal ) delete task.memoryInternal;
        mem = new MemoryTaskSOT( task.getName()+"_memSOT",nJ,mJ );
        task.memoryInternal = mem;
      }
    /***/sotCOUNTER(1,2); // Direct Dynamic

    /* --- COMPUTE JK --- */
    ml::Matrix &JK = mem->JK;
    JK.resize( mJ,nJ );
    mem->Jff.resize( mJ,Jac.nbCols()-nJ );
    mem->Jact.resize( mJ,nJ );
    Sot::computeJacobianConstrained( task,K );
    /***/sotCOUNTER(2,3); // compute JK

    const bubMatrix & J1 = JK.accessToMotherLib();
    const bubVector & e1 = err.accessToMotherLib();
    const unsigned int & m1 = e1.size();

    /* --- Compute the QR decomposition. --- */
    bub::matrix<double,boost::numeric::ublas::column_major> QR1(nJ,m1);
    bubVector beta1(m1);
    bub::vector<int> perm1(m1);
    QR1 = bub::trans(J1);
    boost::numeric::bindings::lapack::geqp(QR1,perm1,beta1);
    const unsigned int & rank1 = rankDetermination(QR1);

    /* --- Organize into Q and R. --- */
    /* If J1 is rank deficient, then the last <m-r> householder vectors
     * do not affect the range basis M (where Q=[M N]). If M is constant
     * wrt these vector, then Span(N)=orth(Span(M)) is also constant, even
     * if N is non constant, and thus these vectors can be neglect. */
    Q.push_back( QR1,beta1,rank1 );
    if( rank1<m1 ) // Rank deficiency
      {
        bub::matrix_range< bub::matrix<double,boost::numeric::ublas::column_major> >
          QR1sup( QR1,bub::range(0,rank1),bub::range(0,m1) );
        invGeneralizeCholeskiUpdate( QR1sup );
      }
    bub::matrix_range< bub::matrix<double,boost::numeric::ublas::column_major> >
      QR1sup( QR1,bub::range(0,rank1),bub::range(0,rank1) );
    const bubMatrixTriSquare R1(QR1sup);

    /* --- Compute the control law. --- */
    bubVector b1(nJ);
    /* b <- J'e */
    bub::axpy_prod( e1,J1,b1,true );
    /* b <- Q' J' e */
    Q.multiplyTransposeRight(b1);
    /* Qe1 <- M' J' e */
    bub::vector_range<bubVector> Qe1(b1,bub::range(0,rank1));
    /* Qe1 <- R-1 M' J' e */
    bub::lu_substitute(R1,Qe1);
    /* Qe1 <- R'-1 R-1 M' J' e */
    bub::lu_substitute(Qe1,(const bubMatrix &)R1);
    /* b <- [ Qe1; 0 ] */
    bub::vector_range<bubVector> Ne1(b1,bub::range(rank1,nJ));
    Ne1.assign( bub::zero_vector<double>( nJ-rank1-1 ) );
    /* b <- Q [ Qe1;0] = M R'-1 R-1 M' J' e */
    Q.multiplyRight(b1);

    /* Ready for the next step... */
    u+=b1;
    freeRank-=rank1;

    /* --- Traces --- */
    sotDEBUG(45) << " * --- Stage 0 ----------------------- *" << std::endl;
    sotDEBUG(15) << "J1 = "    << (MATLAB)J1 << std::endl;
    sotDEBUG(15) << "e1 = "    << (MATLAB)e1 << std::endl;
    sotDEBUG(15) << "rank1 = " << rank1      << std::endl;
    sotDEBUG(45) << "Q1 = "    << Q;
    sotDEBUG(15) << "R1 = "    << (MATLAB)R1 << std::endl;
    sotDEBUG(45) << "du1 = "   << (MATLAB)b1 << std::endl;
    sotDEBUG(15) << "u1 = "    << (MATLAB)u  << std::endl;
  }

  unsigned int stage = 1;
  StackType::iterator iter = stack.begin();
  for( iter++; iter!=stack.end();++iter,stage++ )
    {
      sotDEBUG(45) << " * --- Stage " << stage << " ----------------------- *" << std::endl;

      TaskAbstract & task = **iter;
      const ml::Matrix &Jac = task.jacobianSOUT(iterTime);
      const ml::Vector err = Sot::taskVectorToMlVector(task.taskSOUT(iterTime));
      const unsigned int mJ = Jac.nbRows();
      /***/sotCOUNTER(0,1); // Direct Dynamic

      /* --- INIT MEMORY --- */
      MemoryTaskSOT * mem = dynamic_cast<MemoryTaskSOT *>( task.memoryInternal );
      if( NULL==mem )
        {
          if( NULL!=task.memoryInternal ) delete task.memoryInternal;
          mem = new MemoryTaskSOT( task.getName()+"_memSOT",nJ,mJ );
          task.memoryInternal = mem;
        }
      ml::Matrix &JK = mem->JK;

      /* --- COMPUTE JK --- */
      JK.resize( mJ,nJ );
      mem->Jff.resize( mJ,Jac.nbCols()-nJ );
      mem->Jact.resize( mJ,nJ );
      Sot::computeJacobianConstrained( task,K );
      /***/sotCOUNTER(2,3); // compute JK

      const bubMatrix & J2 = JK.accessToMotherLib();
      const bubVector & e2 = err.accessToMotherLib();
      const unsigned int & m2 = e2.size();

      /* --- Compute the limited matrix. --- */
      bubMatrix QJt2( nJ,m2 ); QJt2.assign( bub::trans(J2) );
      Q.multiplyTransposeRight(QJt2);
      bub::matrix_range<bubMatrix> Jt2( QJt2,bub::range(nJ-freeRank,nJ),bub::range(0,m2));

    /* --- Compute the QR decomposition. --- */
    bub::matrix<double,boost::numeric::ublas::column_major> QR2(freeRank,m2);
    bubVector beta2(m2);
    bub::vector<int> perm2(m2);
    QR2.assign(Jt2);
    boost::numeric::bindings::lapack::geqp(QR2,perm2,beta2);
    const unsigned int & rank2 = rankDetermination(QR2);

    /* --- Organize into Q and R. --- */
    sotHouseholdMatrices Q2;
    Q2.push_back( QR2,beta2,rank2 );
    if( rank2<m2 ) // Rank deficiency
      {
        bub::matrix_range< bub::matrix<double,boost::numeric::ublas::column_major> >
          QR2sup( QR2,bub::range(0,rank2),bub::range(0,m2) );
	invGeneralizeCholeskiUpdate( QR2sup );
     }
    bub::matrix_range< bub::matrix<double,boost::numeric::ublas::column_major> >
      QR2sup( QR2,bub::range(0,rank2),bub::range(0,rank2) );
    const bubMatrixTriSquare R2(QR2sup);

    /* --- Compute the control law. --- */
    bubVector b2(freeRank);
    bubVector u2(nJ);

    /* et <- e - J2'u1 */
    bubVector et2( m2 ); et2.assign( e2 );
    bub::axpy_prod( J2,-u,et2,false );
    /* b <- Jt'et */
    bub::axpy_prod( Jt2,et2,b2,true );
    /* b <- Q' Jt' et */
    Q2.multiplyTransposeRight(b2);
    /* Qe2 <- M' Jt' et */
    bub::vector_range<bubVector> Qe2(b2,bub::range(0,rank2));
    /* Qe2 <- R-2 M' Jt' et */
    bub::lu_substitute(R2,Qe2);
    /* Qe2 <- R'-1 R-1 M' Jt' et */
    bub::lu_substitute(Qe2,(const bubMatrix &)R2);
    /* b <- [ Qe2; 0 ] */
    bub::vector_range<bubVector> Ne2(b2,bub::range(rank2,freeRank));
    Ne2.assign( bub::zero_vector<double>( freeRank-rank2-1 ) );
    /* b <- Q [ Qe2;0] = M R'-1 R-1 M' Jt' et */
    Q2.multiplyRight(b2);
    /* u2 <- N b = Q [0;I] b */
    Q.multiplyRangeRight( b2,u2,nJ-freeRank,0 );

    /* Increment. */
    u+=u2;
    freeRank -= rank2;
    Q.push_back(Q2);

    /* --- Traces --- */
    sotDEBUG(15) << "J"<<stage+1<<" = " << (MATLAB)J2 << std::endl;
    sotDEBUG(25) << "Jt"<<stage+1<<" = " << (MATLAB)Jt2 << std::endl;
    sotDEBUG(15) << "e"<<stage+1<<" = " << (MATLAB)e2 << std::endl;
    sotDEBUG(15) << "rank"<<stage+1<<" = " << rank2 << std::endl;

    sotDEBUG(25) << "QR"<<stage+1<<" = " << (MATLAB)QR2 << std::endl;
    sotDEBUG(45) << "Q"<<stage+1<<" = " << Q2;
    sotDEBUG(15) << "R"<<stage+1<<" = " << (MATLAB)R2 << std::endl;
    sotDEBUG(45) << "et"<<stage+1<<" = " << (MATLAB)et2 << std::endl;

    sotDEBUG(45) << "MRRMb"<<stage+1<<" = " << (MATLAB)b2 << std::endl;
    sotDEBUG(25) << "du"<<stage+1<<" = " << (MATLAB)u2 << std::endl;

    sotDEBUG(15) << "u"<<stage+1<<" = " << (MATLAB)u << std::endl;
    sotDEBUG(45) << "QA"<<stage+1<<" = " << Q << std::endl;
    sotDEBUG(25) << "freerank"<<stage+1<<" = " << freeRank << std::endl;


    }

  control.accessToMotherLib() = u;

#ifdef VP_DEBUG
  for( StackType::iterator iter = stack.begin(); iter!=stack.end();++iter )
    {
      TaskAbstract & task = **iter;
      MemoryTaskSOT * mem = dynamic_cast<MemoryTaskSOT *>( task.memoryInternal );
      const ml::Matrix &Jac = mem->JK;
      const ml::Vector &err = Sot::taskVectorToMlVector(task.taskSOUT.accessCopy());

      ml::Vector diffErr(err.size());
      Jac.multiply(control,diffErr);
      diffErr-=err;
      sotDEBUG(45)<<diffErr<<std::endl;
    }
#endif // #ifdef VP_DEBUG

  sotDEBUGOUT(15);
  return control;
}



ml::Matrix& SotQr::
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
void SotQr::
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
operator<< ( std::ostream& os,const SotQr& sot )
{
  sot.display(os);
  return os;
}

/* --------------------------------------------------------------------- */
/* --- COMMAND --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void SotQr::
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

	 << " - nbJoints <nb> "<<endl;
      Entity::commandLine( cmdLine,cmdArgs,os );
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
	  cmdArgs >> nbJoints;
	} else { os << "nbJoints = "<< nbJoints <<endl; }
      constraintSOUT.setReady();
      controlSOUT.setReady();
    }
  else if( cmdLine == "display")
    {
      display(os);
    }
  else
    Entity::commandLine( cmdLine,cmdArgs,os );


  sotDEBUGOUT(15);
}

std::ostream& SotQr::
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
