
#ifndef __SOT_sotRotationSimple_HH__
#define __SOT_sotRotationSimple_HH__


/* --- BOOST --- */
#include <MatrixAbstractLayer/boost.h>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <sot-core/debug.h>
#include <sot-core/sot-core-api.h>

#include <list>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace bub = boost::numeric::ublas;
typedef bub::vector<double> bubVector;
typedef bub::matrix<double> bubMatrix;

template< class bubTemplateMatrix >
void bubClear( const bubTemplateMatrix& m )
{
  //std::fill(m.begin(),m.end(),0.0);
}


template< class bubTemplateMatrix >
void bubClearMatrix( const bubTemplateMatrix& m )
{
  //m
}

/* --- DEBUG ---------------------------------------------------------------- */
/* --- DEBUG ---------------------------------------------------------------- */
/* --- DEBUG ---------------------------------------------------------------- */
class RotationSimple;
class SOT_CORE_EXPORT MATLAB
{
 public:
  static bool fullPrec;
  std::string str;
  friend std::ostream & operator << (std::ostream & os, const MATLAB & m )
    {return os << m.str; }

  template< typename bubTemplateMatrix >
    void initFromBubMatrix( const bubTemplateMatrix& m1)
    {
      fullPrec=false;
      std::ostringstream os; os << "[ ";
      std::ostringstream ostmp;
      for( unsigned int i=0;i<m1.size1();++i )
        {
          for( unsigned int j=0;j<m1.size2();++j )
            {
              if( m1(i,j)<0 ) ostmp << "-"; else ostmp << " ";
              if(fullPrec||fabs(m1(i,j))>1e-6) ostmp <<  fabs(m1(i,j));
              else { ostmp << "0"; }
              if( m1.size2()!=j+1 )
                {
                  ostmp << ",";
                  const int size = ostmp.str().length();
                  for( unsigned int i=size;i<8;++i) ostmp<<" ";
                  ostmp << "\t";
                }
              os << ostmp.str(); ostmp.str("") ;
            }
          if( m1.size1()!=i+1 ) { os << " ;" << std::endl<<"  "; }
          else { os << "  ];"; }
        }
      str = os.str();
    }

  MATLAB( const bubVector& v1 )
    {
      std::ostringstream os; os << "[ ";
      for( unsigned int i=0;i<v1.size();++i )
        {
          {os << " "; double a=v1(i);  os << a;}
          //DEBUGos <<  v1(i);
          if( v1.size()!=i+1 ) { os << ", "; }
        }
      os << "]';";
      str = os.str();
    }


  MATLAB( const bub::indirect_array<>& v1 )
    {
      std::ostringstream os; os << "[ ";
      for( unsigned int i=0;i<v1.size();++i )
        {
          os <<  v1(i);
          if( v1.size()!=i+1 ) { os << ", "; }
        }
      os << "]";
      str = os.str();
    }


  MATLAB( const bubMatrix& m1)
    {initFromBubMatrix(m1);}

  MATLAB( const RotationSimple& m1,const unsigned int nJ );

};

template< typename bubTemplateMatrix >
void randMatrix( bubTemplateMatrix& M,const unsigned int row,const unsigned int col )
{
  M.resize(row,col);
  for( unsigned int i=0;i<row;++i )
    for( unsigned int j=0;j<col;++j )
      { M(i,j) = ((rand()+0.0)/RAND_MAX*2)-1.; }
}
template< typename bubTemplateVector >
void randVector( bubTemplateVector& M,const unsigned int row)
{
  M.resize(row);
  for( unsigned int i=0;i<row;++i )
      { M(i) = ((rand()+0.0)/RAND_MAX*2)-1.; }
}

/* ---------------------------------------------------------- */
/* --- SIMPLE ROTATION -------------------------------------- */
/* ---------------------------------------------------------- */

#define SOT_ROTATION_SIMPLE_MULTIPLY(MATRIX_TYPE)          \
  virtual void multiplyRight( MATRIX_TYPE& M ) const = 0; /* M:=P.M */  \
  virtual void multiplyLeft( MATRIX_TYPE& M ) const= 0;   /* M:=M.P */
#define SOT_ROTATION_SIMPLE_MULTIPLY_TRANSPOSE(MATRIX_TYPE)             \
  virtual void multiplyRightTranspose( MATRIX_TYPE& M ) const = 0; /* M:=P'.M */  \
  virtual void multiplyLeftTranspose( MATRIX_TYPE& M ) const= 0;   /* M:=M.P' */

/* Define the virtual link with template operators. */
#define SOT_ROTATION_DERIVATED_MULTIPLY_VECTOR(VECTOR_TYPE)     \
  virtual void multiplyRight( VECTOR_TYPE& M ) const  \
  { multiplyRightVectorTemplate( M ); }   \
  virtual void multiplyLeft( VECTOR_TYPE& M ) const   \
  { multiplyLeftVectorTemplate( M ); }
#define SOT_ROTATION_DERIVATED_MULTIPLY_MATRIX(MATRIX_TYPE) \
  virtual void multiplyRight( MATRIX_TYPE& M ) const  \
  { multiplyRightMatrixTemplate( M ); }   \
  virtual void multiplyLeft( MATRIX_TYPE& M ) const   \
  { multiplyLeftMatrixTemplate( M ); } \
  virtual void multiplyRightTranspose( MATRIX_TYPE& M ) const  \
  { multiplyRightTransposeMatrixTemplate( M ); }   \
  virtual void multiplyLeftTranspose( MATRIX_TYPE& M ) const   \
  { multiplyLeftTransposeMatrixTemplate( M ); }

typedef bub::matrix<double,bub::column_major> __SRS_matcolmaj ;
typedef bub::matrix_range<__SRS_matcolmaj> __SRS_rang_matcolmaj;
typedef bub::triangular_adaptor<bub::matrix_range<bub::matrix<double,bub::column_major> >,bub::upper > __SRS_triadup_rang_matcolmaj;
typedef bub::triangular_adaptor<bub::matrix_range< bubMatrix >,bub::upper> __SRS_triadup_rang_mat;
typedef bub::triangular_adaptor<bub::matrix_range<bub::triangular_matrix<double,bub::upper> >,bub::upper > __SRS_triadup_rang_triup ;
typedef bub::triangular_matrix<double,bub::upper> __SRS_triup ;
typedef bub::matrix_column<__SRS_matcolmaj> __SRS_col_matcolmaj;

#define SOT_ROTATION_SIMPLE_TEMPLATE_VECTOR_LIST(A) \
  A(bubVector); A(bub::vector_range<bubVector>) ; A(bub::matrix_column<bubMatrix>); A(bub::vector_range<bub::matrix_column<bubMatrix> >);  A(__SRS_col_matcolmaj);
#define SOT_ROTATION_SIMPLE_TEMPLATE_MATRIX_LIST(A) \
  A(bubMatrix); A(bub::matrix_range<bubMatrix>);A(__SRS_matcolmaj);A( __SRS_rang_matcolmaj);A( __SRS_triadup_rang_matcolmaj);A( __SRS_triadup_rang_mat); A(__SRS_triadup_rang_triup ); A(__SRS_triup);


  /* Virtual Pure. */
class SOT_CORE_EXPORT RotationSimple
{
 public:
  virtual ~RotationSimple( void ) {}

 public:
  /* --- STANDARD (FULL-RANGE) MULTIPLICATIONS. */
  /* Vector multiplications */
  SOT_ROTATION_SIMPLE_TEMPLATE_VECTOR_LIST(SOT_ROTATION_SIMPLE_MULTIPLY);
  /* Matrix multiplications */
  SOT_ROTATION_SIMPLE_TEMPLATE_MATRIX_LIST(SOT_ROTATION_SIMPLE_MULTIPLY);
  SOT_ROTATION_SIMPLE_TEMPLATE_MATRIX_LIST(SOT_ROTATION_SIMPLE_MULTIPLY_TRANSPOSE);

  /* --- (LIMITED-)RANGE MULTIPLICATIONS. */
  /* Vector multiplications */
  /* Multiply M := Q*[0;I;0]*M. */
  template<typename bubTemplateVectorIN,typename bubTemplateVectorOUT >
    void multiplyRangeRight( const bubTemplateVectorIN& M,
                             bubTemplateVectorOUT& res,
                             const unsigned int zeroBefore,
                             const unsigned int zeroAfter  ) const
    {
      const unsigned int m=M.size();
      const unsigned int mres=m+zeroBefore+zeroAfter;
      res.resize( mres ); bubClear(res);
      bub::vector_range< bubTemplateVectorOUT > Mrange
        (res,bub::range(zeroBefore,zeroBefore+m));
      Mrange.assign(M);
      multiplyRight(Mrange);
    }
  /* Multiply M := Q*[0;I;0]*M. */
  template<typename bubTemplateVector>
    void multiplyRangeRight( bubTemplateVector& M,
                             const unsigned int zeroBefore,
                             const unsigned int zeroAfter ) const
    {
      bub::project(M,bub::range(0,zeroBefore))
        .assign( bub::zero_vector<double>(zeroBefore) );
      bub::project(M,bub::range(M.size()-zeroAfter,M.size()))
        .assign( bub::zero_vector<double>(zeroAfter) );
      multiplyRight(M);
    }
  /* Multiply M := M*Q*[0 I 0]. */
  template<typename bubTemplateVectorIN,typename bubTemplateVectorOUT>
    void multiplyRangeLeft( const bubTemplateVectorIN& M,
                            bubTemplateVectorOUT& res,
                            const unsigned int zeroBefore,
                            const unsigned int zeroAfter ) const
    {
      bubVector acopy = M; multiplyLeft(acopy);
      const unsigned int mres = M.size() - zeroBefore - zeroAfter;
      res.resize(mres); res.assign( bub::project( acopy,bub::range(zeroBefore,mres)) );
    }
  /* Multiply M := M*Q*[0 I 0]. */
/*   template<typename bubTemplateVector> */
/*     void multiplyRangeLeft( bubTemplateVector& m, */
/*                             const unsigned int zeroBefore, */
/*                             const unsigned int zeroAfter ) const */
/*     { */
/*       multiplyLeft(m); */
/*       if(zeroBefore>0) */
/*         bub::project( m,bub::range(0,zeroBefore)) *= 0; */
/*       if(zeroAfter>0) */
/*         bub::project( m,bub::range(m.size()-zeroAfter,m.size())) *= 0; */
/*     } */
  //   virtual void multiplyRangeLeft( const bubVector& M,
  //                                    bubVector& res,
  //                                    const unsigned int zeroBefore,
  //                                    const unsigned int zeroAfter  ) const = 0;
  //   virtual void multiplyRangeLeft( bubVector& M,
  //                                    const unsigned int zeroBefore,
  //                                    const unsigned int zeroAfter ) const = 0;

  /* Multiply M := [0;I;0]*Q'*M = M*Q*[0 I 0]. */
  template<typename bubTemplateVector>
    bub::vector_range<bubTemplateVector>
    multiplyRangeLeft(  bubTemplateVector& m,
                        const unsigned int zeroBefore,
                        const unsigned int zeroAfter ) const
    {
      //sotDEBUG(1)<<"Q = " << *this << std::endl;
      multiplyLeft(m);
      if(zeroBefore>0)
        bub::project( m,bub::range(0,zeroBefore))
          .assign(bub::zero_vector<double>(zeroBefore));
      if(zeroAfter>0)
        bub::project( m,bub::range(m.size()-zeroAfter,m.size()))
          .assign(bub::zero_vector<double>(zeroAfter));
      return bub::vector_range<bubTemplateVector>(m,bub::range(zeroBefore,m.size()-zeroAfter));
    }


  /* Matrix multiplications */
  /* Multiply M := Q*[0;I;0]*M. */
  virtual void multiplyRangeRight( const bubMatrix& M,
                                   bubMatrix& res,
                                   const unsigned int zeroBefore,
                                   const unsigned int zeroAfter  ) const
  {
    const unsigned int m=M.size1(); const unsigned int nJ = M.size2();
    const unsigned int mres=m+zeroBefore+zeroAfter;
    res.resize( mres,nJ ); res = bub::zero_matrix<double>(mres,nJ);
    bub::matrix_range< bubMatrix > Mrange(res,bub::range(zeroBefore,zeroBefore+m),
                                          bub::range(0,nJ));
    Mrange.assign(M);    multiplyRight(Mrange);
  }
  /* Multiply M := Q*[0;I;0]*M. */
  virtual void multiplyRangeRight( bubMatrix& M,
                                   const unsigned int zeroBefore,
                                   const unsigned int zeroAfter ) const
  {
    const unsigned int nJ = M.size2(); bub::range fullCol(0,nJ);
    bub::project(M,bub::range(0,zeroBefore),fullCol)
      = bub::zero_matrix<double>(zeroBefore,nJ);
    const unsigned int mp = M.size1()-zeroBefore-zeroAfter;
    bub::project(M,bub::range(zeroBefore+mp,zeroBefore+mp+zeroAfter),fullCol)
      = bub::zero_matrix<double>(zeroAfter,nJ);
    multiplyRight(M);
  }

public:
  virtual std::ostream & display( std::ostream & os ) const = 0;
  friend std::ostream& operator << ( std::ostream & os,const RotationSimple& Q ) { return Q.display(os); }
};



/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */

class SOT_CORE_EXPORT sotRotationSimpleHouseholder
  : public RotationSimple
{
public: // protected:
  bubVector v;
  double beta;

public:
  sotRotationSimpleHouseholder( void ) { v.resize(0);}
  sotRotationSimpleHouseholder( const unsigned int n )
  { v.resize(n); v = bub::zero_vector<double>(n); beta=0;}
  sotRotationSimpleHouseholder( const bubVector& _v, const double& _beta )
  { v= _v; beta=_beta; }
  sotRotationSimpleHouseholder( const bubMatrix& _v, const double& _beta )
  { v.resize(_v.size1()); v = bub::row(_v,0);    beta=_beta;   }
  virtual ~sotRotationSimpleHouseholder( void ) {}


public:

  SOT_ROTATION_SIMPLE_TEMPLATE_VECTOR_LIST(SOT_ROTATION_DERIVATED_MULTIPLY_VECTOR)
  SOT_ROTATION_SIMPLE_TEMPLATE_MATRIX_LIST(SOT_ROTATION_DERIVATED_MULTIPLY_MATRIX)

  /* --- VECTOR --- */
  template< typename bubTemplateVector>
  void multiplyRightVectorTemplate( bubTemplateVector& a ) const // P*a
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

  template< typename bubTemplateVector>
  void multiplyLeftVectorTemplate( bubTemplateVector& m ) const // a*P=P*a
  { multiplyRightVectorTemplate(m); }

  /* --- MATRIX --- */
  template< typename bubTemplateMatrix >
  void multiplyRightMatrixTemplate( bubTemplateMatrix& A ) const // P*A
    // P*A = (I-b.v.v') A = A-b.v.(v'.A)
  {
    const unsigned int m=A.size1(), n=A.size2();
    const unsigned int kv = v.size();
    const unsigned int m_kv = m-kv;

    bubVector w(n); w.clear();
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

  template< typename bubTemplateMatrix >
  void multiplyLeftMatrixTemplate( bubTemplateMatrix& A ) const // A*P
    // A*P = A.(I-b.v.v') = A-b.(A.v).v'

  {
    const unsigned int m=A.size1(), n=A.size2();
    const unsigned int kv = v.size();
    const unsigned int n_kv = n-kv;

    bubVector w(m); w.clear();
    // W <- b A v = b A [ 0; ...; 0; 1; v ]
    for( unsigned int i=0;i<m;++i )
      {
	for( unsigned int j=0;j<kv;++j )
	  { w(i) += A(i,n_kv+j)*v(j);  }
	w(i)*=beta;
      }
    //sotDEBUG(1)<<"whh = " << (MATLAB)w << std::endl;
    // A <- A - w v' = w [ 0 ...  0  1 v' ]
    for( unsigned int j=0;j<kv;++j )
      for( unsigned int i=0;i<m;++i )
	{ A(i,j+n_kv)-=w(i)*v(j); }
  }
  template< typename bubTemplateMatrix >
  void multiplyRightTransposeMatrixTemplate( bubTemplateMatrix& M ) const // P'*A=P*A
  { multiplyRightMatrixTemplate(M); }

  template< typename bubTemplateMatrix>
  void multiplyLeftTransposeMatrixTemplate( bubTemplateMatrix& M ) const // A*P'=A*P
  { multiplyLeftMatrixTemplate(M); }

  /* --- DIPLAY ----------------------------------------------- */
  virtual std::ostream& display( std::ostream & os ) const
  { return os << "[b=" << beta << "] " << (MATLAB)v;  }

  /* --- HOUSEHOLDER EXTRACTION ------------------------------- */
  /* After the execution of the function, x is the corresponding
   * househould vector h so that x-b.h'.h.x = [ || x ||; 0 ].
   * Additionally, return the norm of original x.  */
  template<typename bubTemplateVector>
    static double householderExtraction( bubTemplateVector& x,double& beta,
                                         const double THRESHOLD_ZERO=1e-15 )
  {
    const unsigned int n=x.size();

    double sigma=0;
    for( unsigned int i=1;i<n;++i ) // sigma = x(2:n)^T x(2:n)
      {const double& xi=x(i); sigma+=xi*xi;}

    if( fabs(sigma)<THRESHOLD_ZERO ) // Vector already nullified x=[||x||; 0].
      {
        const double norm = x(0); // return a norm ~homogeneous to x(0)
        beta = 0;   x(0)=1;
        return norm;
      }
    else
      {
        double& v1=x(0);
        const double x1sq=v1*v1;
        const double mu= sqrt(x1sq+sigma); // mu = ||x||
        int signRes;
        if(v1<=0) { v1 -= mu;  signRes = 1; }
        else { v1 = -sigma/(v1+mu); signRes = -1; }

        const double v1sq=v1*v1;
        const double v1inv=1/v1;
        beta=2*v1sq/(sigma+v1sq);
        for( unsigned int i=0;i<n;++i )
          { x(i) *= v1inv; }
        return mu;
      }
  }
};


/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */

class SOT_CORE_EXPORT sotRotationSimpleGiven
  : public RotationSimple
{
public: // protected:
  double cosF,sinF;
  unsigned int idx1,idx2; // The matrix is I but on R(idxN,idxN), N={1,2}.

public: /* --- CONSTRUCTORS ------------------------------------------------- */
  sotRotationSimpleGiven( void )
    : cosF(1),sinF(0),idx1(0),idx2(0) {}

  sotRotationSimpleGiven( const double _cosF,const double _sinF,
                          const unsigned int _idx1,const unsigned int _idx2 )
    : cosF(_cosF),sinF(_sinF),idx1(_idx1),idx2(_idx2) {}
  /* Intialize for nullification of Rx.U: nullify R(r,c2) using R(r,c1). */
  // For nullification from the right.
  sotRotationSimpleGiven( unsigned int row, unsigned int col1,
                          unsigned int col2, const bubMatrix& Rx )
  {    nullifyRow(Rx,row,col1,col2);  }
  // For nullification from the right.
  sotRotationSimpleGiven( const bubMatrix& Rx, unsigned int row1,
                          unsigned int row2, unsigned int col )
  {    nullifyColumn(Rx,row1,row2,col);  }
  virtual ~sotRotationSimpleGiven( void ) {}

public: /* --- MULTIPLIERS -------------------------------------------------- */

  SOT_ROTATION_SIMPLE_TEMPLATE_VECTOR_LIST(SOT_ROTATION_DERIVATED_MULTIPLY_VECTOR)
  SOT_ROTATION_SIMPLE_TEMPLATE_MATRIX_LIST(SOT_ROTATION_DERIVATED_MULTIPLY_MATRIX)

  /* --- VECTORS --- */
  template< typename bubTemplateVector >
  void multiplyLeftVectorTemplate( bubTemplateVector & v ) const // v <- v.U=U'.v
  {
    double & R1 =  v(idx1);  double & R2 =  v(idx2);
    const double r1 = R1;         const double r2 = R2;
    R1 = r1*cosF-r2*sinF;         R2 = r1*sinF+r2*cosF;
  } /// A VERIFIER DANS GOLUB!
  template< typename bubTemplateVector >
  void multiplyRightVectorTemplate( bubTemplateVector & v ) const // v <- U.v
  {
    double & R1 =  v(idx1);  double & R2 =  v(idx2);
    const double r1 = R1;         const double r2 = R2;
    R1 = r1*cosF+r2*sinF;         R2 = -r1*sinF+r2*cosF;
  }

  /* --- LEFT MATRIX --- */
  template< typename bubTemplateMatrix >
  void multiplyLeftMatrixTemplate( bubTemplateMatrix & M ) const // M <- M.U
  {
    const unsigned int n = M.size1();
    for( unsigned int row=0;row<n;++row )
      {
        double & R1 =  M(row,idx1);  double & R2 =  M(row,idx2);
        const double r1 = R1;         const double r2 = R2;
        R1 = r1*cosF-r2*sinF;         R2 = r1*sinF+r2*cosF;
      }
  }
  void multiplyLeftMatrixTemplate( bub::triangular_matrix<double,bub::upper> & M ) const
  {
    const unsigned int n = std::min( std::min(idx1,idx2),M.size1());
    for( unsigned int row=0;row<n;++row )
      {
        double & R1 =  M(row,idx1);  double & R2 =  M(row,idx2);
        const double r1 = R1;         const double r2 = R2;
        R1 = r1*cosF-r2*sinF;         R2 = r1*sinF+r2*cosF;
      }
  }
  template< typename bubTemplateMatrix > // M <- M.U
  void multiplyLeftMatrixTemplate( bub::triangular_adaptor<bubTemplateMatrix,bub::upper> & M ) const
  {
    const unsigned int n = std::min( std::min(idx1,idx2)+1,M.size1());
    for( unsigned int row=0;row<n;++row )
      {
        double & R1 =  M(row,idx1);  double & R2 =  M(row,idx2);
        const double r1 = R1;         const double r2 = R2;
        R1 = r1*cosF-r2*sinF;         R2 = r1*sinF+r2*cosF;
      }
  }

  /* --- RIGHT MATRIX --- */
  template< typename bubTemplateMatrix >
  void multiplyRightMatrixTemplate( bubTemplateMatrix & M ) const // M <- U.M
  {
    const unsigned int m = M.size2();
    for( unsigned int col=0;col<m;++col )
      {
        double & R1 =  M(idx1,col);  double & R2 =  M(idx2,col);
        const double r1 = R1;         const double r2 = R2;
        R1 = r1*cosF+r2*sinF;         R2 = -r1*sinF+r2*cosF;
      }
  }
  void multiplyRightMatrixTemplate( bub::triangular_matrix<double,bub::upper> & M ) const
  {
    const unsigned int m = std::min( std::min(idx1,idx2),M.size2());
    for( unsigned int col=0;col<m;++col )
      {
        double & R1 =  M(idx1,col);  double & R2 =  M(idx2,col);
        const double r1 = R1;         const double r2 = R2;
        R1 = r1*cosF+r2*sinF;         R2 = -r1*sinF+r2*cosF;
      }
  }
  template< typename bubTemplateMatrix >  // M <- M.U'
  void multiplyRightMatrixTemplate( bub::triangular_adaptor<bubTemplateMatrix,bub::upper> & M ) const
  {
    const unsigned int m = std::min( std::min(idx1,idx2)+1,M.size2());
    for( unsigned int col=0;col<m;++col )
      {
        double & R1 =  M(idx1,col);  double & R2 =  M(idx2,col);
        const double r1 = R1;         const double r2 = R2;
        R1 = r1*cosF+r2*sinF;         R2 = -r1*sinF+r2*cosF;
      }
  }

  /* --- LEFT TRANSPOSE MATRIX --- */
  template< typename bubTemplateMatrix >
  void multiplyLeftTransposeMatrixTemplate( bubTemplateMatrix & M ) const // M <- M.U
  {
    const unsigned int n = M.size1();
    for( unsigned int row=0;row<n;++row )
      {
        double & R1 =  M(row,idx1);  double & R2 =  M(row,idx2);
        const double r1 = R1;         const double r2 = R2;
        R1 = r1*cosF+r2*sinF;         R2 = -r1*sinF+r2*cosF;
      }
  }
  void multiplyLeftTransposeMatrixTemplate( bub::triangular_matrix<double,bub::upper> & M ) const
  {
    const unsigned int n = std::min( std::min(idx1,idx2),M.size1());
    for( unsigned int row=0;row<n;++row )
      {
        double & R1 =  M(row,idx1);  double & R2 =  M(row,idx2);
        const double r1 = R1;         const double r2 = R2;
        R1 = r1*cosF+r2*sinF;         R2 = -r1*sinF+r2*cosF;
      }
  }
  template< typename bubTemplateMatrix > // M <- M.U
  void multiplyLeftTransposeMatrixTemplate( bub::triangular_adaptor<bubTemplateMatrix,bub::upper> & M ) const
  {
    const unsigned int n = std::min( std::min(idx1,idx2)+1,M.size1());
    for( unsigned int row=0;row<n;++row )
      {
        double & R1 =  M(row,idx1);  double & R2 =  M(row,idx2);
        const double r1 = R1;         const double r2 = R2;
        R1 = r1*cosF+r2*sinF;         R2 = -r1*sinF+r2*cosF;
      }
  }

  /* --- RIGHT TRANSPOSE MATRIX --- */
  template< typename bubTemplateMatrix >
  void multiplyRightTransposeMatrixTemplate( bubTemplateMatrix & M ) const // M <- U.M
  {
    const unsigned int m = M.size2();
    for( unsigned int col=0;col<m;++col )
      {
        double & R1 =  M(idx1,col);  double & R2 =  M(idx2,col);
        const double r1 = R1;         const double r2 = R2;
        R1 = r1*cosF-r2*sinF;         R2 = r1*sinF+r2*cosF;
      }
  }
  void multiplyRightTransposeMatrixTemplate( bub::triangular_matrix<double,bub::upper> & M ) const
  {
/*     const unsigned int m = std::min( std::min(idx1,idx2),M.size2()); */
/*     for( unsigned int col=0;col<m;++col ) */
/*       { */
/*         double & R1 =  M(idx1,col);  double & R2 =  M(idx2,col); */
/*         const double r1 = R1;         const double r2 = R2; */
/*         R1 = r1*cosF-r2*sinF;         R2 = r1*sinF+r2*cosF; */
/*       } */
    std::cerr << "Not implemented yet (sotRotationSimple l" << __LINE__
             << ")." << std::endl;
    throw  "Not implemented yet.";
  }
  template< typename bubTemplateMatrix >  // M <- M.U'
  void multiplyRightTransposeMatrixTemplate( bub::triangular_adaptor<bubTemplateMatrix,bub::upper> & M ) const
  {
/*     const unsigned int m1 = std::min(idx1,idx2); */
/*     for( unsigned int col=m1;col<M.size2();++col ) */
/*       { */
/*         double & R1 = M(idx1,col);    double & R2 =  M(idx2,col); */
/*         const double r1 = R1;         const double r2 = R2; */
/*         R1 = r1*cosF-r2*sinF;         R2 = r1*sinF+r2*cosF; */
/*       } */
    std::cerr << "Not implemented yet (sotRotationSimple l" << __LINE__
             << ")." << std::endl;
    throw  "Not implemented yet.";

  }

  /* --- DISPLAY --- */
  virtual std::ostream& display(  std::ostream& os ) const
  { return os << "GR["<<idx1<<","<<idx2<<"]("<<cosF<<","<<sinF<<")"; }


  /* --- GIVEN ROTATION EXTRACTION --- */
  /* On line <v>, find the coeff a&b so that (v.R)([col1 col2]) = [ x 0 ]. */
  template< typename bubTemplateVector >
  void nullifyFromRight( const bubTemplateVector & v,
                        unsigned int col1, unsigned int col2 )
  {
    const double &a=v(col1);  const double &b=v(col2); double tau;
    if( fabs(b)>fabs(a) )
      { tau=-a/b; sinF=1/sqrt(1+tau*tau); cosF=sinF*tau; }
    else
      { tau=-b/a; cosF=1/sqrt(1+tau*tau); sinF=cosF*tau; }
    idx1=col1; idx2=col2;
  }
  void inverse( void ) { sinF *= -1; }
  template< typename bubTemplateVector >
  void nullifyFromLeft( const bubTemplateVector & v,
                         unsigned int col1, unsigned int col2 )
  {    nullifyFromRight(v,col1,col2); inverse();  }

  template< typename bubTemplateMatrix >
  void nullifyRow( const bubTemplateMatrix& M, unsigned int row,
                   unsigned int col1, unsigned int col2 )
  {    nullifyFromRight(bub::row(M,row),col1,col2);  }
  template< typename bubTemplateMatrix >
  void nullifyColumn( const bubTemplateMatrix& M, unsigned int row1,
                   unsigned int row2, unsigned int col )
  {    nullifyFromLeft(bub::column(M,col),row1,row2);  }
};

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
class SOT_CORE_EXPORT sotRotationComposed
  :public RotationSimple
{
public: // protected
  std::list< RotationSimple* > listRotationSimple;
  std::list< sotRotationSimpleHouseholder > listHouseholder;
  std::list< sotRotationSimpleGiven > listGivenRotation;

public:
  sotRotationComposed( void )
    :listRotationSimple(0),listHouseholder(0),listGivenRotation(0) {}

  sotRotationComposed( const sotRotationComposed& clone )
    :listRotationSimple(0),listHouseholder(0),listGivenRotation(0)
    {
      pushBack(clone);
    }
  virtual ~sotRotationComposed( void )
    { listRotationSimple.clear(); listGivenRotation.clear(); listHouseholder.clear();}

public:
  template< typename TemplateRotation>
  void pushBack( const TemplateRotation& R )
  { std::cerr << "Error: simple rotation not defined yet. " << std::endl;
    throw  "Error: simple rotation not defined yet. "; }

  void pushBack( const sotRotationSimpleGiven & R )
  {
    listGivenRotation.push_back(R);
    listRotationSimple.push_back(&listGivenRotation.back());
  }
  void pushBack( const sotRotationSimpleHouseholder & R )
  {
    listHouseholder.push_back(R);
    listRotationSimple.push_back(&listHouseholder.back());
  }
  void pushBack( const RotationSimple * R )
  {
    const sotRotationSimpleGiven * GR = dynamic_cast<const sotRotationSimpleGiven *>(R);
    if( GR ) pushBack(*GR);
    else
      {
        const sotRotationSimpleHouseholder* HH = dynamic_cast<const sotRotationSimpleHouseholder *>(R);
        if( HH ) pushBack(*HH);
        else
          {
            std::cerr << "Error: this rotation cast not defined yet. " << std::endl;
            throw  "Error: simple rotation not defined yet. ";
          }
      }
  }
  void pushBack( const sotRotationComposed & R )
  {
    //sotDEBUG(15) << "PB size clone = " << R.listRotationSimple.size() << std::endl;
    for( std::list<RotationSimple*>::const_iterator Pi = R.listRotationSimple.begin();
Pi!=R.listRotationSimple.end(); ++Pi )
      {
        const RotationSimple* R = *Pi;
        const sotRotationSimpleHouseholder * Rh = dynamic_cast<const sotRotationSimpleHouseholder*>(R);
        const sotRotationSimpleGiven * Rg = dynamic_cast<const sotRotationSimpleGiven*>(R);
        if(NULL!=Rh) pushBack(*Rh);
        else if(NULL!=Rg)
          {
            pushBack(*Rg);
            //sotDEBUG(55) << "PB GR " << *Rg << std::endl;
          }
        else
          {
            std::cerr << "Error: this rotation PB not defined yet. " << std::endl;
            throw  "Error: simple rotation not defined yet. ";
          }
      }
  }

  void popBack( void )
  {
    RotationSimple * R = listRotationSimple.back();
    listRotationSimple.pop_back();
    if( dynamic_cast<sotRotationSimpleHouseholder*>(R) ) { listHouseholder.pop_back(); }
    if( dynamic_cast<sotRotationSimpleGiven*>(R) ) { listGivenRotation.pop_back(); }
  }
  void clear( void )
  {
    listRotationSimple.clear();
    listHouseholder.clear();
    listGivenRotation.clear();
  }

  template< typename bubTemplateMatrix >
  void householderQRinit( const bubTemplateMatrix & RQ,
                          const bubVector & betas,
                          const int nbVector=-1 )
  {
    const unsigned int nToProceed
      =((nbVector<0)
	?std::min(RQ.size1(),RQ.size2())
	:(unsigned int)nbVector);
    const unsigned int m=RQ.size1();

    for( unsigned int i=0;i<nToProceed;++i )

      {
	bubVector v(m-i);  v(0)=1; // TODO: optimize to save the nTP allocations of v.
	for( unsigned int j=1;j<m-i;++j ) v(j)=RQ(j+i,i);
	this->pushBack( sotRotationSimpleHouseholder(v,betas(i)) );
      }
  }

  enum GivenRotationModifior { GR_FIRST,GR_SECOND,GR_BOTH };
  void resetGRindex( const GivenRotationModifior modifior,
                     const unsigned int k1,const unsigned int k2=1 )
  {
    for( std::list<sotRotationSimpleGiven>::iterator Gi
           = listGivenRotation.begin();
	 Gi!=listGivenRotation.end(); ++Gi )
      { switch(modifior)
          { case GR_FIRST: Gi->idx1=k1; break;
          case GR_SECOND: Gi->idx2=k1; break;
          case GR_BOTH: Gi->idx1=k1; Gi->idx2=k2;break;
          }
      }
  }
  void increaseGRindex( const GivenRotationModifior modifior,
                        const unsigned int k1,const unsigned int k2=1 )
  {
    for( std::list<sotRotationSimpleGiven>::iterator Gi
           = listGivenRotation.begin();
	 Gi!=listGivenRotation.end(); ++Gi )
      { switch(modifior)
          { case GR_FIRST: Gi->idx1+=k1; break;
          case GR_SECOND: Gi->idx2+=k1; break;
          case GR_BOTH: Gi->idx1+=k1; Gi->idx2+=k2;break;
          }
      }
  }

  /* --- MULTIPLIERS --- */
  SOT_ROTATION_SIMPLE_TEMPLATE_VECTOR_LIST(SOT_ROTATION_DERIVATED_MULTIPLY_VECTOR)
  SOT_ROTATION_SIMPLE_TEMPLATE_MATRIX_LIST(SOT_ROTATION_DERIVATED_MULTIPLY_MATRIX)

  template< typename bubTemplateVector >
  void multiplyRightVectorTemplate( bubTemplateVector & v ) const // v <- U.v
  { multiplyRightTemplate(v);  }
  template< typename bubTemplateVector >
  void multiplyLeftVectorTemplate( bubTemplateVector & v ) const // v <- U.v
  { multiplyLeftTemplate(v);  }
  template< typename bubTemplateMatrix >
  void multiplyRightMatrixTemplate( bubTemplateMatrix & M ) const // M <- U.M
  { multiplyRightTemplate(M);  }
  template< typename bubTemplateMatrix >
  void multiplyLeftMatrixTemplate( bubTemplateMatrix & M ) const // M <- U.M
  { multiplyLeftTemplate(M);  }
  template< typename bubTemplateMatrix >
  void multiplyRightTransposeMatrixTemplate( bubTemplateMatrix & M ) const // M <- U.M
  { multiplyRightTransposeTemplate(M);  }
  template< typename bubTemplateMatrix >
  void multiplyLeftTransposeMatrixTemplate( bubTemplateMatrix & M ) const // M <- U.M
  { multiplyLeftTransposeTemplate(M);  }

  template< typename bubTemplate >
  void multiplyLeftTemplate( bubTemplate & Rx ) const // Rx <- Rx*U1*...*Un
  {
    for( std::list<RotationSimple*>::const_iterator Pi
           = listRotationSimple.begin();
	 Pi!=listRotationSimple.end(); ++Pi )
      {	(*Pi)->multiplyLeft(Rx);      }
  }
  template< typename bubTemplate >
  void multiplyRightTemplate( bubTemplate & Rx ) const // Rx <- U1*...*Un*Rx
  {
    for( std::list<RotationSimple*>::const_reverse_iterator Pi
           = listRotationSimple.rbegin();
	 Pi!=listRotationSimple.rend(); ++Pi )
      {      (*Pi)->multiplyRight(Rx);      }
  }
  template< typename bubTemplate >
  void multiplyLeftTransposeTemplate( bubTemplate & Rx ) const // Rx <- Rx*Un*...*U1
  {
    for( std::list<RotationSimple*>::const_reverse_iterator Pi
           = listRotationSimple.rbegin();
	 Pi!=listRotationSimple.rend(); ++Pi )
      {	(*Pi)->multiplyLeftTranspose(Rx);      }
  }
  template< typename bubTemplate >
  void multiplyRightTransposeTemplate( bubTemplate & Rx ) const // Rx <- Un*...*U1*Rx
  {
    for( std::list<RotationSimple*>::const_iterator Pi
           = listRotationSimple.begin();
	 Pi!=listRotationSimple.end(); ++Pi )
      {        (*Pi)->multiplyRightTranspose(Rx);      }
  }

  /* --- DISPLAY --- */
  virtual std::ostream& display(  std::ostream& os ) const
  {
    for( std::list<RotationSimple*>::const_iterator Pi = listRotationSimple.begin();
	 Pi!=listRotationSimple.end(); ++Pi )
      {
	os << (**Pi) << " ";
      }
    if( listRotationSimple.empty() ) os << "Identity" << std::endl;
    return os;
  }

  /* --- EXTRACTION --- */
  template< typename bubTemplateMatrix >
  void householderTrigonalisation( bubTemplateMatrix &R )
  {
    const unsigned int m=R.size1();
    const unsigned int n=R.size2();
    for( unsigned int i=0;i<std::min(m,n);++i )
      {
        bub::matrix_column<bubMatrix> m(R,i);
        this->multiplyLeft(m);
        bub::vector_range<bub::matrix_column<bubMatrix> > mdown(m,bub::range(i,5));
        double beta;
        R(i,i) = sotRotationSimpleHouseholder::householderExtraction(mdown,beta);
        this->pushBack(sotRotationSimpleHouseholder(mdown,beta));
      }
  }


  /* Rx has a shape like [Rr Rh], with Rr (full-rank)-triangular up, and
   * Rh (generic-shape) rectangular.
   * Regularize to [R0 Rh0], where column <xn> of Rh0 is null.
   */
  template< typename bubTemplateMatrix >
  void regularizeRankDeficientTriangle( bubTemplateMatrix & Rx, const unsigned int xn=0  )
  {
    const unsigned int n = Rx.size1();
    for( unsigned int iter =1; iter<=n;iter++ )
      {
        //sotDEBUG(45) << "Rx"<<iter << " = " << (MATLAB)Rx << std::endl;
        sotRotationSimpleGiven Un;
        Un.nullifyFromRight(bub::row(Rx,n-iter),n-iter,n+xn);
        Un.multiplyLeft(Rx);
        //sotDEBUG(45) << "U"<<iter << " = " << Un << std::endl;
        if(fabsf(Un.sinF)>1e-9)pushBack(Un);
      }
  }

  template< typename bubTemplateMatrix >
    void regularizeRankDeficientTriangle( bubTemplateMatrix & Rx,
                                          bub::indirect_array<> orderCol,
                                          const unsigned int xn )
    {
      sotDEBUG(1)<<"order="<<(MATLAB)orderCol<<std::endl;
      sotDEBUG(45) << "Rx = " << (MATLAB)Rx << std::endl;
      const unsigned int n = Rx.size1();
sotDEBUG(1)<<std::endl;
      for( unsigned int iter =1; iter<=n;iter++ )
        {
          sotDEBUG(45) << "Rx"<<iter << " = " << (MATLAB)Rx << std::endl;
          sotRotationSimpleGiven Un;
          sotDEBUG(1)<<n<<std::endl;
          sotDEBUG(1)<<iter<<std::endl;
          sotDEBUG(1)<<n-iter<<std::endl;
          sotDEBUG(1)<<(MATLAB)bub::row(Rx,n-iter)<<std::endl;
          sotDEBUG(1)<<(MATLAB)bub::row(Rx,n-iter)
                     <<" ["<<orderCol(n-iter)<<","<<xn<<"]"<<std::endl;
          /* Nullify line <n-iter> on column <xn> using the corresponding <n-iter>
           * column in the ordered matrix (ie orderCol(n-iter) in the non-ordered
           * matrix. */
          Un.nullifyFromRight(bub::row(Rx,n-iter),orderCol(n-iter),xn);
sotDEBUG(1)<<std::endl;
          if(fabsf(Un.sinF)>1e-9)
            {
sotDEBUG(1)<<std::endl;
              Un.multiplyLeft(Rx);
sotDEBUG(1)<<std::endl;
              pushBack(Un);
sotDEBUG(1)<<std::endl;
            }
          sotDEBUG(45) << "U"<<iter << " = " << Un << std::endl;
        }
    }


};


/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
class SOT_CORE_EXPORT sotRotationComposedInExtenso
  :public RotationSimple
{
public: // protected
  bubMatrix Q;

public:
 sotRotationComposedInExtenso( const unsigned int nJ )
    :Q(bub::identity_matrix<double>(nJ,nJ)) {}

  sotRotationComposedInExtenso( const sotRotationComposedInExtenso& clone )
    :Q(clone.Q)
    {
    }
  virtual ~sotRotationComposedInExtenso( void )
    { }

public:
  template< typename TemplateRotation>
  void pushBack( const TemplateRotation& R )
  { // Q=U1*...*Un <- Q*Un+1
    R.multiplyLeft(Q);
  }
  void pushBack( const RotationSimple * R )
  {
    R->multiplyLeft(Q);
  }
  void clear( const int newSize = -1 )
  {
    unsigned int nJ = Q.size1();
    if( newSize>0 ) { nJ=(unsigned int)newSize;    Q.resize(nJ,nJ); }
    Q.assign( bub::identity_matrix<double>(nJ,nJ) );
  }
  unsigned int getSize( void ) { return Q.size1(); }
  void resize( unsigned int nJ )
  {
    Q.resize(nJ,nJ);
    Q.assign(bub::identity_matrix<double>(nJ,nJ));
  }
  template< typename bubTemplateMatrix >
  void householderQRinit( const bubTemplateMatrix & RQ,
                          const bubVector & betas,
                          const int nbVector=-1 )
  {
    sotRotationComposed Qlist; Qlist.householderQRinit(RQ,betas,nbVector);
    pushBack(Qlist);
  }

  /* --- MULTIPLIERS --- */
  SOT_ROTATION_SIMPLE_TEMPLATE_VECTOR_LIST(SOT_ROTATION_DERIVATED_MULTIPLY_VECTOR)
  SOT_ROTATION_SIMPLE_TEMPLATE_MATRIX_LIST(SOT_ROTATION_DERIVATED_MULTIPLY_MATRIX)

  template< typename bubTemplateVector >
  void multiplyRightVectorTemplate( bubTemplateVector & v ) const // v <- U.v
  {
    bubVector copy(v);
    v.assign(bub::prod(Q,copy));
  }
  template< typename bubTemplateVector >
  void multiplyLeftVectorTemplate( bubTemplateVector & v ) const // v <- U.v
  {
    bubVector copy(v);
    //sotDEBUG(1)<<"Rxcopy = " << (MATLAB)copy<<std::endl;
    //sotDEBUG(1)<<"Q = " << (MATLAB)Q<<std::endl;
    v.assign(bub::prod(copy,Q));
    //sotDEBUG(1)<<"Q'J' = JQ = " << (MATLAB)v<<std::endl;
  }
  template< typename bubTemplateMatrix >
  void multiplyRightMatrixTemplate( bubTemplateMatrix & M ) const // M <- U.M
  {
    bubMatrix copy(M);
    M.assign(bub::prod(Q,copy));
  }
  template< typename bubTemplateMatrix >
  void multiplyLeftMatrixTemplate( bubTemplateMatrix & M ) const // M <- U.M
  {
    bubMatrix copy(M);
    M.assign(bub::prod(copy,Q));
  }
  template< typename bubTemplateMatrix >
  void multiplyRightTransposeMatrixTemplate( bubTemplateMatrix & M ) const // M <- U.M
  {
    bubMatrix copy(M);
    M.assign(bub::prod(bub::trans(Q),copy));
  }
  template< typename bubTemplateMatrix >
  void multiplyLeftTransposeMatrixTemplate( bubTemplateMatrix & M ) const // M <- U.M
  {
    bubMatrix copy(M);
    M.assign(bub::prod(copy,bub::trans(Q)));
  }


  /* --- (LIMITED-)RANGE MULTIPLICATIONS. */
  /* Vector multiplications */
  /* Multiply M := Q*[0;I;0]*M. */
  template<typename bubTemplateVectorIN,typename bubTemplateVectorOUT >
    void multiplyRangeRight( const bubTemplateVectorIN& M,
                             bubTemplateVectorOUT& res,
                             const unsigned int zeroBefore,
                             const unsigned int zeroAfter  ) const
    {
      const unsigned int nonZero = Q.size2()-zeroAfter-zeroBefore;
      bub::matrix_range<bubMatrix>
        Q0I0(Q,bub::range(0,Q.size1()),bub::range(zeroBefore,zeroBefore+nonZero));
      res.resize( Q.size1() );
      res.assign( bub::prod(Q0I0,M) );
    }
  /* Multiply M := Q*[0;I;0]*M. */
  template<typename bubTemplateVector>
    void multiplyRangeRight( bubTemplateVector& M,
                             const unsigned int zeroBefore,
                             const unsigned int zeroAfter ) const
    {
      const unsigned int nonZero = Q.size2()-zeroAfter-zeroBefore;
      bub::project(M,bub::range(0,zeroBefore))
        .assign(bub::zero_vector<double>(zeroBefore));
      bub::project(M,bub::range(M.size()-zeroAfter,M.size()))
        .assign(bub::zero_vector<double>(zeroAfter));
      bubVector Min( bub::vector_range<bubTemplateVector>
                     ( M,bub::range(zeroBefore,zeroBefore+nonZero)) );
      bub::matrix_range<const bubMatrix>
        Q0I0(Q,bub::range(0,Q.size1()),bub::range(zeroBefore,zeroBefore+nonZero));
      //sotDEBUG(1) << "Qoio = " << (MATLAB)Q0I0 << std::endl;
      //sotDEBUG(1) << "m = " << (MATLAB)Min << std::endl;
      M.assign( bub::prod(Q0I0,Min));
    }
  /* Multiply M := M'*Q*[0;I;0]. */
  template<typename bubTemplateVectorIN,typename bubTemplateVectorOUT>
    void multiplyRangeLeft( const bubTemplateVectorIN& M,
                            bubTemplateVectorOUT& res,
                            const unsigned int zeroBefore,
                            const unsigned int zeroAfter ) const
    {
      const unsigned int nonZero = Q.size2()-zeroAfter-zeroBefore;
      bub::matrix_range<bubMatrix>
        Q0I0(Q,bub::range(0,Q.size1()),bub::range(zeroBefore,zeroBefore+nonZero));
      res.assign( bub::prod(M,Q0I0));
    }

  /* Multiply M := [0;I;0]*Q'*M = M'*Q*[0 I 0]. */
  template<typename bubTemplateVector>
    bub::vector_range<bubTemplateVector>
    multiplyRangeLeft(  bubTemplateVector& m,
                        const unsigned int zeroBefore,
                        const unsigned int zeroAfter ) const
    {
      const unsigned int nonZero = Q.size2()-zeroAfter-zeroBefore;
      const bub::matrix_range<const bubMatrix>
        Q0I0(Q,bub::range(0,Q.size1()),
             bub::range(zeroBefore,zeroBefore+nonZero));
      bub::vector_range<bubTemplateVector> Mnonzero(m,bub::range(zeroBefore,
                                                                 zeroBefore+nonZero));
      bubVector Min(Mnonzero);
      //sotDEBUG(55) << "Qoio = " << (MATLAB)Q0I0 << std::endl;
      //sotDEBUG(55) << "m = " << (MATLAB)Mnonzero << std::endl;
      Min.assign(bub::prod(m,Q0I0));
      Mnonzero.assign(Min);

      if(zeroBefore>0)
        bub::project( m,bub::range(0,zeroBefore))
          .assign(bub::zero_vector<double>(zeroBefore));
      if(zeroAfter>0)
        bub::project( m,bub::range(m.size()-zeroAfter,m.size()))
          .assign(bub::zero_vector<double>(zeroAfter));

      return Mnonzero;
    }



  /* --- DISPLAY --- */
  virtual std::ostream& display(  std::ostream& os ) const
  {
    return os<<Q;
  }

};


#endif //ifndef __SOT_sotRotationSimple_HH__
