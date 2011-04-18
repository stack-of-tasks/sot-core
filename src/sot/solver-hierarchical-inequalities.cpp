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


//#define WITH_CHRONO


//#define VP_DEBUG
//#define VP_DEBUG_MODE 45
#include <sot/core/debug.hh>

#include <sot/core/solver-hierarchical-inequalities.hh>
using namespace dynamicgraph::sot;

#ifndef WIN32
#  include <sys/time.h>
#else
# include <sot-core/utils-windows.h>
#endif /*WIN32*/

#if defined (WIN32) or defined (__APPLE__)
# include <boost/math/special_functions/fpclassify.hpp>
# define isnan (boost::math::isnan)
#endif //define WIN32 or defined __APPLE__

#define FORTRAN_ID( id ) id##_



/* ---------------------------------------------------------- */
/* --- BINDING FORTRAN -------------------------------------- */
/* ---------------------------------------------------------- */
#define LAPACK_DGEQP3 FORTRAN_ID( dgeqp3 )
#define LAPACK_DGEQPF FORTRAN_ID( dgeqpf )
extern "C" {
  void LAPACK_DGEQP3( const int* m, const int* n, double* a, const int* lda, int * jpvt,
                      double* tau, double* work, const int* lwork, int* info );
  void LAPACK_DGEQPF( const int* m, const int* n, double* a, const int* lda, int * jpvt,
                      double* tau, double* work, int* info );
}

namespace boost { namespace numeric { namespace bindings { namespace lapack
      {

        template<typename bubTemplateMatrix>
        inline int geqp (bubTemplateMatrix &A,
                         bub::vector< int >& jp, bubVector& tau)
        {
          int const mF= A.size1();
          int const nF= A.size2();
          if(( nF==0)||(mF==0)) return 0;
          ::boost::numeric::ublas::vector<double> work(std::max(1, nF*32));

          assert (nF <= (int)tau.size());
          assert (nF <= (int)work.size());

	  ::boost::numeric::ublas::matrix<double> tmpA;
	  tmpA = A;
	  double* aF =  MRAWDATA(tmpA);

          int const ldaF = traits::leading_dimension (A);
          int * jpvtF = VRAWDATA(jp);
          double * tauF = VRAWDATA(tau);
          double * workF = VRAWDATA(work);
          int const lworkF =  work.size();
          int infoF;

          LAPACK_DGEQP3(&mF, &nF, aF, &ldaF, jpvtF, tauF, workF, &lworkF, &infoF);
          //LAPACK_DGEQPF(&mF, &nF, aF, &ldaF, jpvtF, tauF, workF, &infoF);
          return infoF;
        }

      }}}} // End Namespaces



template< typename bubTemplateMatrix >
void bubRemoveColumn( bubTemplateMatrix& M, const unsigned int col )
{
  for( unsigned int j=col;j<M.size2()-1;++j )
    for( unsigned int i=0;i<M.size1();++i )
      M(i,j)=M(i,j+1);
  for( unsigned int i=0;i<M.size1();++i )
    M(i,M.size2()-1) = 0;
}
void bubRemoveColumn( bub::triangular_matrix<double,bub::upper>& M, const unsigned int col )
{
  for( unsigned int j=col;j<M.size2()-1;++j )
    for( unsigned int i=0;i<=j;++i )
      M(i,j)=M(i,j+1);
  for( unsigned int i=0;i<M.size1();++i )
    M(i,M.size2()-1) = 0;
}
template< typename bubTemplateMatrix >
void bubRemoveColumn( bub::triangular_adaptor<bubTemplateMatrix,bub::upper> M,
                      const unsigned int col )
{
  for( unsigned int j=col;j<M.size2()-1;++j )
    for( unsigned int i=0;i<=j;++i )
      M(i,j)=M(i,j+1);
  for( unsigned int i=0;i<std::min(M.size1(),M.size2());++i )
    M(i,M.size2()-1) = 0;
}


bub::indirect_array<>& operator+= ( bub::indirect_array<>& order,unsigned int _el )
{
  const unsigned int N = order.size();
  bub::indirect_array<> o2(N+1);
  //  std::copy(order.begin(),order.end(),o2.data().begin()); // Not working! Why?
  for( unsigned int i=0;i<N;++i ) o2[i]=order[i];
  o2[N]=_el;
  return order=o2;
}

bub::indirect_array<>& operator-= ( bub::indirect_array<>& order,unsigned int _el )
{
  const unsigned int N = order.size();
  bub::indirect_array<> o2(N-1); unsigned int newi=0;
  for( unsigned int i=0;i<N;++i )
    {
      if(newi==N)
        {
          std::cerr << "Error while removing one elmt of <order>." << std::endl;
          throw "Error while removing one elmt of <order>.";
        }
      if(order[i]!=_el) o2[newi++]=order[i];
    }
  return order=o2;
}

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */

ConstraintMem::
ConstraintMem( const ConstraintMem& clone )
  :active(clone.active),equality(clone.equality),notToBeConsidered(false)
  ,Ji(0),eiInf(clone.eiInf),eiSup(clone.eiSup)
  ,boundSide(clone.boundSide),activeSide(clone.activeSide)
  ,rankIncreaser(clone.rankIncreaser),constraintRow(clone.constraintRow)
  ,range(clone.range),lagrangian(clone.lagrangian)
  ,Ju(clone.Ju),Jdu(clone.Jdu)
{
  if( clone.Ji.size() ) { Ji.resize(clone.Ji.size(),false); Ji.assign(clone.Ji); }
  sotDEBUG(15) << "ConstraintMem cloning" << std::endl;
}

namespace dynamicgraph { namespace sot {
std::ostream& operator<<( std::ostream& os,const ConstraintMem::BoundSideType& bs )
{
  switch( bs )
    {
    case ConstraintMem::BOUND_VOID:
      os << "#";
      break;
    case ConstraintMem::BOUND_INF:
      os <<"-";
      break;
    case ConstraintMem::BOUND_SUP:
      os << "+";
      break;
    case ConstraintMem::BOUND_BOTH:
      os << "+/-";
      break;
    }
  return os;
}

std::ostream & operator<< (std::ostream& os,const ConstraintMem &c )
{
  os << "Cs[" << c.constraintRow << "] " << std::endl;
  if( c.Ji.size() ) os << "" << c.Ji;
  if( c.boundSide&ConstraintMem::BOUND_INF ) os << "/[-]" << c.eiInf;
  if( c.boundSide&ConstraintMem::BOUND_SUP ) os << "/[+]" << c.eiSup;
  if( c.active )
    { os << "\n\t-> active [" << c.activeSide
         << "] <rg=" << c.range << "> " << std::endl << "\t-> ";
      if( c.rankIncreaser ) os << "rank-inc " ;
      else { os << "non-rank-inc";}
      if(c.lagrangian!=0) os << std::endl << "\t-> l=" << c.lagrangian;
      if(c.equality )os << std::endl << "\t-> blocked(equality)";
      else os << std::endl << "\t-> floating(inequality)";
    }
  else
    {
      if(c.notToBeConsidered) os << "\n\t->not to be considered";
      else os << "\n\t-> inactive";
    }
  return os;
}

} /* namespace sot */} /* namespace dynamicgraph */

/* ---------------------------------------------------------- */
/* Specify the size of the constraint matrix, for pre-alocation. */
void SolverHierarchicalInequalities::
initConstraintSize( const unsigned int size )
{
  if(Rh.size1()!=nJ) {Rh.resize(nJ,nJ,false); Rh.clear();}
  rankh=0;
  u0.resize(nJ,false); u0.clear();
  //constraintH.reserve(size);
  constraintS.reserve(size+1);
}

void SolverHierarchicalInequalities::
setInitialCondition( const bubVector& _u0,
                     const unsigned int _rankh )
{
  u0.resize(nJ,false); u0.assign(_u0);
  rankh=_rankh; freeRank=nJ-rankh;
}

void SolverHierarchicalInequalities::
setInitialConditionVoid( void )
{
  u0.resize(nJ,false); u0.clear();
  rankh=0; freeRank=nJ;
}
void SolverHierarchicalInequalities::
setNbDof( const unsigned int _nJ )
{
  sotDEBUGIN(15);
  if( nJ==_nJ) return;
  nJ=_nJ;
  Qh.resize(nJ);
  Rh.resize(nJ,nJ);
  sotDEBUGOUT(15);
}

/* ---------------------------------------------------------- */

void SolverHierarchicalInequalities::
recordInitialConditions( void )
{
  initialActiveH.resize(constraintH.size()); initialSideH.resize(constraintH.size());
  std::vector<bool>::iterator iterBool = initialActiveH.begin();
  ConstraintMem::BoundSideVector::iterator iterSide = initialSideH.begin();
  ConstraintList::const_iterator iterCH = constraintH.begin();
  for( ;iterCH!=constraintH.end();++iterBool,++iterCH,++iterSide )
    {
      sotDEBUG(45) << "Initial " <<  iterCH->activeSide <<iterCH->active << std::endl;
      (*iterBool) = iterCH->active;
      (*iterSide) = iterCH->activeSide;
      sotDEBUG(45) << "Initial " << (*iterSide) << (*iterBool) << std::endl;
    }

  du0.resize(u0.size(),false); du0.assign(-u0);
}
void SolverHierarchicalInequalities::
computeDifferentialCondition( void )
{
  if( constraintH.size()>initialActiveH.size() )
    {
      std::vector<bool>::const_iterator iterBool = initialActiveH.begin();
      ConstraintList::iterator iterCH = constraintH.begin();
      ConstraintMem::BoundSideVector::const_iterator iterSide = initialSideH.begin();
      toActivate.clear(); toInactivate.clear();
      for( ;iterBool!=initialActiveH.end();++iterBool,++iterCH,++iterSide )
        {
          ConstraintMem & ch = *iterCH;
          sotDEBUG(45) << "Initial " << (*iterSide)<<(*iterBool)
                       << " - Final "<<ch.activeSide<< ch.active << std::endl;
          /* Constraint was active, and is not anymore or changed side. */
          if( (*iterBool)&&( (!ch.active)||(*iterSide!=ch.activeSide) ) )
            {
              toInactivate.push_back(ConstraintRef(ch.constraintRow));
            }
          /* Constraint is now active, but was inactive or changed side. */
          if( (ch.active)&&( (!(*iterBool))||(*iterSide!=ch.activeSide) ) )
            {
              toActivate.push_back(ConstraintRef(ch.constraintRow,ch.activeSide) );
            }
        }
    }
  du0+=u0;

  /* Compute the slack set to be initialize at first iteration. */
  unsigned int i=0; slackActiveSet.resize(constraintSactive.size());
  for( ConstraintRefList::const_iterator iter=constraintSactive.begin();
       iter!=constraintSactive.end();++iter,++i )
    {
      slackActiveSet[i].id = (*iter)->constraintRow;
      slackActiveSet[i].side = (*iter)->activeSide;
    }

  warmStartReady = true;
}

void SolverHierarchicalInequalities::
printDifferentialCondition( std::ostream & os ) const
{
  if(! warmStartReady ) return; 

  os << "To activate = { ";
  for( std::vector<ConstraintRef>::const_iterator iterCH = toActivate.begin();
       toActivate.end()!=iterCH;++iterCH )
    { os << *iterCH << ", "; }
  os << " }" << std::endl;

  os << "To inactivate = { ";
  for( std::vector<ConstraintRef>::const_iterator iterCH = toInactivate.begin();
       toInactivate.end()!=iterCH;++iterCH )
    { os << *iterCH << ", "; }
  os << " }" << std::endl;

  os <<"Active slack = { ";
  for( std::vector<ConstraintRef>::const_iterator iter=slackActiveSet.begin();
       iter!=slackActiveSet.end();++iter )
    { os << (*iter) << ", "; }
  os << " }" << std::endl;

  os << "du = " << (MATLAB)du0 << std::endl;
}

/* ---------------------------------------------------------- */


// SolverHierarchicalInequalities::bubMatrixQROrdered
// SolverHierarchicalInequalities::
// accessQRs( void )
// {
//   bubMatrixQR QRs( QhJsU,freerange(),bub::range::all() );
//   bubMatrixQROrdered QRord( QRs,bubOrder::all(),orderS );
//   return QRord;
// }
// SolverHierarchicalInequalities::bubMatrixQROrderedConst
// SolverHierarchicalInequalities::
// accessQRs( void ) const
// {
//   bubMatrixQRConst QRs( QhJsU,freerange(),bub::range::all() );
//   bubMatrixQROrderedConst QRord( QRs,bubOrder::all(),orderS );
//   return QRord;
// }
// SolverHierarchicalInequalities::bubMatrixQROrderedTri
// SolverHierarchicalInequalities::
// accessRs( void )
// {
//   bubMatrixQR QRs( QhJsU,freeranges(),bub::range::all() );
//    bubMatrixQROrdered QRord( QRs,bubOrder::all(),orderS );
//    return QRord;
//  }
SolverHierarchicalInequalities::bubMatrixQROrderedTriConst
SolverHierarchicalInequalities::
accessRsConst( void ) const
{
  bubMatrixQRConst QRs( QhJsU,freeranges(),bub::range(0,sizes) );
  bubOrder iall(ranks);
  for( unsigned int i=0;i<ranks;++i ) iall(i)=i;
  bubMatrixQROrderedConst QRord( QRs,iall,orderS );
  return QRord;
}

bub::triangular_adaptor<bub::matrix_range< const bubMatrix >,bub::upper>
SolverHierarchicalInequalities::
accessRhConst( void ) const
{
  bub::matrix_range< const bubMatrix >
    Rhup( Rh,rangeh(),rangeh() );
  return Rhup;
}

bub::triangular_adaptor<bub::matrix_range< bubMatrix >,bub::upper>
SolverHierarchicalInequalities::
accessRh( void )
{
  bub::matrix_range< bubMatrix >
    Rhup( Rh,rangeh(),rangeh() );
  return Rhup;
}

/* Assuming a diagonal-ordered triangular matrix. */
template< typename bubTemplateMatrix >
unsigned int SolverHierarchicalInequalities::
rankDetermination( const bubTemplateMatrix& A,
                   const double threshold )
{
  unsigned int res = 0;
  for( unsigned int i=0;i<std::min(A.size1(),A.size2());++i )
    { if( fabs(A(i,i))>threshold ) res++; else break; }
  return res;
}

/* ---------------------------------------------------------- */
#ifdef VP_DEBUG
void SolverHierarchicalInequalities::
displayConstraint( ConstraintList & cs )
{
  for( ConstraintList::iterator iter=cs.begin();
       iter!=cs.end();++iter )
    {
      ConstraintMem & ci = *iter;
      sotDEBUG(25) << ci << std::endl;
    }
}
#else
void SolverHierarchicalInequalities::
displayConstraint( ConstraintList & )
{
}
#endif //#ifdef VP_DEBUG



void SolverHierarchicalInequalities::
printDebug( void )
{
#ifdef VP_DEBUG
  sotDEBUG(15) << "constraintSactive:"<<std::endl;
  for( ConstraintRefList::const_iterator iter=constraintSactive.begin();
       iter!=constraintSactive.end();++iter )
    {
      ConstraintMem & cs = **iter;
      sotDEBUG(15) << "+"<<cs << std::endl;
    }

  sotDEBUG(45) << "Sconstraints :: "<< std::endl;
  displayConstraint(constraintS);

  sotDEBUG(25) << "Hconstraints :: "<< std::endl;
  displayConstraint(constraintH);

  bubMatrix Jh( rankh,nJ ); /***/std::fill(Jh.data().begin(),Jh.data().end(),-1);
  for( ConstraintList::iterator iter=constraintH.begin();
       iter!=constraintH.end();++iter )
    {
      ConstraintMem & cs = *iter;
      if( cs.active && cs.rankIncreaser )
        {
          if(cs.notToBeConsidered) continue;
          if(cs.range<rankh) {bub::row(Jh,cs.range).assign( cs.Ji ); }
          else {sotDEBUG(1)<<"!!!" <<cs<<std::endl;}
        }
    }
  sotDEBUG(15) << "rankh = " << rankh << std::endl;
  sotDEBUG(15) << "Jh = " << (MATLAB)Jh << std::endl;
  sotDEBUG(15) << "Qh = " << MATLAB(Qh,nJ) << std::endl;
  sotDEBUG(15) << "QhJs = " << (MATLAB)bub::subrange(QhJs,0,nJ,0,sizes) << std::endl;
  sotDEBUG(15) << "QhJsU = " << (MATLAB)bub::subrange(QhJsU,0,nJ,0,sizes) << std::endl;
  sotDEBUG(15) << "u0 = " << MATLAB(u0) << std::endl;
  {
    bubMatrix toto(nJ,nJ); toto = bub::prod(QhJs,bub::trans(QhJs));
    bubMatrix tata(nJ,nJ); tata = bub::prod(QhJsU,bub::trans(QhJsU));
    toto-=tata;
    sotDEBUG(45) << "dQJJQ = " << (MATLAB)toto << std::endl;
  }

  bubMatrix Jht(bub::trans(Jh));
  Qh.multiplyRightTranspose(Jht);
  bub::matrix_range<bubMatrix> Jht0(Jht,rangeh(),bub::range(0,rankh));
  sotDEBUG(55) << "QJh = " << (MATLAB)Jht0 << std::endl;
  Jht0-=bub::matrix_range<bubMatrix>(Rh,rangeh(),rangeh());
  sotDEBUG(55) << "Rh = " << (MATLAB)Rh << std::endl;
  sotDEBUG(55) << "Rh_QhJh = " << (MATLAB)Jht0 << std::endl;
  sotDEBUG(15) << "||Rh_QhJh|| = " << bub::norm_1(Jht0) << std::endl;

  bubMatrix Js( sizes,nJ );
  for( ConstraintRefList::iterator iter=constraintSactive.begin();
       iter!=constraintSactive.end();++iter )
    {
      ConstraintMem & cs = **iter;
      if( cs.active )
        {
          if(cs.range<sizes) {bub::row(Js,cs.range).assign( cs.Ji ); }
          else {sotDEBUG(1)<<"!!!" <<cs<<std::endl;}
        }
    }
  sotDEBUG(45) << "Js = " << (MATLAB)Js << std::endl;
  sotDEBUG(45) << "QhJs = " << (MATLAB)QhJs << std::endl;

  bubMatrix Jst(bub::trans(Js));
  Qh.multiplyRightTranspose(Jst);
  Jst-=bub::subrange(QhJs,0,nJ,0,sizes);
  sotDEBUG(15) << "||Rs_QhJs|| = " << bub::norm_1(Jst) << std::endl;



#endif //ifdef VP_DEBUG
}

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */

void SolverHierarchicalInequalities::
warmStart( void )
{
  if(! warmStartReady ) return; 
  ConstraintRefList toInactivateCH;
  for( std::vector<ConstraintRef>::const_iterator iter=toInactivate.begin();
       iter!=toInactivate.end();++iter )
    {
      toInactivateCH.push_back( &constraintH[iter->id] );
    }
  if( toInactivateCH.size()>0 ) forceDowndateHierachic(toInactivateCH);

  applyFreeSpaceMotion(du0);

  ConstraintRefList toActivateCH;
  ConstraintMem::BoundSideVector toActivateSide;
  for( std::vector<ConstraintRef>::const_iterator iter=toActivate.begin();
       iter!=toActivate.end();++iter )
    {
      toActivateCH.push_back( &constraintH[iter->id] );
      toActivateSide.push_back( iter->side );
    }
  if( toActivateCH.size()>0 )forceUpdateHierachic(toActivateCH,toActivateSide);
}

void SolverHierarchicalInequalities::
applyFreeSpaceMotion( const bubVector& _du )
{
  printDebug();
  du.assign(_du);
  sotDEBUG(15) << "du = " << (MATLAB)du << std::endl;
  /* Multiply by Ph. */
  Qh.multiplyRangeLeft(du,rankh,0);
  Qh.multiplyRight(du);
  sotDEBUG(15) << "Phdu = " << (MATLAB)du << std::endl;

  double tau;
  selecActivationHierarchic(tau);
  sotDEBUG(5) << "Warm start du limited by tau=" << tau << std::endl;
  du*=tau;
  u0+=du;
}

void SolverHierarchicalInequalities::
forceUpdateHierachic( ConstraintRefList& toUpdate,
                      const ConstraintMem::BoundSideVector& boundSide )
{
  {
    sotDEBUG(5) << "Now activating { ";
    ConstraintMem::BoundSideVector::const_iterator iterBound = boundSide.begin();
    for( ConstraintRefList::const_iterator iterCH = toUpdate.begin();
         toUpdate.end()!=iterCH;++iterCH,++iterBound )
      { sotDEBUGMUTE(5) << *iterBound  << (*iterCH)->constraintRow << ", "; }
    sotDEBUGMUTE(5) << " }" << std::endl;
  }

  sotDEBUG(15) << "/* Create the matrix Js by concatenation of matrix cs.Ji. */" << std::endl;
  unsigned int sizes=toUpdate.size();
  bubMatrix _Jse(sizes,nJ); bubVector _ese(sizes);

  unsigned int col=0;
  ConstraintMem::BoundSideVector::const_iterator iterBound = boundSide.begin();
  for( ConstraintRefList::iterator iter=toUpdate.begin();
       toUpdate.end() != iter;++iter,++iterBound )
    {
      ConstraintMem & cs = **iter;
      if(! cs.active )
        {
          sotDEBUG(1) << "Activation WSH <" << *iterBound
                      << cs.constraintRow << ">" << std::endl;

          sotDEBUG(45) <<cs<<std::endl;
          bub::row(_Jse,col).assign(cs.Ji);
          if( (*iterBound)==ConstraintMem::BOUND_INF )
            _ese(col)=cs.eiInf; else _ese(col)=cs.eiSup;
          col++;
          cs.active=false; cs.notToBeConsidered=true;
          cs.activeSide = *iterBound;
        }
      else
        {
          sotDEBUG(1) << "Error: <" << cs.constraintRow << "> is already active. "
                      << std::endl;
        }
    }
  sotDEBUG(45) <<"Jwsh = " <<(MATLAB)_Jse<<std::endl;
  if(sizes>col) { sizes=col; _Jse.resize(sizes,nJ,true); _ese.resize(sizes,true); }
  sotDEBUG(25) <<"Jwsh = " <<(MATLAB)_Jse<<std::endl;

  sotDEBUG(15) << "/* Solve these constraints. */" << std::endl;
  bubMatrix _Jsi(0,0); bubVector _esiInf(0),_esiSup(0);
  std::vector<ConstraintMem::BoundSideType> _esiBound(0);
  solve(_Jse,_ese,_Jsi,_esiInf,_esiSup,_esiBound,false);

  sotDEBUG(15) << "/* Copy S in H. */" << std::endl;
  /* Pe is the range of the actual column of QR in the original
   * matrix: QR[:,i] == Jse'[:,pe(i)]. */
  sotDEBUG(15) << "orderS = " << (MATLAB)orderS << std::endl;
//   for( ConstraintList::iterator iter=constraintS.begin();
//        constraintS.end() != iter;++iter )
//     {
//       ConstraintMem & cs = *iter;
//       ConstraintMem & ch = *toUpdate[cs.constraintRow];
  unsigned int rangeInS=0;
  for( ConstraintRefList::iterator iter=toUpdate.begin();
       toUpdate.end() != iter;++iter )
    {
      ConstraintMem & ch = **iter;
      if(! ch.notToBeConsidered ) continue;
      ConstraintMem & cs = constraintS[rangeInS++];
      ch.range=cs.range+rankh;
      ch.rankIncreaser=cs.rankIncreaser;
      ch.equality=false; // TODO: ?? lock a constraint ??
      ch.notToBeConsidered = false; ch.active = true;
      sotDEBUG(15) << "Add eq FR: " << ch << std::endl;
    }

  sotDEBUG(15) << "/* Copy a triangular of Rs in Rh. */" << std::endl;
  sotRotationComposed Qlast;
  for( unsigned int i=0;i<ranks;++i )
    {
      typedef bub::matrix_column<bubMatrixQRWide> bubQJsCol;
      bubQJsCol QJk(QhJs,orderS(i)); Qlast.multiplyLeft(QJk);
      sotDEBUG(45) << "QhJs_" << i << " = " << (MATLAB)QJk << std::endl;
      bub::vector_range<bubQJsCol> Ftdown(QJk,freerange());
      double beta;
      double normSignFt  // Norm with sign!
        = sotRotationSimpleHouseholder::householderExtraction(Ftdown,beta,THRESHOLD_ZERO);
      sotDEBUG(45) << "Ft_" << i << " = " << (MATLAB)Ftdown << std::endl;
      sotDEBUG(45) << "b_" << i << " = " << beta << std::endl;
      if(fabs(beta)>THRESHOLD_ZERO)
        { Qlast.pushBack(sotRotationSimpleHouseholder(Ftdown,beta)); }

      bub::matrix_column<bubMatrix> Rhk(Rh,rankh);
      bub::project(Rhk,rangeh()).assign( bub::project(QJk,rangeh()) );
      bub::project(Rhk,freerange()).assign( bub::zero_vector<double>(freeRank) );
      Rh(rankh,rankh) = normSignFt;
      sotDEBUG(45) << "Rh_" << i << " = " << (MATLAB)Rhk << std::endl;

      rankh++; freeRank--;
    }
  Qh.pushBack(Qlast);
}

void SolverHierarchicalInequalities::
forceDowndateHierachic( ConstraintRefList& toDowndate )
{
  {
    sotDEBUG(5) << "Now inactivating { ";
    for( ConstraintRefList::const_iterator iterCH = toDowndate.begin();
         toDowndate.end()!=iterCH;++iterCH )
      { sotDEBUGMUTE(5) << (*iterCH)->constraintRow << ", "; }
    sotDEBUGMUTE(5) << " }" << std::endl;
  }

  unsigned int rankhDec = 0;

  sotDEBUG(15) << "/* Precompute the new structure of Rh matrix. */" << std::endl;
  std::vector<bool> colDeleted(rankh,false);
  std::vector<unsigned int> colOffset(rankh);
  for( ConstraintRefList::iterator iter=toDowndate.begin();
       toDowndate.end() != iter;++iter )
    {
      ConstraintMem & cs = **iter;
      if( cs.active && cs.rankIncreaser )
        {
          sotDEBUG(1) << "Inactivation WSH <" << cs.activeSide << cs.constraintRow << ">" << std::endl;
          sotDEBUG(45) << "Force downdate cs " << cs << std::endl;
          const unsigned int range = cs.range;
          colDeleted[range]=true;
          ++ rankhDec;
          cs.active = false;
        }
      else
        { sotDEBUG(1) << "Can not force down inactive constraint " << cs.constraintRow << std::endl; }
    }

  {
    unsigned int offset = 0;
    for( unsigned int i=0;i<rankh;++i )
      {if(colDeleted[i]) offset++; else colOffset[i]=offset;}
  }

  for( ConstraintList::iterator iter=constraintH.begin();
       constraintH.end() != iter;++iter )
    {
      ConstraintMem & cs = *iter;
      if( cs.active )
        {
          const unsigned int range = cs.range;
          cs.range -= colOffset[range];
        }
    }

  sotDEBUG(15) << "/* Reduce the Rh matrix. */" << std::endl;
  for( unsigned int j=0;j<rankh;++j )
    {
      const unsigned int offset = colOffset[j];
      if( (!colDeleted[j])&&(offset>0) )
        {
          sotDEBUG(15) << "Copy " << j << " in " << j-offset << std::endl;
          for( unsigned int i=0;i<rankh;++i )
            {
              Rh(i,j-offset)=Rh(i,j);
            }
        }
    }
  rankh-=rankhDec;
  freeRank+=rankhDec;

  sotDEBUG(15) << "/* Correct the pseudo-hessenberg matrix. */" << std::endl;
  bub::matrix_range< bubMatrix > Rhlim( Rh,bub::range(0,rankh+rankhDec),rangeh() );
  sotDEBUG(15) << "Rh_hess = " << (MATLAB)Rhlim << std::endl;
  for( unsigned int j=0;j<rankh+rankhDec;++j )
    {
      const unsigned int offset = colOffset[j];
      if( (!colDeleted[j])&&(offset>0) )
        {
          for( unsigned int i=0;i<offset;++i )
            {
              sotRotationSimpleGiven Gr( Rh,j-i-1,j-i,j-offset );  Gr.inverse();
              Gr.multiplyRightTranspose(Rhlim);
              Qh.pushBack(Gr);
              sotDEBUG(55) << "Rh_down" << j << "x"<<i<<" = " << (MATLAB)Rh << std::endl;
              sotDEBUG(55) << "Gr" << j << "x"<<i<<" = " << Gr << std::endl;
            }
        }
    }
  sotDEBUG(5) << "Rh_down = " << (MATLAB)accessRh() << std::endl;

  sotDEBUG(15) << "/* Activate rankd-def constraints. */" << std::endl;
  for( ConstraintList::iterator iter=constraintH.begin();
       constraintH.end() != iter;++iter )
    {
      ConstraintMem & cs = *iter;
      if( cs.active&&(!cs.rankIncreaser) )
        {
          bubVector QJk(cs.Ji); Qh.multiplyLeft(QJk);
          if(fabs(QJk(rankh))>THRESHOLD_ZERO)
            {
              sotDEBUG(5) << "No rank loss when force downdating => "
                          << cs.constraintRow << std::endl;
              bub::vector_range<bubVector> Ftdown(QJk,freerange());
              double beta;
              double normSignFt  // Norm with sign!
                = sotRotationSimpleHouseholder::householderExtraction(Ftdown,beta,THRESHOLD_ZERO);
              sotDEBUG(45) << "Ft_" << " = " << (MATLAB)Ftdown << std::endl;
              sotDEBUG(45) << "b_" << " = " << beta << std::endl;
              if(fabs(beta)>THRESHOLD_ZERO)
                { Qh.pushBack(sotRotationSimpleHouseholder(Ftdown,beta)); }

              bub::matrix_column<bubMatrix> Rhk(Rh,rankh);
              bub::project(Rhk,rangeh()).assign( bub::project(QJk,rangeh()) );
              bub::project(Rhk,freerange()).assign( bub::zero_vector<double>(freeRank) );
              Rh(rankh,rankh) = normSignFt;
              sotDEBUG(45) << "Rh" << " = " << (MATLAB)Rhk << std::endl;

              cs.range = rankh; cs.rankIncreaser =true;
              rankh++; freeRank--;
            }
        }
    }
}

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
#ifdef WITH_CHRONO
#  define SOT_DEFINE_CHRONO                     \
  struct timeval t0,t1;                         \
  double dtsolver
#  define SOT_INIT_CHRONO                       \
  gettimeofday(&t0,NULL)
#  define SOT_CHRONO(txt)                                               \
  gettimeofday(&t1,NULL);                                               \
  dtsolver = (t1.tv_sec-t0.tv_sec) * 1000. + (t1.tv_usec-t0.tv_usec+0.)/1000. ; \
  std::cout << "t_" << txt << " = " << dtsolver << " %ms" << std::endl; \
  gettimeofday(&t0,NULL)
#else // ifdef WITH_CHRONO
#  define SOT_DEFINE_CHRONO
#  define SOT_INIT_CHRONO
#  define SOT_CHRONO(txt)
#endif // ifdef WITH_CHRONO

/* ---------------------------------------------------------- */

SOT_DEFINE_CHRONO
void SolverHierarchicalInequalities::
solve( const bubMatrix& Jse, const bubVector& ese,
       const bubMatrix& Jsi, const bubVector& esiInf, const bubVector& esiSup,
       const std::vector<ConstraintMem::BoundSideType> esiBoundSide,
       bool pushBackAtTheEnd )
{
  std::vector<ConstraintRef> vectVoid;
  solve(Jse,ese,Jsi,esiInf,esiSup,esiBoundSide,vectVoid,pushBackAtTheEnd);
}

void SolverHierarchicalInequalities::
solve( const bubMatrix& Jse, const bubVector& ese,
       const bubMatrix& Jsi, const bubVector& esiInf, const bubVector& esiSup,
       const ConstraintMem::BoundSideVector & esiBoundSide,
       const std::vector<ConstraintRef> & slackActiveWarmStart,
       bool pushBackAtTheEnd )
{
  sotDEBUGIN(1);
  /*!*/SOT_INIT_CHRONO;

  initializeConstraintMemory(Jse,ese,Jsi,esiInf,esiSup,esiBoundSide,slackActiveWarmStart);
  /*!*/SOT_CHRONO("copy");

  /***/sotDEBUG(15) << "/* Initialize [Qs Rs]. */" << std::endl;
  initializeDecompositionSlack();
  /*!*/SOT_CHRONO("init");

  do
    {



      /***/sotDEBUG(15) << "* - LOOP 1  * - * - * - * - * - * - * - * - * - * - * - *" << std::endl;
      /***/sotDEBUG(15) << "/* Active/inactive H-constraint. */" << std::endl;
      if(Hactivation)
        {
          updateConstraintHierarchic( HactivationRef,HactivationSide );
          /*!*/SOT_CHRONO("UpdateH");
          Hactivation=false;
          /***/sotDEBUG(15) << "/* Update Rank-1 [Qs Rs]. */" << std::endl;
          updateRankOneDowndate();
          /*!*/SOT_CHRONO("rankoneDowndate");

        }
      else if( Hinactivation )
        {
          downdateConstraintHierarchic( HinactivationRef );
          /*!*/SOT_CHRONO("DowndateH");
          Hinactivation=false;
          /***/sotDEBUG(15) << "/* Update Rank-1 [Qs Rs]. */" << std::endl;
          updateRankOneUpdate();
          /*!*/SOT_CHRONO("rankoneUpdate");
        }

      /***/sotDEBUG(15) << "/* Activate/inactivate S-constraint. */" << std::endl;
      if(Sactivation)
        {
          /***/sotDEBUG(15) << "/* Activate S-constraint. */" << std::endl;
          updateConstraintSlack( SactivationRef,SactivationSide );
          /*!*/SOT_CHRONO("UpS");
          /***/sotDEBUG(15) << "/* Activate S-constraint. */" << std::endl;
        }
      else if( Sinactivation )
        {
          /***/sotDEBUG(15) << "/* Inactivate S-constraint. */" << std::endl;
          downdateConstraintSlack( SinactivationRef );
          /*!*/SOT_CHRONO("DownS");
        }
      /***/printDebug();
      /*!*/SOT_CHRONO("Print");

      /***/sotDEBUG(15) << "/* Compute Primal. */" << std::endl;
      computePrimal();
      /*!*/SOT_CHRONO("primal");

      Sactivation=false;Sinactivation=false; Hactivation=false;Hinactivation=false;

      /***/sotDEBUG(15) << "/* Compute Slack. */" << std::endl;
      u0+=du; computeSlack(); u0-=du;
      /*!*/SOT_CHRONO("Slack");
      /***/sotDEBUG(15) << "/* Selec S-activation. */" << std::endl;
      Sactivation = selecActivationSlack();
      /*!*/SOT_CHRONO("SelecAcS");
      if(Sactivation) {  continue; }

      /***/sotDEBUG(15) << "/* Selec H-activation. */" << std::endl;
      double tau;
      Hactivation = selecActivationHierarchic(tau);
      /*!*/SOT_CHRONO("SelecAcH");
      if( Hactivation )
        {
          /***/sotDEBUG(15) << "tau = " << tau << std::endl;
          du*=tau;
          u0+=du;
          /*!*/SOT_CHRONO("IncUtau");
        }
      else
        {
          u0+=du;
          /*!*/SOT_CHRONO("IncUsimple");

          //             /***/sotDEBUG(15) << "/* Compute Slack. */" << std::endl;
          //             computeSlack();
          //             /*!*/SOT_CHRONO("Slack");
          //             /***/sotDEBUG(15) << "/* Selec S-activation. */" << std::endl;
          //             Sactivation = selecActivationSlack();
          //             //if( Sactivation ) continue;
          //             /*!*/SOT_CHRONO("SelecAcS");
          //             /***/sotDEBUG(15) << "/* Selec S-inactivation. */" << std::endl;
          //             Sinactivation = selecInactivationSlack();
          //             //if( Sinactivation ) continue;
          //             /*!*/SOT_CHRONO("SelecInacS");

          /***/sotDEBUG(15) << "/* Compute Lagrangian. */" << std::endl;
          computeLagrangian();
          /*!*/SOT_CHRONO("Lagrang");
          /***/sotDEBUG(15) << "/* Selec H-inactivation. */" << std::endl;
          Hinactivation = selecInactivationHierarchic();
          /*!*/SOT_CHRONO("SelecInacH");

          if(! Hinactivation )
            {
              /***/sotDEBUG(15) << "/* Selec S-inactivation. */" << std::endl;
              Sinactivation = selecInactivationSlack();
              /*!*/SOT_CHRONO("SelecInacS");
              if( Sinactivation ) { continue; }
            }
        }
      /***/sotDEBUG(3) << "u = " << (MATLAB)u0 << std::endl;

    } while(Sactivation||Sinactivation||Hactivation||Hinactivation);

  /* Push-back S-constraint. */
  /* Push-back [ Qs Rs ]. */
  /*!*/SOT_CHRONO("void");
  if(pushBackAtTheEnd) pushBackSlackToHierarchy();
  /*!*/SOT_CHRONO("PBSlack");
  printDebug();
  /***/sotDEBUGOUT(1);
}

/* ---------------------------------------------------------- */
void SolverHierarchicalInequalities::
initializeConstraintMemory( const bubMatrix& Jse, const bubVector& ese,
                            const bubMatrix& Jsi, const bubVector& esiInf, const bubVector& esiSup,
                            const ConstraintMem::BoundSideVector& esiBoundSide,
                            const std::vector<ConstraintRef>& warmStartSide )
{
  // TODO ... activate those protection.
  //     if(!( (esiInf.size()==esiSup.size())&&(esiInf.size()==esiBoundSide.size())&&(esiInf.size()==Jsi.size1())) )
  //       {
  //         sotERROR << "Error in the size of I matrices. " << std::endl;
  //         throw "Error in the size of I matrices. ";
  //       }
  //     if(!( (ese.size()==Jse.size1())) )
  //       {
  //         sotERROR << "Error in the size of E matrices. " << std::endl;
  //         throw "Error in the size of E matrices. ";
  //       }
  sotDEBUG(45) << "Je = " << (MATLAB)Jse << std::endl;
  sotDEBUG(45) << "Ji = " << (MATLAB)Jsi << std::endl;
  {
    sotDEBUG(25) <<"Warm start active slack = { ";
    for( std::vector<ConstraintRef>::const_iterator iter=warmStartSide.begin();
         iter!=warmStartSide.end();++iter )
      {     sotDEBUGMUTE(25) << (*iter) << ", "; }
    sotDEBUGMUTE(25) << " }" << std::endl;
  }

  Hactivation=false; Hinactivation=false;
  constraintS.clear(); constraintS.resize(ese.size()+Jsi.size1());
  constraintSactive.resize(0);

  sotDEBUG(15) <<"/* Copy the Jacobian to memory. */" << std::endl;
  unsigned int row = 0;
  const unsigned int sizee=ese.size();
  for( ConstraintList::iterator iter=constraintS.begin();
       iter!=constraintS.end();++iter,++row )
    {
      ConstraintMem & cs = *iter;

      cs.Ji.resize(nJ,false);
      cs.constraintRow=row;
      cs.active=false;
      cs.range = 0;  cs.rankIncreaser = false;
      if( row<sizee )
        {
          cs.Ji.assign(bub::row(Jse,row)); cs.eiInf=ese(row); cs.equality=true;
          cs.active = true;
          cs.boundSide = cs.activeSide = ConstraintMem::BOUND_INF;
        }
      else
        {
          const unsigned int rowi = row-sizee;
          cs.Ji.assign(bub::row(Jsi,rowi));
          cs.boundSide = ConstraintMem::BOUND_VOID;
          if( (esiBoundSide[rowi]&ConstraintMem::BOUND_INF)&&(!isnan(esiInf(rowi))) )
            {
              cs.eiInf=esiInf(rowi);
              cs.boundSide = (ConstraintMem::BoundSideType)(cs.boundSide|ConstraintMem::BOUND_INF);
            }
          if( (esiBoundSide[rowi]&ConstraintMem::BOUND_SUP)&&(!isnan(esiSup(rowi))) )
            {
              cs.eiSup=esiSup(rowi);
              cs.boundSide = (ConstraintMem::BoundSideType)(cs.boundSide|ConstraintMem::BOUND_SUP);
            }
          cs.equality=false;
        }
      sotDEBUG(15) << "Init " << cs << std::endl;
    }

  /* Activation Warm start. */
  {
    for( std::vector<ConstraintRef>::const_iterator iter=warmStartSide.begin();
         iter!=warmStartSide.end();++iter )
        {
          if( iter->id<constraintS.size() )
            {
              constraintS[iter->id].active=true;
              constraintS[iter->id].activeSide=iter->side;
              sotDEBUG(1) << "Activation WS < " << *iter << ">" << std::endl;
            }
        }
    }
}

void SolverHierarchicalInequalities::
initializeDecompositionSlack(void)
{
  QhJsU.resize(nJ,constraintS.size(),false); QhJsU.clear();
  QhJs.resize(nJ,constraintS.size(),false); QhJs.clear();
  Sactivation=false; Sinactivation=false;

  sotDEBUG(15) <<"/* 1.a Compute the limited matrix. */" << std::endl;
  sizes=0;
  for( ConstraintList::iterator iter=constraintS.begin();
       iter!=constraintS.end();++iter )
    {
      ConstraintMem & cs = *iter;
      if(cs.active)
        { bub::column(QhJsU,sizes++) = cs.Ji; }
    }
  bubMatrixQR QhJeU(QhJsU,fullrange(),bub::range(0,sizes));
  /*!*/SOT_CHRONO("BuildJs");
  Qh.multiplyRightTranspose(QhJeU);
  sotDEBUG(15) << "QhJs = " << (MATLAB)QhJeU << std::endl;
  /*!*/SOT_CHRONO("QhJs");

  if( freeRank==0 )
    {
      ranks=0; orderS = bubOrder(0);
      bub::project(QhJs,fullrange(),bub::range(0,sizes)) .assign(QhJeU);
      /* Initialize the constraintMem. */
      unsigned int row=0;
      for( ConstraintList::iterator iter=constraintS.begin();
           iter!=constraintS.end();++iter,++row )
        {
          ConstraintMem & cs = *iter; cs.range=row;
          if( cs.active ){ constraintSactive.push_back(&cs); }
        }
      sotDEBUG(15)<<"Freerank null, initial decomposition void."<<std::endl;
      return;
    }

  sotDEBUG(15) <<"/* 1.b Compute the QR decomposition. */" << std::endl;
  bubMatrixQR QRs(QhJsU,bub::range(rankh,nJ),bub::range(0,sizes));
  bubVector betas(sizes); betas.clear();
  bub::vector<int> orderSe(sizes); orderSe.clear();
  boost::numeric::bindings::lapack::geqp(QRs,orderSe,betas);
  ranks = rankDetermination(QRs);
  sotDEBUG(15) << "ranks = " << ranks << std::endl;
  sotDEBUG(15) << "QRs = " << (MATLAB)QhJeU << std::endl;
  sotDEBUG(15) << "orderSe = " << orderSe << std::endl;
  /*!*/SOT_CHRONO("QRdecomp");

  sotDEBUG(15) <<"/* 1.c Organize into Q2 and R2. */" << std::endl;
  /* If Je is rank deficient, then the last <m-r> householder vectors
   * do not affect the range basis M (where Q=[M N]). If M is constant
   * wrt these vector, then Span(N)=orth(Span(M)) is also constant, even
   * if N is non constant, and thus these vectors can be neglect. */
  sotRotationComposed Qs; Qs.householderQRinit( QRs,betas,ranks );
  Qh.pushBack(Qs);

  /* Remove the lower triangle (householder vectors). */
  for( unsigned int j=0;j<sizes;++j )
    for( unsigned int i=j+1;i<freeRank;++i ) QRs(i,j)=0;
  sotDEBUG(45) << "Qs = " << MATLAB(Qs,nJ) << std::endl;
  sotDEBUG(15) << "QJs = " << (MATLAB)QhJeU << std::endl;
  /*!*/SOT_CHRONO("QRorganize");

  /* Save QhJs (without U). */
  for( unsigned int j=0;j<sizes;++j )
    {
      for( unsigned int i=0;i<rankh;++i ) { QhJs(i,j) = QhJeU(i,orderSe(j)-1); }
      for( unsigned int i=rankh;i<nJ;++i ){ QhJs(i,j) = QhJeU(i,j); }
    }
  QhJeU = bub::project(QhJs,fullrange(),bub::range(0,sizes));
  orderS = bubOrder(ranks); for( unsigned int i=0;i<ranks;++i ) orderS[i]=i;
  sotDEBUG(15) << "QhsJs = " << (MATLAB)QhJeU << std::endl;
  sotDEBUG(15) << "orderS = " << (MATLAB)orderS << std::endl;
  /*!*/SOT_CHRONO("SaveQhJs");

  /* Full range decomposition R0 = Qh.Js.U . */
  regularizeQhJsU();
  /*!*/SOT_CHRONO("FullRankDecompo");

  /* Initialize the constraintMem. */
  ConstraintRefList activeOrderRaw;
  for( ConstraintList::iterator iter=constraintS.begin();
       iter!=constraintS.end();++iter )
    {
      ConstraintMem & cs = *iter;
      if( cs.active ) activeOrderRaw.push_back(&cs);
    }
  /* Push the constraint in cSactive in the order decided by the QR decomposition. */
  for( unsigned int i=0;i<sizes;++i )
    {
      ConstraintMem & cs = *activeOrderRaw[orderSe[i]-1];
      if(i<ranks){ cs.rankIncreaser = true; }
      cs.range = i;
      constraintSactive.push_back(&cs);
      sotDEBUG(15) << "Activate " << cs << std::endl;
    }
  /*!*/SOT_CHRONO("InitActiveMem");
}




/* ---------------------------------------------------------- */
/* <constraintId> is the number of the constraint in the constraintH list. */
void SolverHierarchicalInequalities::
updateConstraintHierarchic( const unsigned int constraintId,
                            const ConstraintMem::BoundSideType side )
{
  sotDEBUG(15) << "khup = " << constraintId << std::endl;
  ConstraintMem & chup = constraintH[constraintId];

  /* Compute the limited Jacobian. */
  bub::matrix_column<bubMatrix> QtJt(Rh,rankh);
  QtJt.assign(chup.Ji);
  Qh.multiplyLeft(QtJt);
  sotDEBUG(15) << "Jk = " << (MATLAB)chup.Ji << std::endl;
  sotDEBUG(15) << "QJk = " << (MATLAB)QtJt << std::endl;

  chup.rankIncreaser=false;
  bub::triangular_adaptor<bub::matrix_range< bubMatrix >,bub::upper>
    Rhprev = accessRh();
  if(rankh<nJ)
    {
      lastRotation.clear();
      /* Given rotation to nullify the new column. */
      for( unsigned int row=nJ-1;row>rankh;row-- )
        {
          if( fabs(QtJt(row))>THRESHOLD_ZERO )
            {
              chup.rankIncreaser=true;
              sotRotationSimpleGiven gr(Rh,row-1,row,rankh);
              gr.inverse();
              gr.multiplyLeft(QtJt);
              lastRotation.pushBack(gr);
            }
        }
      sotDEBUG(45) << "Add GR = " << lastRotation << std::endl;
      Qh.pushBack(lastRotation);

      /* Update information about the constraint. */
      chup.active = true;  chup.rankIncreaser=true; chup.activeSide = side;
      chup.range=rankh;
      /* Increase the number of data. */
      rankh++; freeRank--; freeRankChange = -1;
    }

  if(!chup.rankIncreaser)
    {
      std::cerr << "Error, H-up should always be full rank!" << std::endl;
      throw "Error, H-up should always be full rank!";
    }

  sotDEBUG(15) << "Rh = " << (MATLAB)accessRhConst() << std::endl;
  sotDEBUG(15) << "Constraint = " << chup << std::endl;
}


/* <constraintId> is the number of the constraint in the constraintH list. */
void SolverHierarchicalInequalities::
downdateConstraintHierarchic( const unsigned int kdown )
{
  ConstraintMem & cdown = constraintH[kdown];
  sotDEBUG(15) << "kdown = " << kdown << std::endl;
  if(! constraintH[kdown].active) return;

  /* Rh is hessenberg: trigonalize. */
  lastRotation.clear();
  for( unsigned int i=cdown.range+1;i<rankh;++i )
    {
      sotRotationSimpleGiven Gr( Rh,i-1,i,i );  Gr.inverse();
      Gr.multiplyRightTranspose(Rh);
      lastRotation.pushBack(Gr);
    }
  Qh.pushBack(lastRotation);
  /* Shift the column after kdown to the left. */
  for( ConstraintList::iterator iter=constraintH.begin();
       iter!=constraintH.end();++iter )
    {
      ConstraintMem & cs = *iter;
      if( cs.range>cdown.range ) cs.range--;
    }
  /* Remove the columnd <kdown> from Rh. */
  //bubClearMatrix(bub::column(Rh,cdown.range));
  //bubClearMatrix(bub::column(Rh,cdown.range).data());
  //bub::vector_assign_scalar<bub::scalar_assign>(bub::column(Rh,cdown.range),0);
  bub::column(Rh,cdown.range).assign(bub::zero_vector<double>(Rh.size1()));
  bubRemoveColumn(accessRh(),cdown.range);
  rankh--; freeRank++; freeRankChange = +1;
  cdown.active = false;
  sotDEBUG(5) << "Rhdown = " << (MATLAB)accessRhConst() << std::endl;
  sotDEBUG(5) << "Qhdown = " << MATLAB(Qh,nJ) << std::endl;
  sotDEBUG(5) << "Qhlast = " << MATLAB(lastRotation,nJ) << std::endl;
  sotDEBUG(5) << "Qhlast = " << lastRotation << std::endl;

  /* Check for active rank-def constraint. */
  for( ConstraintList::iterator iter=constraintH.begin();
       iter!=constraintH.end();++iter )
    {
      ConstraintMem & cs = *iter;
      if( cs.active && (!cs.rankIncreaser) )
        {
          bubVector QJk(cs.Ji); Qh.multiplyLeft(QJk);
          if(fabs(QJk(rankh))>THRESHOLD_ZERO)
            { /* Update constraint: Qh'Ji' = [ Mh'Ji' mh'Ji' Nh'Ji],
               * with Nh'Ji=0.
               * Simply add the [Mh'Ji' mh'Ji' ] to Rh. */
              sotDEBUG(5) << "No rank loss when downdating [" << kdown
                          << "] => " << cs.constraintRow << std::endl;
              bub::matrix_column<bubMatrix > Rhk(Rh,rankh);
              bub::project(Rhk,bub::range(0,rankh+1))
                .assign( bub::project(QJk,bub::range(0,rankh+1)));
              cs.range = rankh; cs.rankIncreaser =true;
              rankh++; freeRank--; freeRankChange = 0;
              break;
            }
        }
    }
}


/* ---------------------------------------------------------- */
/* <kup> is the number of the constraint in the constraintS list. */
void SolverHierarchicalInequalities::
updateConstraintSlack( const unsigned int kup,const ConstraintMem::BoundSideType activeSide )
{
  sotDEBUG(15) << "kup = " << activeSide << kup << std::endl;
  ConstraintMem & csup = constraintS[kup];
  //     if( freeRank>0 )
  //       {
  /* Compute the limited Jacobian. */
  bubVector QtJt(nJ); QtJt.assign(csup.Ji);
  Qh.multiplyLeft(QtJt);
  bub::vector_range<bubVector> Ft(QtJt,freerange());
  sotDEBUG(15) << "QsNhJk = " << (MATLAB)Ft << std::endl;

  /* TODO: copy directly ck.Ji in JsU, instead of using the tmp var Ft. */
  bub::matrix_column< bubMatrixQRWide > JskU(QhJsU,sizes);
  bub::project(JskU,rangehs())=bub::project(QtJt,rangehs());
  bub::matrix_column< bubMatrixQRWide > Jsk(QhJs,sizes);
  bub::project(Jsk,rangehs())=bub::project(QtJt,rangehs());

  bool rankDef=true;
  if(ranks+rankh<nJ)
    {
      /* Householder transposition of QJk. */
      bub::vector_range<bubVector> Ftdown(QtJt,bub::range(rankh+ranks,nJ));
      double beta;
      double normSignFt  // Norm with sign!
        = sotRotationSimpleHouseholder::householderExtraction(Ftdown,beta,THRESHOLD_ZERO);
      sotDEBUG(15) << "hk = " << (MATLAB)Ftdown << std::endl;
      sotDEBUG(15) << "betak = " << beta << std::endl;
      sotDEBUG(15) << "normH = " << normSignFt << std::endl;

      /* Add a new column in R. */
      QhJsU(rankh+ranks,sizes) = normSignFt;
      QhJs(rankh+ranks,sizes) = normSignFt;

      /* Add a new rotation in Qh. */
      if( fabs(normSignFt)>THRESHOLD_ZERO )
        { /* If Jk is adding information to the current R. */
          /* Add the new householder vector in Q. */
          if(fabs(beta)>THRESHOLD_ZERO)
            { Qh.pushBack( sotRotationSimpleHouseholder(Ftdown,beta)); }
          /* Update information about the constraint. */
          csup.rankIncreaser = true;
          /* Increase the number of data. */
          ranks++; rankDef=false; orderS+=sizes;
        }
    }
  csup.range = sizes; sizes++;

  sotDEBUG(45) << "QhJs = "  << (MATLAB)QhJs << std::endl;
  sotDEBUG(45) << "QhJsU = " << (MATLAB)QhJsU << std::endl;
  if( rankDef&&(freeRank>0) )
    { /* The new line does not add any information. Regularize the pseudo-triangle. */
      /* Select the non zero part of R. */
      bubMatrixQR QJdown( QhJsU,freeranges(),bub::range(0,sizes));
      bubMatrixQRTri RrRh(QJdown);
      /* Regularize. */
      sotDEBUG(15) << "RrRh = " << (MATLAB)RrRh << std::endl;
      sotRotationComposed Uchol;
      Uchol.regularizeRankDeficientTriangle(RrRh,orderS,sizes-1);
      csup.rankIncreaser = false;
      bubMatrixQR Jsup(QhJsU,rangeh(),bub::range(0,sizes));
      Uchol.multiplyLeft(Jsup);
    }
  sotDEBUG(15) << "kup = "<<kup<<" VS "<<constraintS.size()<<std::endl;
  csup.active = true; csup.activeSide = activeSide;
  constraintSactive.push_back(&csup);
  sotDEBUG(15) << "Rei = " << (MATLAB)accessRsConst() << std::endl;
  sotDEBUG(15) << "Constraint = " << csup << std::endl;
}

/* Regularize from right (Q.J from Q.J.U). */
void SolverHierarchicalInequalities::
regularizeQhJs( void )
{
  typedef bub::matrix_column<bubMatrixQRWide> bubQJsCol;
  sotRotationComposed Qlast;
  for( unsigned int i=0;i<ranks;++i )
    {
      bubQJsCol QJk(QhJs,orderS(i)); Qlast.multiplyLeft(QJk);
      sotDEBUG(45) << "QhJs_" << i << " = " << (MATLAB)QJk << std::endl;
      bub::vector_range<bubQJsCol> Ftdown(QJk,bub::range(rankh+i,std::min(nJ,rankh+ranks+1)));
      double beta;
      double normSignFt
        = sotRotationSimpleHouseholder::householderExtraction(Ftdown,beta,THRESHOLD_ZERO);
      sotDEBUG(45) << "Ft_" << i << " = " << (MATLAB)Ftdown << std::endl;
      sotDEBUG(45) << "b_" << i << " = " << beta << std::endl;
      if(fabs(beta)>THRESHOLD_ZERO)
        { Qlast.pushBack(sotRotationSimpleHouseholder(Ftdown,beta)); }
      std::fill(Ftdown.begin(),Ftdown.end(),0); Ftdown(0)=normSignFt;
    }
  Qh.pushBack(Qlast);
  for( ConstraintRefList::iterator iter=constraintSactive.begin();
       iter!=constraintSactive.end();++iter )
    {
      ConstraintMem & cs = **iter;
      if(!cs.rankIncreaser)
        {
          bubQJsCol QJk(QhJs,cs.range);
          Qlast.multiplyLeft(QJk);
        }
    }
  sotDEBUG(15)<<"QhJs = " << (MATLAB)QhJs << std::endl;
  sotDEBUG(15)<<"Qreg = " << MATLAB(Qlast,nJ) << std::endl;
}

/* Regularize from right (Q.J from Q.J.U). */
void SolverHierarchicalInequalities::
regularizeQhJsU( void )
{
  if( ranks==sizes ) return;
  bubMatrixQR QhJsinf( QhJsU,bub::range(rankh,rankh+ranks),bub::range(0,sizes) );
  bubMatrixQRTri Rrh( QhJsinf );
  bubMatrixQR Jssup(QhJsU,rangeh(),bub::range(0,sizes));
  sotDEBUG(15) << "Rrh = " << (MATLAB)Rrh << std::endl;

  /* Search for the rank def columns. */
  std::vector<bool> rankDefColumns(sizes,true);
  for( unsigned int i=0;i<ranks;++i ) rankDefColumns[orderS(i)]=false;

  /* Regularize the rank def cols. */
  for( unsigned int i=0;i<sizes;++i )
    if( rankDefColumns[i] )
      {
        sotRotationComposed Gr; Gr.regularizeRankDeficientTriangle(Rrh,orderS,i);
        Gr.multiplyLeft(Jssup);
      }
  sotDEBUG(45) << "R0 = " << (MATLAB)Rrh << std::endl;
  sotDEBUG(15) << "QhJsU = " << (MATLAB)bub::subrange(QhJsU,0,nJ,0,sizes) << std::endl;
}



/* <constraintId> is the number of the constraint in the constraintS list. */
void SolverHierarchicalInequalities::
downdateConstraintSlack( const unsigned int kdown )
{
  /* Remove the column. */
  ConstraintMem & cs = constraintS[kdown];
  sotDEBUG(15) << "Downdate S " << cs;
  const unsigned int range = cs.range;
  if( orderS.end()!=std::find(orderS.begin(),orderS.end(),range) )
    { orderS-=range; ranks--; }
  bubRemoveColumn(QhJs,range);    sizes--;
  cs.active=false;
  constraintSactive.erase(constraintSactive.begin()+range);
  for( unsigned int i=0;i<ranks;++i ) if( orderS(i)>range ) orderS(i)--;
  for( ConstraintRefList::iterator iter=constraintSactive.begin();
       iter!=constraintSactive.end();++iter )
    {
      if((*iter)->range>range) (*iter)->range--;
      sotDEBUG(15) << "Remains " << **iter << std::endl;
    }
  /* Triangularize the remaining QhJs matrix. */
  regularizeQhJs();
  /* Check for rank update. */
  for( unsigned int i=0;i<sizes;++i )
    {
      sotDEBUG(45) << "At i=" << i << ",j="<<rankh+ranks
                   <<": qj=" << QhJs(rankh+ranks,i) << std::endl;
      if( (freeRank>ranks)&&(fabs(QhJs(rankh+ranks,i))>THRESHOLD_ZERO) )
        {
          sotDEBUG(15) << "No ranks lost when downdating S." << std::endl;
          orderS+=i; ranks++; constraintSactive[i]->rankIncreaser=true;
          break;
        }
    }

  bub::project(QhJsU,bub::range(0,nJ),bub::range(0,sizes))
    .assign(bub::project(QhJs,bub::range(0,nJ),bub::range(0,sizes)));
  bub::project(QhJs,bub::range(0,nJ),bub::range(sizes,QhJsU.size2()))
    .assign(bub::zero_matrix<double>(nJ,QhJsU.size2()-sizes));
  bub::project(QhJsU,bub::range(0,nJ),bub::range(sizes,QhJsU.size2()))
    .assign(bub::zero_matrix<double>(nJ,QhJsU.size2()-sizes));
  regularizeQhJsU();

  //     std::cerr << "Not implemented yet (RotationSimple l" << __LINE__
  //               << ")." << std::endl;
  //     throw  "Not implemented yet.";
}


/* ---------------------------------------------------------- */
void SolverHierarchicalInequalities::
updateRankOneDowndate( void )
{
  sotDEBUG(15) << "/* Apply the last corrections of Qh. */"<<std::endl;
  bubMatrixQR QhJranksU(QhJsU,fullrange(),bub::range(0,sizes));
  lastRotation.multiplyRightTranspose(QhJranksU);
  bubMatrixQR QhJranks(QhJs,fullrange(),bub::range(0,sizes));
  lastRotation.multiplyRightTranspose(QhJranks);

  sotDEBUG(15) << "/* Check for the full rank of Rs. */"<<std::endl;
  sotDEBUG(25) << "beforeQhJsU = " << (MATLAB)bub::subrange(QhJsU,0,nJ,0,sizes) << std::endl;
  sotDEBUG(25) << "beforeQhJs = " << (MATLAB)bub::subrange(QhJs,0,nJ,0,sizes) << std::endl;
  bubMatrixQR Rsno(QhJsU,bub::range(rankh,std::min(nJ,rankh+ranks)),bub::range(0,sizes));
  for( unsigned int i=0;i<ranks;++i )
    {
      const unsigned int col = orderS[i];
      if( (i>=freeRank)||(fabs(Rsno(i,col))<THRESHOLD_ZERO) )
        {
          sotDEBUG(5) << "Rank lost at " << i << "." << std::endl;
          orderS-=col; ranks--;
          constraintSactive[col]->rankIncreaser = false;
          if( i>0 )
            {
              sotRotationComposed Gr;
              bubMatrixQR
                Ri(QhJsU,bub::range(rankh,std::min(nJ,rankh+i)),bub::range(0,sizes));
              Gr.regularizeRankDeficientTriangle(Ri,orderS,col);
              bubMatrixQR Jsup(QhJsU,rangeh(),bub::range(0,sizes));
              Gr.multiplyLeft(Jsup);
            }

          break;
        }
    }
  sotDEBUG(5) << "unrQhJsU = "<< (MATLAB)QhJsU << std::endl;
  sotDEBUG(15) << "/* Regularize the Hessenberg matrix. */"<<std::endl;
  for( unsigned int i=0;i<ranks;++i )
    {
      const unsigned int col = orderS[i];
      if( (i+1<freeRank)&&(fabsf(QhJsU(rankh+i+1,col))>THRESHOLD_ZERO) )
        {
          sotRotationSimpleGiven Gr( QhJsU,rankh+i,rankh+i+1,col ); Gr.inverse();
          Gr.multiplyRightTranspose(QhJsU);
          Gr.multiplyRightTranspose(QhJs);
          Qh.pushBack(Gr);
        }
      sotDEBUG(45) << "QhJs = " << (MATLAB)QhJs << std::endl;
      sotDEBUG(15) << "QhJsU = " << (MATLAB)QhJsU << std::endl;
      sotDEBUG(5) << "Rs = " << (MATLAB)accessRsConst() << std::endl;
    }
}

void SolverHierarchicalInequalities::
updateRankOneUpdate( void )
{
  sotDEBUG(15) << "ranks = " << ranks << std::endl;
  sotDEBUG(15) << "/* Apply the last corrections of Qh. */"<<std::endl;
  bubMatrixQR QhJranksU(QhJsU,fullrange(),bub::range(0,sizes));
  sotDEBUG(25) << "beforeQhJsU = " << (MATLAB)QhJranksU << std::endl;
  lastRotation.multiplyRightTranspose(QhJranksU);
  bubMatrixQR QhJranks(QhJs,fullrange(),bub::range(0,sizes));
  sotDEBUG(25) << "beforeQhJs = " << (MATLAB)QhJranks << std::endl;
  lastRotation.multiplyRightTranspose(QhJranks);

  sotDEBUG(25) << "hessenbergQhJsU = " << (MATLAB)QhJranksU << std::endl;
  sotDEBUG(25) << "order = " << (MATLAB)orderS << std::endl;
  sotDEBUG(15) << "/* Regularize the Hessenberg matrix. */"<<std::endl;
  std::vector<bool> notFullRank(sizes,true);
  sotRotationComposed Qhdebug;
  for( unsigned int i=0;i<ranks;++i )
    {
      const unsigned int col = orderS[i];
      if((i+1<freeRank)&&(fabs(QhJsU(rankh+i+1,col))>THRESHOLD_ZERO))
        {
          sotDEBUG(45) << "Regularize diag-up on " << col << ". " << std::endl;
          sotRotationSimpleGiven Gr( QhJsU,rankh+i,rankh+i+1,col ); Gr.inverse();
          Gr.multiplyRightTranspose(QhJsU);
          Gr.multiplyRightTranspose(QhJs);
          Qh.pushBack(Gr);
          Qhdebug.pushBack(Gr);
        }
      notFullRank[col]=false;
    }

  sotDEBUG(25) << "Qhmodif = " << MATLAB(Qhdebug,nJ) << std::endl;
  if( freeRank>0 )
    {
      sotDEBUG(25) << "regQhJsU = " << (MATLAB)QhJranksU << std::endl;
      sotDEBUG(25) << "regQhJs = " << (MATLAB)QhJranks << std::endl;
      sotDEBUG(15) << "/* Check for the rank update of Rs. */"<<std::endl;
      bool rankOneUpdateDone = false;
      for( unsigned int i=0;i<sizes;++i )
        {
          bubMatrixQR Rsno(QhJsU,bub::range(rankh,rankh+ranks),bub::range(0,sizes));
          if( notFullRank[i] )
            {
              /* Check that last-row value is still null. */
              if( (!rankOneUpdateDone)&&(rankh+ranks<nJ)
                  &&(fabs(QhJsU(rankh+ranks,i))>THRESHOLD_ZERO) )
                {
                  sotDEBUG(5) << "Rank inc at " << i << "." << std::endl;
                  ranks++; orderS+=i; rankOneUpdateDone=true;;
                  constraintSactive[i]->rankIncreaser = true;
                }
              else if( orderS.size()>0)
                {
                  sotDEBUG(5)<<"ranks_after = " << ranks <<std::endl;
                  sotRotationComposed U;
                  U.regularizeRankDeficientTriangle(Rsno,orderS,i);
                  bubMatrixQR Jsup(QhJsU,rangeh(),bub::range(0,sizes));
                  U.multiplyLeft(Jsup);
                }
            }
        }
    }
  sotDEBUG(45) << "QhJs = " << (MATLAB)QhJs << std::endl;
  sotDEBUG(15) << "QhJsU = " << (MATLAB)QhJsU << std::endl;
  sotDEBUG(5) << "Rs = " << (MATLAB)accessRsConst() << std::endl;
}

/* ---------------------------------------------------------- */
//   void updateRankOneSlack( void )
//   {
//     sotDEBUG(15) << "/* Apply the last corrections of Qh. */"<<std::endl;
//     bubMatrixQR QhJranksU(QhJsU,fullrange(),bub::range(0,sizes));
//     lastRotation.multiplyRightTranspose(QhJranksU);
//     bubMatrixQR QhJranks(QhJs,fullrange(),bub::range(0,sizes));
//     lastRotation.multiplyRightTranspose(QhJranks);

//     sotDEBUG(15) << "/* Check for the full rank of Rs. */"<<std::endl;
//     sotDEBUG(25) << "beforeQhJsU = " << (MATLAB)bub::subrange(QhJsU,0,nJ,0,sizes) << std::endl;
//     /* rpc contains a map giving for each rank=1:sizes the ref of the column of that size. */
//     std::vector< std::list<unsigned int> > rankPerColumn(sizes);
//     sotDEBUG(15) << "/* Classify the columns per rank in <rankPerColumn>. */"<<std::endl;
//     for( unsigned int col=0;col<sizes;++col )
//       {
//         for( unsigned int row=nJ-1;row>=rankh;--row )
//           {
//             sotDEBUG(50) << row << " x " << col << " = " << fabs(QhJsU(row,col)) << std::endl;
//             /* Search for the first non-zero. */
//             if( fabs(QhJsU(row,col))>THRESHOLD_ZERO )
//               { rankPerColumn[row-rankh].push_back(col); break; }
//           }
//       }
//     sotDEBUG(15) << "/* Select the columns to shape a triangle. */"<<std::endl;
//     bub::unbounded_array<std::size_t> columnOrder(sizes); ranks=0;
//     std::list<unsigned int> needRightRegularization;
//     bool needLeftRegularization = false;
//     for( unsigned int i=0;i<sizes;++i )
//       {
//         sotDEBUG(45) << i << std::endl;
//         std::list<unsigned int> & rpci = rankPerColumn[i];
//         if( rpci.size()>0 )
//           {
//             columnOrder[i] = rpci.front(); ranks++;
//             rpci.pop_front();
//           }
//         else
//           {
//             sotDEBUG(5) << "Need left regularization for col " << i << std::endl;
//             needLeftRegularization = true;
//           }
//         if( rpci.size()>0 )
//           needRightRegularization.insert(needRightRegularization.end(), rpci.begin(), rpci.end());
//       }
//     orderS = bubOrder(ranks,columnOrder);
//     sotDEBUG(5) << "unrQhJsU = "<< (MATLAB)accessQRs() << std::endl;
//     if( needLeftRegularization )
//       {
//         sotDEBUG(15) << "/* Regularize the Hessenberg matrix. */"<<std::endl;
//         for( unsigned int i=0;i<ranks;++i )
//           {
//             sotRotationSimpleGiven Gr( QhJsU,rankh+i,rankh+i+1,columnOrder[i] );  Gr.inverse();
//             Gr.multiplyRightTranspose(QhJsU);
//             Gr.multiplyRightTranspose(QhJs);
//             Qh.pushBack(Gr);
//           }
//       }
//     sotDEBUG(5) << "rlQhJsU = "<< (MATLAB)QhJsU << std::endl;
//     sotDEBUG(15) << "/* Regularize the rank-def columns. */"<<std::endl;
//     bubMatrixQR QhJsUdown( QhJsU,bub::range(rankh,rankh+ranks),bub::range(0,sizes));
//     for( std::list<unsigned int>::iterator iter=needRightRegularization.begin();
//          iter!=needRightRegularization.end();++iter )
//       {
//         sotDEBUG(25) << "Regularize right " << *iter << std::endl;
//         sotRotationComposed Gr; Gr.regularizeRankDeficientTriangle(QhJsUdown,orderS,*iter);
//       }
//     sotDEBUG(45) << "QhJs = " << (MATLAB)QhJs << std::endl;
//     sotDEBUG(15) << "QhJsU = " << (MATLAB)QhJsU << std::endl;
//     sotDEBUG(5) << "Rs = " << (MATLAB)accessRsConst() << std::endl;
//   }

/* ---------------------------------------------------------- */
/* Compute b = Ja' ( ea - Ja u0), with Ja,ea the jacobian and reference
 * of the active slack constraints.
 * <gradient> must be of size <nJ> (no resize).
 */
void SolverHierarchicalInequalities::
computeGradient( bubVector& gradientWide )
{
  //     bubVector gse(ese.size()); gse=ese;
  //     bub::axpy_prod( Jse,-u0,gse,false ); /* gse = ese - Jse u0 */
  //     bub::axpy_prod( gse,Jse,gradientWide,true ); /* g = J' (ese - Jse u0 ) */
  gradientWide .clear();
  for( ConstraintRefList::const_iterator iter=constraintSactive.begin();
       iter!=constraintSactive.end();++iter )
    {
      const ConstraintMem & ci = **iter;
      const double ciei = (ci.activeSide==ConstraintMem::BOUND_INF)?ci.eiInf:ci.eiSup;
      double epJu = ciei - bub::inner_prod(ci.Ji,u0);
      gradientWide += epJu*ci.Ji;
      sotDEBUG(55) << "Gradient with " << ci << " -> g = " << (MATLAB)gradientWide << std::endl;
    }
}

/* Compute the primal solution of the (equality-only) QP problem,
 * using the Null-Space method:
 *   du = Nh Ms Rs^-T Rs^-1 Ms' Nh' Js' ( es - Js u0 )
 */
void SolverHierarchicalInequalities::
computePrimal( void )
{
  if( (0==freeRank)||(0==ranks) )
    {
      sotDEBUG(15) <<"Ranks null, du is 0." << std::endl;
      du.resize(nJ,false); du.clear();
      return;
    }

  const bubMatrixQROrderedTriConst Rei = accessRsConst();
  sotDEBUG(15) << "Rei = " << (MATLAB)Rei << std::endl;
  sotDEBUG(15) << "/* du <- Js' ( es - Js u0 ) */"<< std::endl;
  du.resize(nJ,false); computeGradient(du);
  sotDEBUG(15) << "Jtei = " << (MATLAB)du << std::endl;
  sotDEBUG(15) << "/* Mbei <- Ms' Nh' Js' ( es - Js u0 ) */"<< std::endl;
  bub::vector_range<bubVector> Mbei
    =    Qh.multiplyRangeLeft(du,rankh,freeRank-ranks);
  sotDEBUG(15) << "--"<< std::endl;
  sotDEBUG(45) << "Qbei = " << (MATLAB)du << std::endl;
  sotDEBUG(15) << "Mbei = " << (MATLAB)Mbei << std::endl;
  sotDEBUG(15) << "/* Mbei <- R-ei Ms' Jt' et */"<< std::endl;
  bub::lu_substitute(Rei,Mbei);
  sotDEBUG(15) << "RMbei = " << (MATLAB)Mbei << std::endl;
  sotDEBUG(15) << "/* Mbei <- R'-1 R-1 Ms' Jt' et */"<< std::endl;
  bub::lu_substitute(Mbei,(const bubMatrix &)Rei);
  sotDEBUG(15) << "RRMbei = " << (MATLAB)Mbei << std::endl;
  sotDEBUG(15) << "/* du <- Qh * [ 0 ; Qs * [ Mbei; 0 ] ] */"<< std::endl;
  Qh.multiplyRangeRight(du,rankh,(freeRank-ranks));
  sotDEBUG(5) << "duei = " << (MATLAB)du << std::endl;
}
/* ---------------------------------------------------------- */
/* Compute the slack w = es-Js(u0+du). Since esi<Jsi.u, the slack
 * w = esi-Jsi.u should be negative. Since Jss.u<ess, the slack
 * w = Jss.u-ess should also be negative. */
void SolverHierarchicalInequalities::
computeSlack( void )
{
  slackInf.resize(constraintS.size(),false); slackInf.clear();
  slackSup.resize(constraintS.size(),false); slackSup.clear();
  bubVector updu(nJ); updu=u0; // updu+=du;
  sotDEBUG(5) << "updu = " << (MATLAB)updu << std::endl;
  for( ConstraintList::const_iterator iter=constraintS.begin();
       iter!=constraintS.end();++iter )
    {
      const ConstraintMem & cs = *iter;
      if(! cs.equality)
        {
          const double Ju = bub::inner_prod(cs.Ji,updu);
          sotDEBUG(55) << "J = " << (MATLAB)cs.Ji << std::endl;
          sotDEBUG(55) << "Ju = " << Ju << std::endl;

          if( cs.boundSide&ConstraintMem::BOUND_INF )
            { slackInf(cs.constraintRow) = cs.eiInf - Ju; }
          else { slackInf(cs.constraintRow) = 0; }
          if( cs.boundSide&ConstraintMem::BOUND_SUP )
            { slackSup(cs.constraintRow) = Ju - cs.eiSup; }
          else { slackSup(cs.constraintRow) = 0; }
        }
    }
  sotDEBUG(5) << "slackInf = " << (MATLAB)slackInf << std::endl;
  sotDEBUG(5) << "slackSup = " << (MATLAB)slackSup << std::endl;
}
void SolverHierarchicalInequalities::
computeLagrangian( void )
{
  if(rankh==0) return;
  lagrangian.resize(rankh,false);    lagrangian.clear();

  /* bs <- Js' (es - Js.u) */
  bubVector bs(nJ);    computeGradient(bs);
  sotDEBUG(15) << "bs = " << (MATLAB)bs << std::endl;
  /* bs <- Qh' Js' (es - Js.u) */
  /* ms <- Mh' Js' (es - Js.u) */
  bub::vector_range<bubVector> Mbs = Qh.multiplyRangeLeft(bs,0,freeRank);
  sotDEBUG(45) << "Qbs = " << (MATLAB)bs << std::endl;
  sotDEBUG(45) << "Mbs = " << (MATLAB)Mbs << std::endl;
  Mbs *= -1;
  /* Mbs <- R0^-1 Mh' Js' (-es + Js.u) */
  sotDEBUG(45) << "Rh = " << (MATLAB)accessRhConst() << std::endl;
  bub::lu_substitute(accessRhConst(),Mbs);
  sotDEBUG(45) << "RMbs = " << (MATLAB)Mbs << std::endl;

  lagrangian.resize(Mbs.size(),false);
  for( ConstraintList::iterator iter=constraintH.begin();
       iter!=constraintH.end();++iter )
    {
      ConstraintMem & cs = *iter;
      if( cs.active && cs.rankIncreaser )
        {
          const unsigned int range = cs.range;
          if( cs.activeSide&ConstraintMem::BOUND_INF )
            { lagrangian(range) = Mbs(range); }
          else if( cs.activeSide&ConstraintMem::BOUND_SUP )
            { lagrangian(range) = -Mbs(range); }
        }
    }
  //lagrangian.assign(Mbs);
  sotDEBUG(5) << "lagrangian = " << (MATLAB)lagrangian << std::endl;
}


/* ---------------------------------------------------------- */
bool SolverHierarchicalInequalities::
selecActivationHierarchic( double & tau )
{
  tau=1-THRESHOLD_ZERO; unsigned int constraintRef = 0;
  bool res = false;
  for( ConstraintList::iterator iter=constraintH.begin();
       iter!=constraintH.end();++iter,++constraintRef )
    {
      ConstraintMem & cs = *iter;
      if(!(cs.active||cs.notToBeConsidered))
        {
          cs.Ju = bub::inner_prod(cs.Ji,u0);
          cs.Jdu = bub::inner_prod(cs.Ji,du);
          /* Activation Inf. */
          if( cs.Jdu<-THRESHOLD_ZERO&&(cs.boundSide&ConstraintMem::BOUND_INF) )
            {
              const double taui = (cs.eiInf-cs.Ju)/cs.Jdu;
              if( taui<tau ) // If not: activate i.
                { tau=taui; HactivationRef=constraintRef;
                  HactivationSide=ConstraintMem::BOUND_INF; res=true; }
            }
          /* Activation Sup. */
          if( cs.Jdu>THRESHOLD_ZERO&&(cs.boundSide&ConstraintMem::BOUND_SUP) )
            {
              const double taui = (cs.eiSup-cs.Ju)/cs.Jdu;
              if( taui<tau ) // If not: activate i.
                { tau=taui; HactivationRef=constraintRef;
                  HactivationSide=ConstraintMem::BOUND_SUP; res=true; }
            }
        }
    }
  if(res) {sotDEBUG(1) << "Activation H <" << HactivationSide
                       << HactivationRef << ">" << std::endl;}
  return res;
}
bool SolverHierarchicalInequalities::
selecInactivationHierarchic( void )
{
  bool res=false; double HinactivationScore=-THRESHOLD_ZERO;
  for( ConstraintList::iterator iter=constraintH.begin();
       iter!=constraintH.end();++iter )
    {
      ConstraintMem & cs = *iter;
      if( cs.active && (!cs.equality) && cs.rankIncreaser )
        {
          const double &l=cs.lagrangian=lagrangian(cs.range);
          if(l<HinactivationScore) { HinactivationScore=l; res=true; HinactivationRef=cs.constraintRow; }
        }
      sotDEBUG(45) << cs << std::endl;
    }
  if(res) {sotDEBUG(1) << "Inactivation H <" << constraintH[HinactivationRef].activeSide
                       << HinactivationRef << ">" << constraintH[HinactivationRef] << std::endl;}
  return res;
}
/* The slack w = es-Js.u should be negative. */
bool SolverHierarchicalInequalities::
selecActivationSlack( void )
{
  unsigned int row = 0; SactivationScore=THRESHOLD_ZERO;
  for( ConstraintList::const_iterator iter=constraintS.begin();
       iter!=constraintS.end();++iter,++row )
    {
      // Slack should be negative
      if( (!iter->equality)&&(!iter->active) )
        {
         if( slackInf(row)>SactivationScore )
            { SactivationRef=row; SactivationScore=slackInf(row);
              SactivationSide=ConstraintMem::BOUND_INF; }
          if( slackSup(row)>SactivationScore )
            { SactivationRef=row; SactivationScore=slackSup(row);
              SactivationSide=ConstraintMem::BOUND_SUP; }
        }
    }
  if( SactivationScore>THRESHOLD_ZERO )
    {sotDEBUG(1) << "Activation S <" << SactivationSide << SactivationRef << ">" << std::endl;}
  return SactivationScore>THRESHOLD_ZERO;
}
/* The slack should be negative. If strickly negative: unactivate. */
bool SolverHierarchicalInequalities::
selecInactivationSlack( void )
{
  unsigned int row = 0; SinactivationScore=-THRESHOLD_ZERO;
  for( ConstraintList::const_iterator iter=constraintS.begin();
       iter!=constraintS.end();++iter,++row )
    {
      if( (!iter->equality)&&(iter->active) )
        {
          if( (iter->activeSide&ConstraintMem::BOUND_INF)&&(slackInf(row)<SinactivationScore) )
            { SinactivationRef=row; SinactivationScore=slackInf(row);}
          if( (iter->activeSide&ConstraintMem::BOUND_SUP)&&(slackSup(row)<SinactivationScore) )
            { SinactivationRef=row; SinactivationScore=slackSup(row);}
        }
    }
  if( SinactivationScore<-THRESHOLD_ZERO )
    {sotDEBUG(1) << "Inactivation S <" <<constraintS[SinactivationRef].activeSide
                 <<SinactivationRef << ">" << std::endl;}
  return (SinactivationScore<-THRESHOLD_ZERO); //TODO
}

/* ---------------------------------------------------------- */
void SolverHierarchicalInequalities::
pushBackSlackToHierarchy( void )
{
  if( freeRank>0 )
    {
      /* Pe is the range of the actual column of QR in the original
       * matrix: QR[:,i] == Jse'[:,pe(i)]. */
      //sotDEBUG(15) << "pe = " << orderSe << std::endl;
      /* Ps is the range of the columns of Rs in the QR matrix:
       * R[:,i]=QR[:,ps(i)]. */
      sotDEBUG(15) << "ps = " << (MATLAB)orderS << std::endl;
      sotDEBUG(15) << "ranks = " << ranks << std::endl;

      /* Push back S constraints. */
      for( ConstraintList::iterator iter=constraintS.begin();
           iter!=constraintS.end();++iter )
        {
          ConstraintMem& cs = *iter;
          if( cs.active )
            {
              const unsigned int nbcInJ = cs.constraintRow;
              if(! cs.equality )
                {
                  if(cs.activeSide==ConstraintMem::BOUND_INF)
                    cs.equality=(slackInf(nbcInJ)>THRESHOLD_ZERO);
                  if(cs.activeSide==ConstraintMem::BOUND_SUP)
                    cs.equality=(slackSup(nbcInJ)>THRESHOLD_ZERO);
                }
            }
          cs.constraintRow = constraintH.size();
          constraintH.push_back(cs);
          sotDEBUG(15) << "Add S cs: " <<cs << std::endl;
        }

      /* Push back Rs. */
      sotRotationComposed Qlast;
      for( unsigned int i=0;i<ranks;++i )
        {
          typedef bub::matrix_column<bubMatrixQRWide> bubQJsCol;
          bubQJsCol QJk(QhJs,orderS(i)); Qlast.multiplyLeft(QJk);
          sotDEBUG(45) << "QhJs_" << i << " = " << (MATLAB)QJk << std::endl;
          bub::vector_range<bubQJsCol> Ftdown(QJk,freerange());
          double beta;
          double normSignFt
            = sotRotationSimpleHouseholder::householderExtraction(Ftdown,beta,THRESHOLD_ZERO);
          sotDEBUG(45) << "Ft_" << i << " = " << (MATLAB)Ftdown << std::endl;
          sotDEBUG(45) << "b_" << i << " = " << beta << std::endl;
          if(fabs(beta)>THRESHOLD_ZERO)
            { Qlast.pushBack(sotRotationSimpleHouseholder(Ftdown,beta)); }

          bub::matrix_column<bubMatrix> Rhk(Rh,rankh);
          bub::project(Rhk,rangeh()).assign( bub::project(QJk,rangeh()) );
          bub::project(Rhk,freerange()).assign(bub::zero_vector<double>(freeRank));
          Rh(rankh,rankh) = normSignFt;
          sotDEBUG(45) << "Rh_" << i << " = " << (MATLAB)Rhk << std::endl;

          const unsigned int colInH = constraintSactive[orderS(i)]->constraintRow;
          constraintH[colInH].range=rankh;
          sotDEBUG(15) << "Constraint S " << i << " (rangeS=" << orderS(i)
                       << ") set in H " << rankh << ": " << constraintH[colInH]  << std::endl;
          rankh++; freeRank--;
        }
      Qh.pushBack(Qlast);

    }
  else
    {
      for( ConstraintList::iterator iter=constraintS.begin();
           iter!=constraintS.end();++iter )
        {
          ConstraintMem & cs = *iter;
          cs.rankIncreaser = false;
          cs.constraintRow = constraintH.size();
          constraintH.push_back(cs);
          sotDEBUG(15) << "Add rank-void cs: " << constraintH.back() << std::endl;
        }
    }

  if( rankh+freeRank!=nJ )
    {
      std::cerr << "Error: ( rankh+freeRank!=nJ ). " << std::endl;
      throw "Error: ( rankh+freeRank!=nJ ). ";
    }

  sotDEBUG(15) << "Qh = " << Qh << std::endl;
  sotDEBUG(15) << "Rh = " << (MATLAB)accessRhConst() << std::endl;
}


double SolverHierarchicalInequalities::THRESHOLD_ZERO = 1e-6;
