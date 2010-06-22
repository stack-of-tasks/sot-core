/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Gepetto, LAAS, 2009
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      solver-hierarchical-inequalities.h
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

#ifndef __SOT_sotSolverHierarchicalInequalities_HH__
#define __SOT_sotSolverHierarchicalInequalities_HH__


/* --- STD --- */
#include <iostream>
#include <sstream>
#include <list>
#include <vector>
#include <deque>
#include <typeinfo>
#include <algorithm>


#include <sot-core/rotation-simple.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#ifndef SOTSOTH_EXPORT 
#  if defined (WIN32)
#    if defined (sotSOTH_EXPORTS)
#      define SOTSOTH_EXPORT __declspec(dllexport)
#    else
#      define SOTSOTH_EXPORT __declspec(dllimport)
#    endif
#  else
#    define SOTSOTH_EXPORT
#  endif
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

class SOTSOTH_EXPORT ConstraintMem
{
public:
  enum BoundSideType
    {
      BOUND_VOID = 0,
      BOUND_INF = 1,
      BOUND_SUP = 2,
      BOUND_BOTH = 3
    };
  typedef std::vector<ConstraintMem::BoundSideType> BoundSideVector;

public:
  bool active; // If the constraint is active.
  bool equality; // If the constraint cannot be inactivate.
  bool notToBeConsidered; // (during warm start) if constraint is unactive and not to be activated.

  bubVector Ji; // The row of the jacobian.
  double eiInf,eiSup; // The row of the error reference.
  BoundSideType boundSide,activeSide;

  bool rankIncreaser; // If the constraint is non redundant wrt to R. */
  unsigned int constraintRow; // Num of the row where it is located in Ji.
  unsigned int range; // Num of the col where it has been added in F. Used in H, not in S.
  double lagrangian; // last value of the lagrangian -> negatif: should be unactivated.
  double Ju,Jdu;  // effet of the control law in the task space.

public:
  ConstraintMem( void )
    :active(false),equality(0),notToBeConsidered(0)
    ,Ji(0),eiInf(0),eiSup(0),boundSide(BOUND_VOID),activeSide(BOUND_VOID)
    ,rankIncreaser(false),constraintRow(0),range(0),lagrangian(0)
    ,Ju(0),Jdu(0) {}

  ConstraintMem( const ConstraintMem& clone );
  SOTSOTH_EXPORT friend std::ostream& operator<<( std::ostream& os,const BoundSideType& bs );
  SOTSOTH_EXPORT friend std::ostream & operator<< (std::ostream& os,const ConstraintMem &c );
};


  /* ---------------------------------------------------------- */
  /* ---------------------------------------------------------- */
  /* ---------------------------------------------------------- */

class SOTSOTH_EXPORT ConstraintRef
{
public:
  unsigned int id;
  ConstraintMem::BoundSideType side;
  ConstraintRef( const unsigned int id_ = 0,
                    const ConstraintMem::BoundSideType side_
                    =ConstraintMem::BOUND_VOID )
    :id(id_),side(side_) {}
  SOTSOTH_EXPORT friend std::ostream & operator<< (std::ostream & os,
                                    const ConstraintRef & cr )
  { return os << cr.side << cr.id; }
};


  /* ---------------------------------------------------------- */
  /* ---------------------------------------------------------- */
  /* ---------------------------------------------------------- */

class SOTSOTH_EXPORT SolverHierarchicalInequalities
{
public: // protected:
  typedef bub::matrix<double,bub::column_major> bubMatrixQRWide;
  typedef bub::matrix_range<bubMatrixQRWide> bubMatrixQR;
  typedef bub::matrix_range<const bubMatrixQRWide> bubMatrixQRConst;
  typedef bub::triangular_adaptor<bubMatrixQR,bub::upper> bubMatrixQRTri;
  typedef bub::triangular_adaptor<bubMatrixQRConst,bub::upper> bubMatrixQRTriConst;
  typedef bub::indirect_array<> bubOrder;

  typedef bub::matrix_indirect<bubMatrix> bubMatrixOrdered;
  typedef bub::matrix_indirect<bubMatrixQRWide> bubMatrixQRWideOrdered;
  typedef bub::matrix_indirect<bubMatrixQR> bubMatrixQROrdered;
  typedef bub::triangular_adaptor<bubMatrixQROrdered,bub::upper> bubMatrixQROrderedTri;

  typedef bub::matrix_indirect<const bubMatrixQRWide> bubMatrixQRWideOrderedConst;
  typedef bub::matrix_indirect<bubMatrixQRConst> bubMatrixQROrderedConst;
  typedef bub::triangular_adaptor<bubMatrixQROrderedConst,bub::upper> bubMatrixQROrderedTriConst;

  typedef std::vector<ConstraintMem> ConstraintList;
  typedef std::vector<ConstraintMem*> ConstraintRefList;
  static double THRESHOLD_ZERO;

public: // protected:
  unsigned int nJ;
  bubVector u0;

  /* --- H data --- */
  sotRotationComposedInExtenso& Qh;
  bubMatrix& Rh;
  unsigned int rankh,freeRank;
  ConstraintList& constraintH;
  bool Hactivation,Hinactivation;
  unsigned int HactivationRef,HinactivationRef;
  ConstraintMem::BoundSideType HactivationSide;

  /* --- S data --- */
  bubMatrixQRWide QhJsU,QhJs;
  unsigned int ranks,sizes;
  bubOrder orderS;
  ConstraintList constraintS;
  ConstraintRefList constraintSactive;
  bool Sactivation,Sinactivation;
  unsigned int SactivationRef,SinactivationRef;
  ConstraintMem::BoundSideType SactivationSide;

  /* --- Next step --- */
  double SactivationScore,SinactivationScore,HactivationScore,HinactivationScore;

  /* --- divers --- */
  bubVector du,slackInf,slackSup,lagrangian;
  sotRotationComposed lastRotation;
  int freeRankChange;

  /* --- Warm start knowledge --- */
  enum ActivationTodoType { TODO_NOTHING, TODO_ACTIVATE, TODO_INACTIVATE };
  std::vector<bool> initialActiveH;
  ConstraintMem::BoundSideVector initialSideH;
  bubVector du0;
  std::vector<ConstraintRef> slackActiveSet;
  std::vector<ConstraintRef> toActivate,toInactivate;
  bool warmStartReady;

  /* ---------------------------------------------------------- */
public:
  SolverHierarchicalInequalities( unsigned int _nJ,
                                     sotRotationComposedInExtenso& _Qh,
                                     bubMatrix &_Rh,
                                     ConstraintList &_cH )
    :nJ(_nJ),u0(_nJ),Qh(_Qh),Rh(_Rh),rankh(0),freeRank(_nJ),constraintH(_cH)
    ,ranks(0),warmStartReady(false)
  {
    u0*=0;
  }
 private:
  SolverHierarchicalInequalities( const SolverHierarchicalInequalities& clone )
    : nJ(clone.nJ),Qh(clone.Qh),Rh(clone.Rh),constraintH(clone.constraintH){ /* forbiden */ }

 public:
  /* ---------------------------------------------------------- */
  void initConstraintSize( const unsigned int size );
  void setInitialCondition( const bubVector& _u0,
                            const unsigned int _rankh );
  void setInitialConditionVoid( void );
  void setNbDof( const unsigned int nJ );
  unsigned int getNbDof( void ) { return nJ; }
  /* ---------------------------------------------------------- */
  void recordInitialConditions( void );
  void computeDifferentialCondition( void );

  const std::vector<ConstraintRef> & getToActivateList( void ) const
  { return toActivate; }
  const std::vector<ConstraintRef> & getToInactivateList( void ) const
  { return toInactivate; }
  const bubVector & getDifferentialU0( void ) const
  {    return du0;  }
  const std::vector<ConstraintRef>& getSlackActiveSet( void ) const
  { return slackActiveSet; }
  void printDifferentialCondition( std::ostream & os ) const;

  /* ---------------------------------------------------------- */
  bub::range fullrange( void ) const { return bub::range(0,nJ); }
  bub::range rangeh( void ) const { return bub::range(0,rankh); }
  bub::range rangehs( void ) const { return bub::range(0,rankh+ranks); }
  bub::range freerange( void ) const { return bub::range(rankh,nJ); }
  bub::range freeranges( void ) const { return bub::range(rankh,rankh+ranks); }

/*   bubMatrixQROrdered accessQRs( void ); */
/*   bubMatrixQROrderedConst accessQRs( void ) const; */
/*   bubMatrixQROrderedTri accessRs( void ); */
  bubMatrixQROrderedTriConst accessRsConst( void ) const;
  bub::triangular_adaptor<bub::matrix_range< const bubMatrix >,bub::upper>
    accessRhConst( void ) const;
  bub::triangular_adaptor<bub::matrix_range< bubMatrix >,bub::upper>
    accessRh( void );
  template< typename bubTemplateMatrix >
  unsigned int rankDetermination( const bubTemplateMatrix& A,
                                  const double threshold = THRESHOLD_ZERO );
  /* ---------------------------------------------------------- */
  static void displayConstraint( ConstraintList & cs );
  void printDebug( void );

  /* ---------------------------------------------------------- */
  void warmStart( void );
  void applyFreeSpaceMotion( const bubVector& _du );
  void forceUpdateHierachic( ConstraintRefList& toUpdate,
                             const ConstraintMem::BoundSideVector& boundSide );
  void forceDowndateHierachic( ConstraintRefList& toDowndate );

  /* ---------------------------------------------------------- */
  void solve( const bubMatrix& Jse, const bubVector& ese,
              const bubMatrix& Jsi, const bubVector& esiInf, const bubVector& esiSup,
              const std::vector<ConstraintMem::BoundSideType> esiBoundSide,
              bool pushBackAtTheEnd = true );
  void solve( const bubMatrix& Jse, const bubVector& ese,
              const bubMatrix& Jsi, const bubVector& esiInf, const bubVector& esiSup,
              const ConstraintMem::BoundSideVector & esiBoundSide,
              const std::vector<ConstraintRef> & slackActiveWarmStart,
              bool pushBackAtTheEnd = true );

  /* ---------------------------------------------------------- */
  void initializeConstraintMemory( const bubMatrix& Jse, const bubVector& ese,
                                   const bubMatrix& Jsi, const bubVector& esiInf,
                                   const bubVector& esiSup,
                                   const ConstraintMem::BoundSideVector& esiBoundSide,
                                   const std::vector<ConstraintRef>& warmStartSide );
  void initializeDecompositionSlack(void);

  /* ---------------------------------------------------------- */
  void updateConstraintHierarchic( const unsigned int constraintId,
                                   const ConstraintMem::BoundSideType side );
  void downdateConstraintHierarchic( const unsigned int kdown );
  void updateRankOneDowndate( void );
  void updateRankOneUpdate( void );

  /* ---------------------------------------------------------- */
  void updateConstraintSlack( const unsigned int kup,
                              const ConstraintMem::BoundSideType activeSide );
  void regularizeQhJs( void );
  void regularizeQhJsU( void );
  void downdateConstraintSlack( const unsigned int kdown );

  /* ---------------------------------------------------------- */
  void computeGradient( bubVector& gradientWide );
  void computePrimal( void );
  void computeSlack( void );
  void computeLagrangian( void );

  bool selecActivationHierarchic( double & tau );
  bool selecInactivationHierarchic( void );
  bool selecActivationSlack( void );
  bool selecInactivationSlack( void );
  void pushBackSlackToHierarchy( void );

};


} // namespace sot



#endif // #ifdef __SOT_sotSolverHierarchicalInequalities_HH__

