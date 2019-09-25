/*
 * Copyright 2019
 * Joseph Mirabel
 *
 * LAAS-CNRS
 *
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
//#define VP_DEBUG
//#define VP_DEBUG_MODE 45
#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <pinocchio/multibody/liegroup/liegroup.hpp>

#include <Eigen/LU>

#include <sot/core/debug.hh>
#include <sot/core/feature-transformation.hh>

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

//typedef pinocchio::CartesianProductOperation <
//        pinocchio::VectorSpaceOperationTpl<3, double>,
//        pinocchio::SpecialOrthogonalOperationTpl<3, double>
//        > LieGroup_t;
typedef pinocchio::SpecialEuclideanOperationTpl<3, double> LieGroup_t;

#include <sot/core/factory.hh>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureTransformation,"FeatureTransformation");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

static const MatrixHomogeneous Id (MatrixHomogeneous::Identity());

FeatureTransformation::
FeatureTransformation( const string& pointName )
  : FeatureAbstract( pointName )
    , oMja  ( NULL,"FeatureTransformation("+name+")::input(matrixHomo)::oMja" )
    , jaMfa ( NULL,"FeatureTransformation("+name+")::input(matrixHomo)::jaMfa")
    , oMjb  ( NULL,"FeatureTransformation("+name+")::input(matrixHomo)::oMjb" )
    , jbMfb ( NULL,"FeatureTransformation("+name+")::input(matrixHomo)::jbMfb")
    , jaJja ( NULL,"FeatureTransformation("+name+")::input(matrix)::jaJja")
    , jbJjb ( NULL,"FeatureTransformation("+name+")::input(matrix)::jbJjb")

    , faMfbDes ( NULL,"FeatureTransformation("+name+")::input(matrixHomo)::faMfbDes")
    , faMfbDesDot ( NULL,"FeatureTransformation("+name+")::input(vector)::faMfbDesDot")

    , q_oMfb (boost::bind (&FeatureTransformation::computeQoMfb, this, _1, _2),
        oMjb << jbMfb,
        "FeatureTransformation("+name+")::output(vector7)::q_oMfb")
    , q_oMfbDes (boost::bind (&FeatureTransformation::computeQoMfbDes, this, _1, _2),
        oMja << jaMfa << faMfbDes,
        "FeatureTransformation("+name+")::output(vector7)::q_oMfbDes")
{
  oMja.setConstant (Id);
  jaMfa.setConstant (Id);
  jbMfb.setConstant (Id);
  faMfbDes.setConstant (Id);
  faMfbDesDot.setConstant (Vector::Zero(6));

  jacobianSOUT.addDependencies(q_oMfbDes << q_oMfb
      << jaJja << jbJjb );

  errorSOUT.addDependencies( q_oMfbDes << q_oMfb );

  signalRegistration( oMja << jaMfa << oMjb << jbMfb << jaJja << jbJjb );
  signalRegistration (errordotSOUT << faMfbDes << faMfbDesDot);

  errordotSOUT.setFunction (boost::bind (&FeatureTransformation::computeErrorDot,
					 this, _1, _2));
  errordotSOUT.addDependencies (q_oMfbDes << q_oMfb << faMfbDesDot);

  // Commands
  //
  {
    using namespace dynamicgraph::command;
    addCommand("keep",
	       makeCommandVoid0(*this,&FeatureTransformation::servoCurrentPosition,
				docCommandVoid0("modify the desired position to servo at current pos.")));
  }
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

static inline void check (const FeatureTransformation& ft)
{
  assert (ft. oMja .isPlugged() );
  assert (ft. jaMfa.isPlugged() );
  assert (ft. oMjb .isPlugged() );
  assert (ft. jbMfb.isPlugged() );
  assert (ft. faMfbDes   .isPlugged() );
  assert (ft. faMfbDesDot.isPlugged() );
}

unsigned int& FeatureTransformation::
getDimension( unsigned int & dim, int time )
{
  sotDEBUG(25)<<"# In {"<<endl;

  const Flags &fl = selectionSIN.access(time);

  dim = 0;
  for( int i=0;i<6;++i ) if( fl(i) ) dim++;

  sotDEBUG(25)<<"# Out }"<<endl;
  return dim;
}

void toVector (const MatrixHomogeneous& M, Vector7& v)
{
  v.head<3>() = M.translation();
  QuaternionMap(v.tail<4>().data()) = M.linear();
}

Vector7 toVector (const MatrixHomogeneous& M)
{
  Vector7 ret;
  toVector (M, ret);
  return ret;
}

Matrix& FeatureTransformation::computeJacobian( Matrix& J,int time )
{
  check(*this);

  q_oMfb   .recompute(time);
  q_oMfbDes.recompute(time);

  const int & dim = dimensionSOUT(time);
  const Flags &fl = selectionSIN(time);

  const Matrix & _jbJjb = jbJjb (time);

  const MatrixHomogeneous& _jbMfb = (jbMfb.isPlugged() ? jbMfb.accessCopy() : Id);

  const Matrix::Index cJ = _jbJjb.cols();
  J.resize(dim,cJ) ;

  MatrixTwist X;
  Eigen::Matrix<double,6,6,Eigen::RowMajor> Jminus;

  buildFrom (_jbMfb.inverse(Eigen::Affine), X);
  LieGroup_t().dDifference<pinocchio::ARG1>(q_oMfbDes.accessCopy(), q_oMfb.accessCopy(), Jminus);
  
  // Contribution of b:
  // J = Jminus * X * jbJjb;
  unsigned int rJ = 0;
  for( unsigned int r=0;r<6;++r )
    if( fl(r) )
      J.row(rJ++) = (Jminus * X).row(r) * _jbJjb;

  if (jaJja.isPlugged()) {
    const Matrix & _jaJja = jaJja (time);
    const MatrixHomogeneous& _jaMfa = (jaMfa.isPlugged() ? jaMfa.accessCopy() : Id),
                             _faMfbDes = (faMfbDes.isPlugged() ? faMfbDes.accessCopy() : Id);

    LieGroup_t().dDifference<pinocchio::ARG0>(q_oMfbDes.accessCopy(), q_oMfb.accessCopy(), Jminus);
    buildFrom ((_jaMfa *_faMfbDes).inverse(Eigen::Affine), X);

    // J += (Jminus * X) * jaJja(time);
    rJ = 0;
    for( unsigned int r=0;r<6;++r )
      if( fl(r) )
        J.row(rJ++).noalias() += (Jminus * X).row(r) * _jaJja;
  }

  return J;
}

Vector7& FeatureTransformation::computeQoMfb (Vector7& res, int time)
{
  check(*this);

  toVector (oMjb(time) * jbMfb(time), res);
  return res;
}

Vector7& FeatureTransformation::computeQoMfbDes (Vector7& res, int time)
{
  check(*this);

  toVector (oMja(time) * jaMfa(time) * faMfbDes (time), res);
  return res;
}

Vector& FeatureTransformation::computeError( Vector& error,int time )
{
  check(*this);

  const Flags &fl = selectionSIN(time);

  Eigen::Matrix<double,6,1> v;
  LieGroup_t().difference (q_oMfbDes(time), q_oMfb(time), v);

  error.resize(dimensionSOUT(time)) ;
  unsigned int cursor = 0;
  for( unsigned int i=0;i<6;++i )
    if( fl(i) )
      error(cursor++) = v(i);

  return error ;
}

Vector& FeatureTransformation::computeErrorDot( Vector& errordot,int time )
{
  check(*this);

  errordot.resize(dimensionSOUT(time));
  const Flags &fl = selectionSIN(time);
  if (!faMfbDesDot.isPlugged()) {
    errordot.setZero();
    return errordot;
  }

  q_oMfb   .recompute(time);
  q_oMfbDes.recompute(time);

  const MatrixHomogeneous& _faMfbDes = (faMfbDes.isPlugged() ? faMfbDes.accessCopy() : Id);

  Eigen::Matrix<double,6,6,Eigen::RowMajor> Jminus;

  LieGroup_t().dDifference<pinocchio::ARG0>(q_oMfbDes.accessCopy(), q_oMfb.accessCopy(), Jminus);
  // Assume _faMfbDesDot is expressed in fa
  Jminus = Jminus * pinocchio::SE3(_faMfbDes.rotation(), _faMfbDes.translation()).toActionMatrixInverse();
  // Assume _faMfbDesDot is expressed in fb*
  // Jminus = Jminus
  unsigned int cursor = 0;
  for( unsigned int i=0;i<6;++i )
    if( fl(i) )
      errordot(cursor++) = Jminus.row(i) * faMfbDesDot.accessCopy();

  return errordot;
}

/* Modify the value of the reference (sdes) so that it corresponds
 * to the current position. The effect on the servo is to maintain the
 * current position and correct any drift. */
void FeatureTransformation::
servoCurrentPosition( void )
{
  check(*this);

  const MatrixHomogeneous& _oMja  = (oMja .isPlugged() ? oMja .accessCopy() : Id),
                           _jaMfa = (jaMfa.isPlugged() ? jaMfa.accessCopy() : Id),
                           _oMjb  =                      oMjb .accessCopy(),
                           _jbMfb = (jbMfb.isPlugged() ? jbMfb.accessCopy() : Id);
  faMfbDes = (_oMja * _jaMfa).inverse(Eigen::Affine) * _oMjb * _jbMfb;
}

static const char * featureNames  []
= { "X ",
    "Y ",
    "Z ",
    "RX",
    "RY",
    "RZ"  };
void FeatureTransformation::
display( std::ostream& os ) const
{
  os <<"Point6d <"<<name<<">: (" ;

  try{
    const Flags &fl = selectionSIN.accessCopy();
    bool first = true;
    for( int i=0;i<6;++i )
      if( fl(i) )
	{
	  if( first ) { first = false; } else { os << ","; }
	  os << featureNames[i];
	}
    os<<") ";
  }  catch(ExceptionAbstract e){ os<< " selectSIN not set."; }
}
