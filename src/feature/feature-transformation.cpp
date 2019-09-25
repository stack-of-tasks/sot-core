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

    , faMfb (boost::bind (&FeatureTransformation::computefaMfb, this, _1, _2),
        oMja << jaMfa << oMjb << jbMfb,
        "FeatureTransformation("+name+")::output(vector7)::faMfbDesDot")
    , faMfbDes_q (boost::bind (&FeatureTransformation::computefaMfbDes_q, this, _1, _2),
        faMfbDes,
        "FeatureTransformation("+name+")::output(vector7)::faMfbDes_q")
{
  jacobianSOUT.addDependencies( faMfb << faMfbDes_q
      << jaJja << jbJjb );

  errorSOUT.addDependencies( faMfb << faMfbDes_q );

  signalRegistration( oMja << jaMfa << oMjb << jbMfb << jaJja << jbJjb );
  signalRegistration (errordotSOUT << faMfbDes << faMfbDesDot);

  errordotSOUT.setFunction (boost::bind (&FeatureTransformation::computeErrorDot,
					 this, _1, _2));
  errordotSOUT.addDependencies (faMfbDesDot << faMfb << faMfbDes_q);

  // Commands
  //
  {
    using namespace dynamicgraph::command;
    addCommand("keep",
	       makeCommandVoid0(*this,&FeatureTransformation::servoCurrentPosition,
				docCommandVoid0("modify the desired position to servo at current pos.")));
  }
}

/* TODO Add this dependency in constructor.
void FeaturePoint6d::
addDependenciesFromReference( void )
{
  assert( isReferenceSet() );
  errorSOUT.addDependency( getReference()->positionSIN );
  jacobianSOUT.addDependency( getReference()->positionSIN );
}

void FeaturePoint6d::
removeDependenciesFromReference( void )
{
  assert( isReferenceSet() );
  errorSOUT.removeDependency( getReference()->positionSIN );
  jacobianSOUT.removeDependency( getReference()->positionSIN );
}
*/

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

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

static const MatrixHomogeneous Id (MatrixHomogeneous::Identity());

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

void fromVector (const Vector7& v, MatrixHomogeneous& M)
{
  M.translation() = v.head<3>();
  M.linear() = Eigen::Map<const Quaternion>(v.tail<4>().data()).toRotationMatrix();
}

Matrix& FeatureTransformation::computeJacobian( Matrix& J,int time )
{
  const int & dim = dimensionSOUT(time);
  const Flags &fl = selectionSIN(time);

  const Matrix & _jbJjb = jbJjb (time);
  //const Vector7& _faMfb    = faMfb      (time),
                 //_faMfbDes = faMfbDes_q (time);

  const MatrixHomogeneous& _oMja  = (oMja .isPlugged() ? oMja (time) : Id),
                           _jaMfa = (jaMfa.isPlugged() ? jaMfa(time) : Id),
                           _oMjb  =                      oMjb (time),
                           _jbMfb = (jbMfb.isPlugged() ? jbMfb(time) : Id),
                           _faMfbDes = (faMfbDes.isPlugged() ? faMfbDes(time) : Id);

  const Matrix::Index cJ = _jbJjb.cols();
  J.resize(dim,cJ) ;
  Matrix tmp (6,cJ);

  MatrixTwist X;
  Eigen::Matrix<double,6,6,Eigen::RowMajor> Jminus;

  buildFrom (_jbMfb.inverse(Eigen::Affine), X);
  LieGroup_t().dDifference<pinocchio::ARG1>(
      toVector(_oMja * _jaMfa * _faMfbDes),
      toVector(_oMjb * _jbMfb),
      Jminus);
  
  tmp.noalias() = (Jminus * X) * _jbJjb;

  if (jaJja.isPlugged()) {
    LieGroup_t().dDifference<pinocchio::ARG0>(
      toVector(_oMja * _jaMfa * _faMfbDes),
      toVector(_oMjb * _jbMfb),
      Jminus);
    buildFrom ((_jaMfa *_faMfbDes).inverse(Eigen::Affine), X);

    tmp.noalias() += (Jminus * X) * jaJja(time);
  }

  /* Select the active line of Jq. */
  unsigned int rJ = 0;
  for( unsigned int r=0;r<6;++r )
    if( fl(r) )
      J.row(rJ++) = tmp.row(r);

  return J;
}

Vector7& FeatureTransformation::computefaMfb (Vector7& res, int time)
{
  const MatrixHomogeneous& _oMja  = (oMja .isPlugged() ? oMja (time) : Id),
                           _jaMfa = (jaMfa.isPlugged() ? jaMfa(time) : Id),
                           _oMjb  =                      oMjb (time),
                           _jbMfb = (jbMfb.isPlugged() ? jbMfb(time) : Id);

  MatrixHomogeneous _faMfb = (_oMja * _jaMfa).inverse(Eigen::Affine) * _oMjb * _jbMfb;

  toVector (_faMfb, res);
  return res;
}

Vector7& FeatureTransformation::computefaMfbDes_q (Vector7& res, int time)
{
  if (faMfbDes.isPlugged()) {
    const MatrixHomogeneous& _faMfbDes = faMfbDes(time);
    toVector (_faMfbDes, res);
  } else {
    res.head<6>().setZero();
    res[6] = 1.;
  }
  return res;
}

Vector& FeatureTransformation::computeError( Vector& error,int time )
{
  /*
  const Vector7& _faMfb    = faMfb      (time),
                 _faMfbDes = faMfbDes_q (time);

  const Flags &fl = selectionSIN(time);

  Eigen::Matrix<double,6,1> v;
  LieGroup_t().difference (_faMfbDes, _faMfb, v); // _faMfb - _faMfbDes
  */
  const MatrixHomogeneous& _oMja  = (oMja .isPlugged() ? oMja (time) : Id),
                           _jaMfa = (jaMfa.isPlugged() ? jaMfa(time) : Id),
                           _oMjb  =                      oMjb (time),
                           _jbMfb = (jbMfb.isPlugged() ? jbMfb(time) : Id),
                           _faMfbDes = (faMfbDes.isPlugged() ? faMfbDes(time) : Id);

  const Flags &fl = selectionSIN(time);

  Eigen::Matrix<double,6,1> v;
  LieGroup_t().difference (
      toVector(_oMja * _jaMfa * _faMfbDes),
      toVector(_oMjb * _jbMfb),
      v);

  error.resize(dimensionSOUT(time)) ;
  unsigned int cursor = 0;
  for( unsigned int i=0;i<6;++i )
    if( fl(i) )
      error(cursor++) = v(i);

  return error ;
}

Vector& FeatureTransformation::computeErrorDot( Vector& errordot,int time )
{
  errordot.resize(dimensionSOUT(time));
  const Flags &fl = selectionSIN(time);
  if (!faMfbDesDot.isPlugged()) {
    errordot.setZero();
    return errordot;
  }

  const Vector& _faMfbDesDot = faMfbDesDot(time);
  const MatrixHomogeneous& _oMja  = (oMja .isPlugged() ? oMja (time) : Id),
                           _jaMfa = (jaMfa.isPlugged() ? jaMfa(time) : Id),
                           _oMjb  =                      oMjb (time),
                           _jbMfb = (jbMfb.isPlugged() ? jbMfb(time) : Id),
                           _faMfbDes = (faMfbDes.isPlugged() ? faMfbDes(time) : Id);

  Eigen::Matrix<double,6,6,Eigen::RowMajor> Jminus;

  LieGroup_t().dDifference<pinocchio::ARG0>(
      toVector(_oMja * _jaMfa * _faMfbDes),
      toVector(_oMjb * _jbMfb),
      Jminus);
  // Assume _faMfbDesDot is expressed in fa
  Jminus = Jminus * pinocchio::SE3(_faMfbDes.rotation(), _faMfbDes.translation()).toActionMatrixInverse();
  // Assume _faMfbDesDot is expressed in fb*
  // Jminus = Jminus
  unsigned int cursor = 0;
  for( unsigned int i=0;i<6;++i )
    if( fl(i) )
      errordot(cursor++) = Jminus.row(i) * _faMfbDesDot;

  return errordot;
}

/* Modify the value of the reference (sdes) so that it corresponds
 * to the current position. The effect on the servo is to maintain the
 * current position and correct any drift. */
void FeatureTransformation::
servoCurrentPosition( void )
{
  MatrixHomogeneous M;
  fromVector (faMfb.accessCopy(), M);
  faMfbDes = M;
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
