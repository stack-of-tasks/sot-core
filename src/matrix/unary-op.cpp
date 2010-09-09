/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      unary-op.h
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

#include <sot-core/unary-op.h>

#include <sot-core/factory.h>

#include <sot-core/matrix-homogeneous.h>
#include <sot-core/matrix-twist.h>
#include <sot-core/vector-utheta.h>
#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot-core/vector-quaternion.h>


using namespace sot;
using namespace dynamicgraph;


#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(sotClassType,sotType,index,className,CMDLINE,CMDHELP)    \
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeInName( void ) { return std::string(#sotType); }                               \
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeOutName( void ) { return std::string(#sotType); }                              \
  template<>                                                                            \
  const std::string sotClassType::CLASS_NAME                                            \
     = std::string(className);                                                          \
  template<>                                                                            \
  void sotClassType::commandLine( const std::string& cmdLine,                           \
                                  std::istringstream& cmdArgs,                          \
 			          std::ostream& os )                                    \
  {                                                                                     \
    if( cmdLine=="help" ) { os << CMDHELP << std::endl; }                               \
    CMDLINE                                                                             \
      else { Entity::commandLine(cmdLine,cmdArgs,os); }                              \
  }                                                                                     \
   extern "C" {                                                                         \
    Entity *regFunction##_##index( const std::string& objname )                      \
    {                                                                                   \
      return new sotClassType( objname );                                               \
    }                                                                                   \
  EntityRegisterer regObj##_##index( std::string(className),                         \
					  &regFunction##_##index );                     \
  }

using namespace ml;

namespace sot {

template< typename matrixgen >
struct Inverser
{
  void operator()( const matrixgen& m,matrixgen& res ) const { m.inverse(res); }
};

typedef UnaryOp<Matrix,Matrix,Inverser<Matrix> > invMat;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(invMat,matrix,inv_mat,"Inverse<matrix>", ,"");

typedef UnaryOp<MatrixHomogeneous,MatrixHomogeneous,Inverser<MatrixHomogeneous> > invMatHomo;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(invMatHomo,matrixHomo,inv_mathomo,"Inverse<matrixhomo>", ,"");

typedef UnaryOp<MatrixTwist,MatrixTwist,Inverser<MatrixTwist> > invMatTwist;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(invMatTwist,matrixTwist,inv_mattwist,"Inverse<matrixtwist>", ,"");


struct InverserRot
{
  void operator()( const MatrixRotation& m,MatrixRotation& res ) const { m.transpose(res); }
};

typedef UnaryOp<MatrixRotation,MatrixRotation,InverserRot > invMatRot;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(invMatRot,matrixRotation,inv_matrot,"Inverse<matrixrotation>", ,"");

struct InverserQuat
{
  void operator()( const VectorQuaternion& q, VectorQuaternion& res ) const
  {
    q.conjugate(res);
  }
};

typedef UnaryOp<VectorQuaternion,VectorQuaternion,InverserQuat> invQuat;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(invQuat,q,qstar,"Inverse<unitquat>", ,"");

struct VectorSelector
{
public:
  unsigned int imin,imax;
  void operator()( const Vector& m,Vector& res ) const 
  {
    if( (imax<imin)||(m.size()<imax) ) return;
    res.resize( imax-imin );
    for( unsigned int i=imin;i<imax;++i ) res(i-imin)=m(i);
  }
};

typedef UnaryOp<Vector,Vector,VectorSelector > selcVec;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(selcVec,vector,sec_vec,"Selec<vector>",   
  else if( cmdLine == "selec" ) 
    {  cmdArgs >> op.imin >> op.imax; },
  "Selec<vector>\n  - selec min max\t\t");

struct MatrixSelector
{
public:
  unsigned int imin,imax;
  unsigned int jmin,jmax;
  void operator()( const Matrix& m,Matrix& res ) const 
  {
    if( (imax<imin)||(m.nbRows()<imax) ) return;
    if( (jmax<jmin)||(m.nbCols()<jmax) ) return;
    res.resize( imax-imin,jmax-jmin );
    for( unsigned int i=imin;i<imax;++i ) 
      for( unsigned int j=jmin;j<jmax;++j ) 
	res(i-imin,j-jmin)=m(i,j);
  }
};

typedef UnaryOp<Matrix,Matrix,MatrixSelector > selcMat;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(selcMat,matrix,sec_mat,"Selec<matrix>",   
  else if( cmdLine == "iselec" ) 
    {  cmdArgs >> op.imin >> op.imax; }
  else if( cmdLine == "jselec" ) 
    {  cmdArgs >> op.jmin >> op.jmax; },
  "Selec<matrix>\n  - {i|j}selec min max\t\t:Selec {rows|cols} indices.");


/* -------------------------------------------------------------------------- */
#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(sotClassType,sotType,sotTypeRes,index,className,CMDLINE,CMDHELP)   \
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeInName( void ) { return std::string(#sotType); }                               \
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeOutName( void ) { return std::string(#sotTypeRes); }                           \
  template<>                                                                            \
  const std::string sotClassType::CLASS_NAME                                            \
     = std::string(className);                                                          \
  template<>                                                                            \
  void sotClassType::commandLine( const std::string& cmdLine,                           \
                                  std::istringstream& cmdArgs,                          \
 			          std::ostream& os )                                    \
  {                                                                                     \
    if( cmdLine=="help" ) { os << CMDHELP << std::endl; }                               \
    CMDLINE                                                                             \
      else { Entity::commandLine(cmdLine,cmdArgs,os); }                              \
  }                                                                                     \
  extern "C" {                                                                          \
    Entity *regFunction##_##index( const std::string& objname )                      \
    {                                                                                   \
      return new sotClassType( objname );                                               \
    }                                                                                   \
  EntityRegisterer regObj##_##index( std::string(className),                         \
					  &regFunction##_##index );                     \
  }



struct MatrixColumnSelector
{
public:
  unsigned int imin,imax;
  unsigned int jcol;
  void operator()( const Matrix& m,Vector& res ) const 
  {
    if( (imax<imin)||(m.nbRows()<imax) ) return;
    if( m.nbCols()<jcol ) return;

    res.resize( imax-imin );
    for( unsigned int i=imin;i<imax;++i ) 
	res(i-imin)=m(i,jcol);
  }
};

typedef UnaryOp<Matrix,Vector,MatrixColumnSelector > selcMatCol;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(selcMatCol,matrix,vector,sec_mat_col,"Selec<matrix,column>",   
  else if( cmdLine == "iselec" ) 
    {  cmdArgs >> op.imin >> op.imax; }
  else if( cmdLine == "jselec" ) 
    {  cmdArgs >> op.jcol; },
  "Selec<matrix,column>\n  - iselec min max\t\t:Selec rows indices.\n - jselec\t\t:Selec column.");



struct HomogeneousMatrixToVector
{
  void operator()( const MatrixHomogeneous& M,ml::Vector& res )
  {
    MatrixRotation R; M.extract(R);
    VectorUTheta r; r.fromMatrix(R);
    ml::Vector t(3); M.extract(t);
    res.resize(6);
    for( int i=0;i<3;++i ) res(i)=t(i);
    for( int i=0;i<3;++i ) res(i+3)=r(i);
  }
};

typedef UnaryOp<MatrixHomogeneous,Vector,HomogeneousMatrixToVector> RT2V;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(RT2V,matrixHomo,matrix,RT2V_,"MatrixHomoToPoseUTheta", ,"");


struct SkewSymToVector
{
  void operator()( const Matrix& M,Vector& res )
  {
    res.resize(3);
    res(0) = M(7);
    res(1) = M(2);
    res(2) = M(3);
  }
};

typedef UnaryOp<Matrix,Vector,SkewSymToVector> SS2V;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(SS2V,matrix,vector,SS2V_,"SkewSymToVector", ,"");

struct VectorUThetaToHomogeneousMatrix
{
  void operator()( const ml::Vector& v,MatrixHomogeneous& res )
  {
    VectorUTheta ruth; ml::Vector trans(3); 
    for( int i=0;i<3;++i )
      {
 	trans(i)=v(i); 
	ruth(i)=v(i+3); 
      }

    MatrixRotation R; ruth.toMatrix(R);
    res.buildFrom(R,trans);
  }
};

typedef UnaryOp<Vector,MatrixHomogeneous,VectorUThetaToHomogeneousMatrix> PUTH2M;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(PUTH2M,vector6,matrixhome,PUTH2M_,"PoseUThetaToMatrixHomo", ,"");

struct HomogeneousMatrixToVectorQuaternion
{
  void operator()( const MatrixHomogeneous& M,ml::Vector& res )
  {
    MatrixRotation R; M.extract(R);
    VectorQuaternion r; r.fromMatrix(R);
    ml::Vector t(3); M.extract(t);
    res.resize(7);
    for( int i=0;i<3;++i ) res(i)=t(i);
    for( int i=0;i<4;++i ) res(i+3)=r(i);
  }
};

typedef UnaryOp<MatrixHomogeneous,Vector,HomogeneousMatrixToVectorQuaternion> RT2VQ;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(RT2VQ,matrixHomo,matrix,RT2VQ_,"MatrixHomoToPoseQuaternion", ,"");

struct HomogeneousMatrixToVectorRPY
{
  void operator()( const MatrixHomogeneous& M,ml::Vector& res )
  {
    MatrixRotation R; M.extract(R);
    VectorRollPitchYaw r; r.fromMatrix(R);
    ml::Vector t(3); M.extract(t);
    res.resize(6);
    for( unsigned int i=0;i<3;++i ) res(i)=t(i);
    for( unsigned int i=0;i<3;++i ) res(i+3)=r(i);
  }
};

typedef UnaryOp<MatrixHomogeneous,Vector,HomogeneousMatrixToVectorRPY> RT2VRPY;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(RT2VRPY,matrixHomo,matrix,RT2VRPY_,"MatrixHomoToPoseRollPitchYaw", ,"");

struct VectorRPYToHomogeneousMatrix
{
  void operator()( const ml::Vector& vect, MatrixHomogeneous& Mres )
  {

    VectorRollPitchYaw r; 
    for( unsigned int i=0;i<3;++i ) r(i)=vect(i+3);
    MatrixRotation R;  r.toMatrix(R);
    
    ml::Vector t(3); 
    for( unsigned int i=0;i<3;++i ) t(i)=vect(i);
    Mres.buildFrom(R,t);
  }
};

typedef UnaryOp<Vector,MatrixHomogeneous,VectorRPYToHomogeneousMatrix> VRPY2RT;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(VRPY2RT,vector,matrixHomo,VRPY2RT_,"PoseRollPitchYawToMatrixHomo", ,"");

struct VectorRPYToVector6D
{
  void operator()( const ml::Vector& vect,ml::Vector& vectres )
  {

    VectorRollPitchYaw r; 
    for( unsigned int i=0;i<3;++i ) r(i)=vect(i+3);
    MatrixRotation R;  r.toMatrix(R);
    
    VectorUTheta rrot; rrot.fromMatrix(R);

    vectres .resize(6); 
    for( unsigned int i=0;i<3;++i )
      {
	vectres(i)=vect(i);
	vectres(i+3)=rrot(i);
      }
  }
};

typedef UnaryOp<Vector,Vector,VectorRPYToVector6D> VRPY2VRUT;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(VRPY2VRUT,vector,vector,VRPY2RUT_,"PoseRollPitchYawToPoseUTheta", ,"");

struct HomoToMatrix
{
  void operator()( const MatrixHomogeneous& M,ml::Matrix& res )
  {  res=(ml::Matrix&)M;  }
};

typedef UnaryOp<MatrixHomogeneous,Matrix,HomoToMatrix> H2M;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(H2M,matrixHomo,matrix,H2M_,"HomoToMatrix", ,"");

struct MatrixToHomo
{
  void operator()( const ml::Matrix& M,MatrixHomogeneous& res )
  {  res=M;  }
};

typedef UnaryOp<Matrix,MatrixHomogeneous,MatrixToHomo> M2H;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(M2H,matrixHomo,matrix,M2H_,"MatrixToHomo", ,"");



struct HomogeneousMatrixToTwist
{
  void operator()( const MatrixHomogeneous& M,MatrixTwist& res )
  {
    res.buildFrom( M );
  }
};

typedef UnaryOp<MatrixHomogeneous,MatrixTwist,HomogeneousMatrixToTwist> H2Tw;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(H2Tw,matrixHomo,matrixTwist,H2Tw_,"HomoToTwist", ,"");

struct ExtractRotation
{
  void operator()( const MatrixHomogeneous& M,MatrixRotation& res )
  {
    M.extract(res);
  }
};

typedef UnaryOp<MatrixHomogeneous,MatrixRotation,ExtractRotation> H2R;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(H2R,matrixHomo,matrixRotation,H2R_,"HomoToRotation", ,"");

struct RPYtoMatrix
{
  void operator()( const VectorRollPitchYaw& r,MatrixRotation& res )
  {
    r.toMatrix(res);
  }
};

typedef UnaryOp<VectorRollPitchYaw,MatrixRotation,RPYtoMatrix> rpy2R;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(rpy2R,vectorRPY,matrixRotation,rpy2R_,"RPYToMatrix", ,"");

struct MatrixToRPY
{
  void operator()( const MatrixRotation& r,VectorRollPitchYaw & res )
  {
    res.fromMatrix(r);
  }
};

typedef UnaryOp<MatrixRotation,VectorRollPitchYaw,MatrixToRPY> R2rpy;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(R2rpy,matrixRotation,vectorRPY,R2rpy_,"MatrixToRPY", ,"");

struct QuaterniontoMatrix
{
  void operator()( const VectorQuaternion& r,MatrixRotation& res )
  {
    r.toMatrix(res);
  }
};

typedef UnaryOp<VectorQuaternion,MatrixRotation,QuaterniontoMatrix> Quaternion2R;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(Quaternion2R,vectorQuaternion,matrixRotation,Quaternion2R_,"QuaternionToMatrix", ,"");

struct MatrixToQuaternion
{
  void operator()( const MatrixRotation& r,VectorQuaternion & res )
  {
    res.fromMatrix(r);
  }
};


typedef UnaryOp<MatrixRotation,VectorQuaternion,MatrixToQuaternion> R2Quaternion;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(R2Quaternion,matrixRotation,vectorQuaternion,R2Quaternion_,"MatrixToQuaternion", ,"");

struct MatrixToUTheta
{
  void operator()( const MatrixRotation& r,VectorUTheta & res )
  {
    res.fromMatrix(r);
  }
};


typedef UnaryOp<MatrixRotation,VectorUTheta,MatrixToUTheta> R2UTheta;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(R2UTheta,matrixRotation,vectorUTheta,R2UTheta_,"MatrixToUTheta", ,"");


struct UThetaToQuaternion
{
  void operator()( const VectorUTheta& r,VectorQuaternion& res )
  {
    res.fromVector(r);
  }
};

typedef UnaryOp<VectorUTheta,VectorQuaternion,UThetaToQuaternion> UT2Quaternion;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(UT2Quaternion,vectorQuaternion,vectorUTheta,UT2Quaternion_,"UThetaToQuaternion", ,"");

struct Diagonalizer
{
public:
  Diagonalizer( void ) : nbr(0),nbc(0) {}
  unsigned int nbr, nbc;
  void operator()( const ml::Vector& r,ml::Matrix & res )
  {
    unsigned imax=r.size(),jmax=r.size();
    if(( nbr!=0)&&(nbc!=0)) { imax=nbr; jmax=nbc; }
    res.resize(imax,jmax);
    for( unsigned int i=0;i<imax;++i )
      for( unsigned int j=0;j<jmax;++j ) if( i==j ) res(i,i)=r(i); else res(i,j)=0;
  }
};

typedef UnaryOp<ml::Vector,ml::Matrix,Diagonalizer> v2mDiagonalizer;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(v2mDiagonalizer,vector,matrix,v2mDiag_,"MatrixDiagonal",
else if( cmdLine == "resize" ) 
{  cmdArgs>>std::ws; if( cmdArgs.good()) {cmdArgs >> op.nbr >> op.nbc;}
 else { os << "size = " << op.nbr << " x " << op.nbc << std::endl; } },"");


struct DirtyMemory
{
public:
DirtyMemory( void ) {}
unsigned int nbr, nbc;
void operator()( const ml::Vector& r,ml::Vector & res )
{
res=r;
}
};

typedef UnaryOp<ml::Vector,ml::Vector,DirtyMemory> v2mDirtyMemory;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(v2mDirtyMemory,vector,v2mDM_,"DirtyMemory", ,"");

struct MatrixRotToMatrix
{
public:
MatrixRotToMatrix( void ) {}
void operator()( const MatrixRotation& a,ml::Matrix& result )
{
	result = a;
}
};

typedef UnaryOp<MatrixRotation,ml::Matrix,MatrixRotToMatrix> mrot2matrix;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(mrot2matrix,matrixRotation,matrix,"MatrixRotToMatrix", ,"");

} // namespace sot
