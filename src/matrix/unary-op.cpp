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

template< typename matrixgen >
struct sotInverser
{
  void operator()( const matrixgen& m,matrixgen& res ) const { m.inverse(res); }
};

typedef sotUnaryOp<Matrix,Matrix,sotInverser<Matrix> > invMat;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(invMat,matrix,inv_mat,"Inverse<matrix>", ,"");

typedef sotUnaryOp<sotMatrixHomogeneous,sotMatrixHomogeneous,sotInverser<sotMatrixHomogeneous> > invMatHomo;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(invMatHomo,matrixHomo,inv_mathomo,"Inverse<matrixhomo>", ,"");

typedef sotUnaryOp<sotMatrixTwist,sotMatrixTwist,sotInverser<sotMatrixTwist> > invMatTwist;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(invMatTwist,matrixTwist,inv_mattwist,"Inverse<matrixtwist>", ,"");


struct sotInverserRot
{
  void operator()( const sotMatrixRotation& m,sotMatrixRotation& res ) const { m.transpose(res); }
};

typedef sotUnaryOp<sotMatrixRotation,sotMatrixRotation,sotInverserRot > invMatRot;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(invMatRot,matrixRot,inv_matrot,"Inverse<matrixrotation>", ,"");

struct sotInverserQuat
{
  void operator()( const sotVectorQuaternion& q, sotVectorQuaternion& res ) const
  {
    q.conjugate(res);
  }
};

typedef sotUnaryOp<sotVectorQuaternion,sotVectorQuaternion,sotInverserQuat> invQuat;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(invQuat,q,qstar,"Inverse<unitquat>", ,"");

struct sotVectorSelector
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

typedef sotUnaryOp<Vector,Vector,sotVectorSelector > selcVec;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(selcVec,vector,sec_vec,"Selec<vector>",   
  else if( cmdLine == "selec" ) 
    {  cmdArgs >> op.imin >> op.imax; },
  "Selec<vector>\n  - selec min max\t\t");

struct sotMatrixSelector
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

typedef sotUnaryOp<Matrix,Matrix,sotMatrixSelector > selcMat;
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



struct sotMatrixColumnSelector
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

typedef sotUnaryOp<Matrix,Vector,sotMatrixColumnSelector > selcMatCol;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(selcMatCol,matrix,vector,sec_mat_col,"Selec<matrix,column>",   
  else if( cmdLine == "iselec" ) 
    {  cmdArgs >> op.imin >> op.imax; }
  else if( cmdLine == "jselec" ) 
    {  cmdArgs >> op.jcol; },
  "Selec<matrix,column>\n  - iselec min max\t\t:Selec rows indices.\n - jselec\t\t:Selec column.");



struct sotHomogeneousMatrixToVector
{
  void operator()( const sotMatrixHomogeneous& M,ml::Vector& res )
  {
    sotMatrixRotation R; M.extract(R);
    sotVectorUTheta r; r.fromMatrix(R);
    ml::Vector t(3); M.extract(t);
    res.resize(6);
    for( int i=0;i<3;++i ) res(i)=t(i);
    for( int i=0;i<3;++i ) res(i+3)=r(i);
  }
};

typedef sotUnaryOp<sotMatrixHomogeneous,Vector,sotHomogeneousMatrixToVector> RT2V;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(RT2V,matrixHomo,matrix,RT2V_,"MatrixHomoToPoseUTheta", ,"");


struct sotSkewSymToVector
{
  void operator()( const Matrix& M,Vector& res )
  {
    res.resize(3);
    res(0) = M(7);
    res(1) = M(2);
    res(2) = M(3);
  }
};

typedef sotUnaryOp<Matrix,Vector,sotSkewSymToVector> SS2V;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(SS2V,matrix,vector,SS2V_,"SkewSymToVector", ,"");

struct sotVectorUThetaToHomogeneousMatrix
{
  void operator()( const ml::Vector& v,sotMatrixHomogeneous& res )
  {
    sotVectorUTheta ruth; ml::Vector trans(3); 
    for( int i=0;i<3;++i )
      {
 	trans(i)=v(i); 
	ruth(i)=v(i+3); 
      }

    sotMatrixRotation R; ruth.toMatrix(R);
    res.buildFrom(R,trans);
  }
};

typedef sotUnaryOp<Vector,sotMatrixHomogeneous,sotVectorUThetaToHomogeneousMatrix> PUTH2M;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(PUTH2M,vector6,matrixhome,PUTH2M_,"PoseUThetaToMatrixHomo", ,"");

struct sotHomogeneousMatrixToVectorQuaternion
{
  void operator()( const sotMatrixHomogeneous& M,ml::Vector& res )
  {
    sotMatrixRotation R; M.extract(R);
    sotVectorQuaternion r; r.fromMatrix(R);
    ml::Vector t(3); M.extract(t);
    res.resize(7);
    for( int i=0;i<3;++i ) res(i)=t(i);
    for( int i=0;i<4;++i ) res(i+3)=r(i);
  }
};

typedef sotUnaryOp<sotMatrixHomogeneous,Vector,sotHomogeneousMatrixToVectorQuaternion> RT2VQ;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(RT2VQ,matrixHomo,matrix,RT2VQ_,"MatrixHomoToPoseQuaternion", ,"");

struct sotHomogeneousMatrixToVectorRPY
{
  void operator()( const sotMatrixHomogeneous& M,ml::Vector& res )
  {
    sotMatrixRotation R; M.extract(R);
    sotVectorRollPitchYaw r; r.fromMatrix(R);
    ml::Vector t(3); M.extract(t);
    res.resize(6);
    for( unsigned int i=0;i<3;++i ) res(i)=t(i);
    for( unsigned int i=0;i<3;++i ) res(i+3)=r(i);
  }
};

typedef sotUnaryOp<sotMatrixHomogeneous,Vector,sotHomogeneousMatrixToVectorRPY> RT2VRPY;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(RT2VRPY,matrixHomo,matrix,RT2VRPY_,"MatrixHomoToPoseRollPitchYaw", ,"");

struct sotVectorRPYToHomogeneousMatrix
{
  void operator()( const ml::Vector& vect, sotMatrixHomogeneous& Mres )
  {

    sotVectorRollPitchYaw r; 
    for( unsigned int i=0;i<3;++i ) r(i)=vect(i+3);
    sotMatrixRotation R;  r.toMatrix(R);
    
    ml::Vector t(3); 
    for( unsigned int i=0;i<3;++i ) t(i)=vect(i);
    Mres.buildFrom(R,t);
  }
};

typedef sotUnaryOp<Vector,sotMatrixHomogeneous,sotVectorRPYToHomogeneousMatrix> VRPY2RT;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(VRPY2RT,vector,matrixHomo,VRPY2RT_,"PoseRollPitchYawToMatrixHomo", ,"");

struct sotVectorRPYToVector6D
{
  void operator()( const ml::Vector& vect,ml::Vector& vectres )
  {

    sotVectorRollPitchYaw r; 
    for( unsigned int i=0;i<3;++i ) r(i)=vect(i+3);
    sotMatrixRotation R;  r.toMatrix(R);
    
    sotVectorUTheta rrot; rrot.fromMatrix(R);

    vectres .resize(6); 
    for( unsigned int i=0;i<3;++i )
      {
	vectres(i)=vect(i);
	vectres(i+3)=rrot(i);
      }
  }
};

typedef sotUnaryOp<Vector,Vector,sotVectorRPYToVector6D> VRPY2VRUT;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(VRPY2VRUT,vector,vector,VRPY2RUT_,"PoseRollPitchYawToPoseUTheta", ,"");

struct sotHomoToMatrix
{
  void operator()( const sotMatrixHomogeneous& M,ml::Matrix& res )
  {  res=(ml::Matrix&)M;  }
};

typedef sotUnaryOp<sotMatrixHomogeneous,Matrix,sotHomoToMatrix> H2M;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(H2M,matrixHomo,matrix,H2M_,"HomoToMatrix", ,"");

struct sotMatrixToHomo
{
  void operator()( const ml::Matrix& M,sotMatrixHomogeneous& res )
  {  res=M;  }
};

typedef sotUnaryOp<Matrix,sotMatrixHomogeneous,sotMatrixToHomo> M2H;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(M2H,matrixHomo,matrix,M2H_,"MatrixToHomo", ,"");



struct sotHomogeneousMatrixToTwist
{
  void operator()( const sotMatrixHomogeneous& M,sotMatrixTwist& res )
  {
    res.buildFrom( M );
  }
};

typedef sotUnaryOp<sotMatrixHomogeneous,sotMatrixTwist,sotHomogeneousMatrixToTwist> H2Tw;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(H2Tw,matrixHomo,matrixTwist,H2Tw_,"HomoToTwist", ,"");

struct sotExtractRotation
{
  void operator()( const sotMatrixHomogeneous& M,sotMatrixRotation& res )
  {
    M.extract(res);
  }
};

typedef sotUnaryOp<sotMatrixHomogeneous,sotMatrixRotation,sotExtractRotation> H2R;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(H2R,matrixHomo,matrixRot,H2R_,"HomoToRotation", ,"");

struct sotRPYtoMatrix
{
  void operator()( const sotVectorRollPitchYaw& r,sotMatrixRotation& res )
  {
    r.toMatrix(res);
  }
};

typedef sotUnaryOp<sotVectorRollPitchYaw,sotMatrixRotation,sotRPYtoMatrix> rpy2R;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(rpy2R,vectorRPY,matrixRot,rpy2R_,"RPYToMatrix", ,"");

struct sotMatrixToRPY
{
  void operator()( const sotMatrixRotation& r,sotVectorRollPitchYaw & res )
  {
    res.fromMatrix(r);
  }
};

typedef sotUnaryOp<sotMatrixRotation,sotVectorRollPitchYaw,sotMatrixToRPY> R2rpy;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(R2rpy,matrixRot,vectorRPY,R2rpy_,"MatrixToRPY", ,"");

struct sotQuaterniontoMatrix
{
  void operator()( const sotVectorQuaternion& r,sotMatrixRotation& res )
  {
    r.toMatrix(res);
  }
};

typedef sotUnaryOp<sotVectorQuaternion,sotMatrixRotation,sotQuaterniontoMatrix> Quaternion2R;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(Quaternion2R,vectorQuaternion,matrixRot,Quaternion2R_,"QuaternionToMatrix", ,"");

struct sotMatrixToQuaternion
{
  void operator()( const sotMatrixRotation& r,sotVectorQuaternion & res )
  {
    res.fromMatrix(r);
  }
};


typedef sotUnaryOp<sotMatrixRotation,sotVectorQuaternion,sotMatrixToQuaternion> R2Quaternion;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(R2Quaternion,matrixRot,vectorQuaternion,R2Quaternion_,"MatrixToQuaternion", ,"");

struct sotMatrixToUTheta
{
  void operator()( const sotMatrixRotation& r,sotVectorUTheta & res )
  {
    res.fromMatrix(r);
  }
};


typedef sotUnaryOp<sotMatrixRotation,sotVectorUTheta,sotMatrixToUTheta> R2UTheta;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(R2UTheta,matrixRot,vectorUTheta,R2UTheta_,"MatrixToUTheta", ,"");


struct sotUThetaToQuaternion
{
  void operator()( const sotVectorUTheta& r,sotVectorQuaternion& res )
  {
    res.fromVector(r);
  }
};

typedef sotUnaryOp<sotVectorUTheta,sotVectorQuaternion,sotUThetaToQuaternion> UT2Quaternion;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(UT2Quaternion,vectorQuaternion,vectorUTheta,UT2Quaternion_,"UThetaToQuaternion", ,"");

struct sotDiagonalizer
{
public:
  sotDiagonalizer( void ) : nbr(0),nbc(0) {}
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

typedef sotUnaryOp<ml::Vector,ml::Matrix,sotDiagonalizer> v2mDiagonalizer;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_F(v2mDiagonalizer,vector,matrix,v2mDiag_,"MatrixDiagonal",
else if( cmdLine == "resize" ) 
{  cmdArgs>>std::ws; if( cmdArgs.good()) {cmdArgs >> op.nbr >> op.nbc;}
 else { os << "size = " << op.nbr << " x " << op.nbc << std::endl; } },"");


struct sotDirtyMemory
{
public:
sotDirtyMemory( void ) {}
unsigned int nbr, nbc;
void operator()( const ml::Vector& r,ml::Vector & res )
{
res=r;
}
};

typedef sotUnaryOp<ml::Vector,ml::Vector,sotDirtyMemory> v2mDirtyMemory;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_E_E(v2mDirtyMemory,vector,v2mDM_,"DirtyMemory", ,"");
