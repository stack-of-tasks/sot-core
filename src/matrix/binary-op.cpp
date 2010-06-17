/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      binary-op.h
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

#include <sot-core/binary-op.h>

#include <sot-core/factory.h>
#include <sot-core/matrix-homogeneous.h>
#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot-core/matrix-rotation.h>
#include <sot-core/matrix-twist.h>
#include <sot-core/sotDebug.h>



#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD(sotClassType,sotType,index,className,CMDLINE,CMDHELP)  \
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeIn1Name( void ) { return std::string(#sotType); }                              \
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeIn2Name( void ) { return std::string(#sotType); }                              \
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
  extern "C" {                                                                          \
    Entity *regFunction##_##index( const std::string& objname )                      \
    {                                                                                   \
      return new sotClassType( objname );                                               \
    }                                                                                   \
  EntityRegisterer regObj##_##index( std::string(className),                         \
					  &regFunction##_##index );                     \
  }

#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(sotClassType,sotType,index,className)  \
 SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD(sotClassType,sotType,index,className, ,"No help")

using namespace ml;

template< typename T>
struct sotAdder
{
  double coeff1, coeff2;
  void operator()( const T& v1,const T& v2,T& res ) const { res=v1; res+=v2; }
};

typedef sotBinaryOp<Vector,Vector,Vector,sotAdder<Vector> > advector;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD
(advector,vector,ad_vector,"Add<vector>"
,else if( cmdLine=="coeff1" ){ cmdArgs>>op.coeff1; } 
else if( cmdLine=="coeff2" ){ cmdArgs>>op.coeff2; } 
else if( cmdLine=="print" ){ os<<"Add ["<<op.coeff1<<","<<op.coeff2<<"]"<<std::endl; }, 
"Add<vector>: \n - coeff{1|2} value.");


typedef sotBinaryOp<Matrix,Matrix,Matrix,sotAdder<Matrix> > admatrix;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(admatrix,matrix,ad_matrix,"Add<matrix>");

typedef sotBinaryOp<double,double,double,sotAdder<double> > addouble; 
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(addouble,double,ad_double,"Add<double>");

/* -------------------------------------------------------------------------- */
/* --- MULTIPLICATION ------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

template< typename T>
struct sotMultiplier
{
  void operator()( const T& v1,const T& v2,T& res ) const { res=v1; res*=v2; }
};
template<>
void sotMultiplier<Matrix>::
operator()( const Matrix& v1,const Matrix& v2,Matrix& res ) const { v1.multiply(v2,res); }
template<>
void sotMultiplier<sotMatrixHomogeneous>::
operator()( const sotMatrixHomogeneous& v1,const sotMatrixHomogeneous& v2,
	    sotMatrixHomogeneous& res ) const 
{ v1.multiply(v2,res); }
template<>
void sotMultiplier<sotMatrixRotation>::
operator()( const sotMatrixRotation& v1,const sotMatrixRotation& v2,
	    sotMatrixRotation& res ) const 
{ v1.multiply(v2,res); }
template<>
void sotMultiplier<sotMatrixTwist>::
operator()( const sotMatrixTwist& v1,const sotMatrixTwist& v2,
	    sotMatrixTwist& res ) const 
{ v1.multiply(v2,res); }
template<>
void sotMultiplier<sotVectorQuaternion>::
operator()(const sotVectorQuaternion& q1,const sotVectorQuaternion& q2,
	   sotVectorQuaternion& res) const
{ q1.multiply(q2,res); }

typedef sotBinaryOp<Vector,Vector,Vector,sotMultiplier<Vector> > multvector;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multvector,vector,mult_vector,"Multiply<vector>");

typedef sotBinaryOp<Matrix,Matrix,Matrix,sotMultiplier<Matrix> > multmatrix;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multmatrix,matrix,mult_matrix,"Multiply<matrix>");
typedef sotBinaryOp<sotMatrixHomogeneous,sotMatrixHomogeneous,sotMatrixHomogeneous,sotMultiplier<sotMatrixHomogeneous> > multmatrixhomo;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multmatrixhomo,matrixhomo,mult_matrixhomo,"Multiply<matrixhomo>");
typedef sotBinaryOp<sotMatrixRotation,sotMatrixRotation,sotMatrixRotation,sotMultiplier<sotMatrixRotation> > multmatrixrot;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multmatrixrot,matrixrot,mult_matrixrot,"Multiply<matrixrotation>");
typedef sotBinaryOp<sotMatrixTwist,sotMatrixTwist,sotMatrixTwist,sotMultiplier<sotMatrixTwist> > multmatrixtwist;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multmatrixtwist,matrixtwist,mult_matrixtwist,"Multiply<matrixtwist>");
typedef sotBinaryOp<sotVectorQuaternion,sotVectorQuaternion,sotVectorQuaternion,sotMultiplier<sotVectorQuaternion> > multquat;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multquat,q,mult_q,"Multiply<quaternion>");

typedef sotBinaryOp<double,double,double,sotMultiplier<double> > multdouble;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multdouble,double,mult_double,"Multiply<double>");
/* -------------------------------------------------------------------------- */
/* --- SUBSTRACTION --------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

template< typename T>
struct sotSubstract
{
  void operator()( const T& v1,const T& v2,T& res ) const { res=v1; res-=v2; }
};

typedef sotBinaryOp<Vector,Vector,Vector,sotSubstract<Vector> > subsvector;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(subsvector,vector,subs_vector,"Substract<vector>");

typedef sotBinaryOp<Matrix,Matrix,Matrix,sotSubstract<Matrix> > subsmatrix;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(subsmatrix,matrix,subs_matrix,"Substract<matrix>");

typedef sotBinaryOp<double,double,double,sotSubstract<double> > subsdouble;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(subsdouble,double,subs_double,"Substract<double>");
/* -------------------------------------------------------------------------- */
/* --- STACK ---------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */


struct sotVectorStack
{
public:
  unsigned int v1min,v1max;
  unsigned int v2min,v2max;
  void operator()( const ml::Vector& v1,const ml::Vector& v2,ml::Vector& res ) const 
  { 
    unsigned int v1min_local=0,v1max_local=0;
    unsigned int v2min_local=0,v2max_local=0;

    if( (v1max>=v1min)&&(v1.size()>=v1max) ) 
      { v1min_local=v1min;   v1max_local=v1max; }
    else { v1min_local=0; v1max_local=v1.size(); }
    if( (v2max>=v2min)&&(v2.size()>=v2max) ) 
      { v2min_local=v2min;   v2max_local=v2max; }
    else { v2min_local=0; v2max_local=v2.size(); }

    const unsigned int v1size = v1max_local-v1min_local;
    const unsigned int v2size = v2max_local-v2min_local;
    res.resize( v1size+v2max_local-v2min_local );
    for( unsigned int i=0;i<v1size;++i )
      { res(i) = v1(i+v1min_local); }
    for( unsigned int i=0;i<v2size;++i )
      { res(v1size+i) = v2(i+v2min_local); }
  }
};
typedef sotBinaryOp< Vector,Vector,Vector,sotVectorStack > stackvector;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD(stackvector,vector,stack_vector,"Stack<vector>",else if( cmdLine=="selec1" ){ cmdArgs>>op.v1min>>op.v1max; } 
   else if( cmdLine=="selec2" ){ cmdArgs>>op.v2min>>op.v2max; } 
   else if( cmdLine=="print" ){ os<<"Stack ["<<op.v1min<<","<<op.v1max<<"] - ["<<op.v2min<<","<<op.v2max<<"] "<<std::endl; }, 
"Stack<vector>: \n - select{1|2} index_min index_max.");

/* -------------------------------------------------------------------------- */
/* --- ADDER WEIGHTED ------------------------------------------------------- */
/* -------------------------------------------------------------------------- */


struct sotWeightedAdder
{
public:
  double gain1,gain2;
  void operator()( const ml::Vector& v1,const ml::Vector& v2,ml::Vector& res ) const 
  { 
    res=v1; res*=gain1;
    res += gain2*v2;
  }
};
typedef sotBinaryOp< Vector,Vector,Vector,sotWeightedAdder > weightadd;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD(weightadd,vector,weight_add,"WeightAdd<vector>",else if( cmdLine=="gain1" ){ cmdArgs>>op.gain1; } 
   else if( cmdLine=="gain2" ){ cmdArgs>>op.gain2;}
   else if( cmdLine=="print" ){os<<"WeightAdd: "<<op.gain1<<" "<<op.gain2<<std::endl; }, 
  "WeightAdd<vector>: \n - gain{1|2} gain.");

/* -------------------------------------------------------------------------- */

struct sotWeightedDirection
{
public:
  void operator()( const ml::Vector& v1,const ml::Vector& v2,ml::Vector& res ) const 
  { 
    const double norm1 = v1.norm();
    const double norm2 = v2.norm();
    res=v2; res*=norm1; 
    res*= (1/norm2);
  }
};
typedef sotBinaryOp< Vector,Vector,Vector,sotWeightedDirection > weightdir;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(weightdir,vector,weight_dir,"WeightDir");


/* -------------------------------------------------------------------------- */

struct sotNullificator
{
public:
  void operator()( const ml::Vector& v1,const ml::Vector& v2,ml::Vector& res ) const 
  { 
    const unsigned int s = std::max( v1.size(),v2.size() );
    res.resize(s);
    for( unsigned int i=0;i<s;++i )
      {
	if( v1(i)>v2(i) ) res(i)=v1(i)-v2(i);
	else 	if( v1(i)<-v2(i) ) res(i)=v1(i)+v2(i);
	else res(i)=0;
      }
  }
};
typedef sotBinaryOp< Vector,Vector,Vector,sotNullificator > vectNil;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(vectNil,vector,vectnil_,"Nullificator");



/* -------------------------------------------------------------------------- */

#define CMDARGS_INOUT(varname) \
  cmdArgs>>std::ws; if(! cmdArgs.good() ) os << #varname" = " << varname << std::endl; \
  else cmdArgs >> varname

struct sotVirtualSpring
{
public:
  double spring;

  void operator()( const ml::Vector& pos,const ml::Vector& ref,ml::Vector& res ) const 
  { 
    double norm = ref.norm(); 
    double dist = ref.scalarProduct(pos) / (norm*norm);
    
    res.resize( ref.size() );
    res = ref;  res *= dist; res -= pos;
    res *= spring;
  }
};
typedef sotBinaryOp< Vector,Vector,Vector,sotVirtualSpring > virtspring;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD
(virtspring,vector,virtspring_,
 "VirtualSpring"
 ,else if( cmdLine=="spring" ){  CMDARGS_INOUT(op.spring); }
 ,"VirtualSpring<pos,ref> compute the virtual force of a spring attache "
 "to the reference line <ref>. The eq is: k.(<ref|pos>/<ref|ref>.ref-pos)"
 "Params:\n  - spring: get/set the spring factor.");





/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(sotClassType,sotTypeE,sotTypeF,index,className)\
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeIn1Name( void ) { return std::string(#sotTypeE); }                             \
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeIn2Name( void ) { return std::string(#sotTypeF); }                             \
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeOutName( void ) { return std::string(#sotTypeE); }                             \
  template<>                                                                            \
  const std::string sotClassType::CLASS_NAME                                            \
     = std::string(className);                                                          \
  template<>                                                                            \
  void sotClassType::commandLine( const std::string& cmdLine,                           \
                                  std::istringstream& cmdArgs,                          \
 			          std::ostream& os )                                    \
  {                                                                                     \
    if( cmdLine=="help" ) { os << "NO HELP" << std::endl; }                             \
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

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_G(sotClassType,sotTypeE,sotTypeF,sotTypeG,index,className)\
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeIn1Name( void ) { return std::string(#sotTypeE); }                             \
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeIn2Name( void ) { return std::string(#sotTypeF); }                             \
  template<>                                                                            \
  std::string sotClassType::                                                            \
  getTypeOutName( void ) { return std::string(#sotTypeG); }                             \
  template<>                                                                            \
  const std::string sotClassType::CLASS_NAME                                            \
     = std::string(className);                                                          \
  template<>                                                                            \
  void sotClassType::commandLine( const std::string& cmdLine,                           \
                                  std::istringstream& cmdArgs,                          \
 			          std::ostream& os )                                    \
  {                                                                                     \
    if( cmdLine=="help" ) { os << "NO HELP" << std::endl; }                             \
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


struct sotComposer
{
  void operator() ( const ml::Matrix& R,const ml::Vector& t, sotMatrixHomogeneous& H ) const 
  { 
    for( int i=0;i<3;++i )
      {
	H(i,3)=t(i);
	for( int j=0;j<3;++j )
	  H(i,j) = R(i,j);
	H(3,i) = 0;
      }
    H(3,3)=1.;
  };
};
typedef sotBinaryOp<ml::Matrix,ml::Vector,sotMatrixHomogeneous,sotComposer > TandRtoH;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(TandRtoH,matrix,vector,composeTR,"Compose<R+T>");
struct sotVectorComposerPRPY
{
  void operator() ( const sotVectorRollPitchYaw& R,const ml::Vector& t, ml::Vector& H ) const 
  { 
    H.resize(6);
    for( int i=0;i<3;++i )
      {
	H(i) = t(i);
	H(i+3) = R(i);
      }
  };
};
typedef sotBinaryOp<sotVectorRollPitchYaw,ml::Vector,ml::Vector,sotVectorComposerPRPY > TandRPYtoV;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(TandRPYtoV,matrix,vector,composeTRPYV,"ComposeVector<RPY+T>");

struct sotVectorScalarMultiplyer
{
  void operator()(const ml::Vector& v, double a, ml::Vector& res)
  {
    unsigned size = v.size();
    res.resize(size);
    for( unsigned i=0;i<size;++i ) {
      res(i) = v(i) * a;
    }
  }
};
typedef sotBinaryOp<ml::Vector,double,ml::Vector,sotVectorScalarMultiplyer> VAndScalToV;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(VAndScalToV,vec,scal,newvec,"Multiply<vector,double>");

struct sotMatrixHomeComposerPRPY
{
  void operator() ( const sotVectorRollPitchYaw& r,const ml::Vector& t, sotMatrixHomogeneous& Mres ) const 
  { 
    sotMatrixRotation R;  r.toMatrix(R);
    
    Mres.buildFrom(R,t);

  };
};
typedef sotBinaryOp<sotVectorRollPitchYaw,ml::Vector,sotMatrixHomogeneous,sotMatrixHomeComposerPRPY > TandRPYtoM;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_G(TandRPYtoM,vectorRPY,vector,matrixHomo,composeTRPYM,"Compose<RPY+T>");


/* This one is really awkward. It is to change the basis of the 
 * transfo matrix seen as an endomorhpism. The params are the initial transfo M = [ R t ]
 * and the new basis P. 
 * The result is P.M.P' = [ P.R.P' P.t ].
 * Strange, isn't it?
 */
struct sotEndomorphismBasis
{
  void operator() ( const sotMatrixHomogeneous& M, 
		    const sotMatrixRotation& P,
		    sotMatrixHomogeneous& res ) const 
  { 
    sotMatrixRotation R; M.extract(R);
    ml::Vector t(3); M.extract(t);

    ml::Vector tres(3); P.multiply(t,tres);
    sotMatrixRotation PR,PRP;
    P.multiply(R,PR);
    PR.multiply(P.transpose(),PRP);

    res.buildFrom( PRP,tres );
  };
};



typedef sotBinaryOp<sotMatrixHomogeneous,sotMatrixRotation,sotMatrixHomogeneous,sotEndomorphismBasis > endoMRM;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(endoMRM,matrixhomo,matrixrotation,endoMRM_,"EndomorphismBasis");



template< typename T1,typename T2 >
struct sotMultiplierE_F
{
  void operator()( const T1& v1,const T2& m2,T1& res ) const 
  { m2.multiply(v1,res); }
};

typedef sotBinaryOp<ml::Vector,ml::Matrix,ml::Vector,sotMultiplierE_F<ml::Vector,ml::Matrix> > multmatrixvector;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(multmatrixvector,vector,matrix,multmatrixvector,"Multiply<vector,matrix>");

typedef sotBinaryOp<ml::Vector,sotMatrixHomogeneous,ml::Vector,sotMultiplierE_F<ml::Vector,sotMatrixHomogeneous> > multmatrixhomovector;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(multmatrixhomovector,vector,matrixHomo,multmatrixhomovector,"Multiply<vector,matrixHomo>");



/* --- CONVOLUTION PRODUCT --- */
#include <deque>

struct sotConvolutionTemporal
{
  typedef std::deque<ml::Vector> MemoryType;
  MemoryType memory;

  void convolution( const MemoryType &f1,const ml::Matrix & f2,
		    ml::Vector &res )
  {
    const unsigned int nconv = f1.size(),nsig=f2.nbRows();
    sotDEBUG(15) << "Size: " << nconv << "x" << nsig << std::endl;
    if( nconv>f2.nbCols() ) return; // TODO: error, this should not happen

    res.resize( nsig ); res.fill(0);
    unsigned int j=0;
    for( MemoryType::const_iterator iter=f1.begin();iter!=f1.end();iter++ )
      {
	const ml::Vector & s_tau = *iter;
	sotDEBUG(45) << "Sig"<<j<< ": " << s_tau ;
	if( s_tau.size()!=nsig ) 
	  return; // TODO: error throw;
	for( unsigned int i=0;i<nsig;++i ) 
	  {
	    res(i)+=f2(i,j)*s_tau(i);	
	  }
	j++;
      }
  }

  void operator()( const ml::Vector& v1,const ml::Matrix& m2,ml::Vector& res ) 
  {
    memory.push_front( v1 );
    while( memory.size()>m2.nbCols() ) memory.pop_back();
    convolution( memory,m2,res );
 	sotDEBUG(45) << "Res = " << res ;
   return ;
  }

};

typedef sotBinaryOp<ml::Vector,ml::Matrix,ml::Vector,sotConvolutionTemporal> convtemp;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(convtemp,vector,matrix,convtemp,"ConvolutionTemporal");

