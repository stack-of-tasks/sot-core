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

#include <sot-core/binary-op.h>

#include <sot-core/factory.h>
#include <sot/core/matrix-homogeneous.hh>
#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot/core/matrix-rotation.hh>
#include <sot-core/matrix-twist.h>
#include <sot-core/debug.h>

#include <deque>

namespace dynamicgraph { namespace sot {
using namespace dynamicgraph;

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
  SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD(sotClassType,sotType,index,className,else if( 0 ) {},"No help")

using namespace ml;

template< typename T>
struct Adder
{
  double coeff1, coeff2;
  void operator()( const T& v1,const T& v2,T& res ) const { res=v1; res+=v2; }
};

typedef BinaryOp<Vector,Vector,Vector,Adder<Vector> > advector;


SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD
(advector,vector,ad_vector,"Add_of_vector"
,else if( cmdLine=="coeff1" ){ cmdArgs>>op.coeff1; } 
else if( cmdLine=="coeff2" ){ cmdArgs>>op.coeff2; } 
else if( cmdLine=="print" ){ os<<"Add ["<<op.coeff1<<","<<op.coeff2<<"]"<<std::endl; }, 
"Add<vector>: \n - coeff{1|2} value.")


typedef BinaryOp<Matrix,Matrix,Matrix,Adder<Matrix> > admatrix;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(admatrix,matrix,ad_matrix,"Add_of_matrix")

typedef BinaryOp<double,double,double,Adder<double> > addouble; 
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(addouble,double,ad_double,"Add_of_double")

/* -------------------------------------------------------------------------- */
/* --- MULTIPLICATION ------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

template< typename T>
struct Multiplier
{
  void operator()( const T& v1,const T& v2,T& res ) const { res=v1; res*=v2; }
};
template<>
void Multiplier<Matrix>::
operator()( const Matrix& v1,const Matrix& v2,Matrix& res ) const { v1.multiply(v2,res); }
template<>
void Multiplier<MatrixHomogeneous>::
operator()( const MatrixHomogeneous& v1,const MatrixHomogeneous& v2,
	    MatrixHomogeneous& res ) const 
{ v1.multiply(v2,res); }
template<>
void Multiplier<MatrixRotation>::
operator()( const MatrixRotation& v1,const MatrixRotation& v2,
	    MatrixRotation& res ) const 
{ v1.multiply(v2,res); }
template<>
void Multiplier<MatrixTwist>::
operator()( const MatrixTwist& v1,const MatrixTwist& v2,
	    MatrixTwist& res ) const 
{ v1.multiply(v2,res); }
template<>
void Multiplier<VectorQuaternion>::
operator()(const VectorQuaternion& q1,const VectorQuaternion& q2,
	   VectorQuaternion& res) const
{ q1.multiply(q2,res); }

typedef BinaryOp<Vector,Vector,Vector,Multiplier<Vector> > multvector;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multvector,vector,mult_vector,"Multiply_of_vector")

typedef BinaryOp<Matrix,Matrix,Matrix,Multiplier<Matrix> > multmatrix;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multmatrix,matrix,mult_matrix,"Multiply_of_matrix")
typedef BinaryOp<MatrixHomogeneous,MatrixHomogeneous,MatrixHomogeneous,Multiplier<MatrixHomogeneous> > multmatrixhomo;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multmatrixhomo,matrixhomo,mult_matrixhomo,"Multiply_of_matrixhomo")
typedef BinaryOp<MatrixRotation,MatrixRotation,MatrixRotation,Multiplier<MatrixRotation> > multmatrixrot;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multmatrixrot,matrixrot,mult_matrixrot,"Multiply_of_matrixrotation")
typedef BinaryOp<MatrixTwist,MatrixTwist,MatrixTwist,Multiplier<MatrixTwist> > multmatrixtwist;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multmatrixtwist,matrixtwist,mult_matrixtwist,"Multiply_of_matrixtwist")
typedef BinaryOp<VectorQuaternion,VectorQuaternion,VectorQuaternion,Multiplier<VectorQuaternion> > multquat;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multquat,q,mult_q,"Multiply_of_quaternion")

typedef BinaryOp<double,double,double,Multiplier<double> > multdouble;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(multdouble,double,mult_double,"Multiply_of_double")
/* -------------------------------------------------------------------------- */
/* --- SUBSTRACTION --------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

template< typename T>
struct Substract
{
  void operator()( const T& v1,const T& v2,T& res ) const { res=v1; res-=v2; }
};

typedef BinaryOp<Vector,Vector,Vector,Substract<Vector> > subsvector;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(subsvector,vector,subs_vector,"Substract_of_vector")

typedef BinaryOp<Matrix,Matrix,Matrix,Substract<Matrix> > subsmatrix;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(subsmatrix,matrix,subs_matrix,"Substract_of_matrix")

typedef BinaryOp<double,double,double,Substract<double> > subsdouble;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(subsdouble,double,subs_double,"Substract_of_double")
/* -------------------------------------------------------------------------- */
/* --- STACK ---------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */


struct VectorStack
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
typedef BinaryOp< Vector,Vector,Vector,VectorStack > stackvector;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD(stackvector,vector,stack_vector,"Stack_of_vector",else if( cmdLine=="selec1" ){ cmdArgs>>op.v1min>>op.v1max; }
   else if( cmdLine=="selec2" ){ cmdArgs>>op.v2min>>op.v2max; } 
   else if( cmdLine=="print" ){ os<<"Stack ["<<op.v1min<<","<<op.v1max<<"] - ["<<op.v2min<<","<<op.v2max<<"] "<<std::endl; }, 
"Stack<vector>: \n - select{1|2} index_min index_max.")

/* -------------------------------------------------------------------------- */
/* --- ADDER WEIGHTED ------------------------------------------------------- */
/* -------------------------------------------------------------------------- */


struct WeightedAdder
{
public:
  double gain1,gain2;
  void operator()( const ml::Vector& v1,const ml::Vector& v2,ml::Vector& res ) const 
  { 
    res=v1; res*=gain1;
    res += gain2*v2;
  }
};
typedef BinaryOp< Vector,Vector,Vector,WeightedAdder > weightadd;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD(weightadd,vector,weight_add,"WeightAdd_of_vector",else if( cmdLine=="gain1" ){ cmdArgs>>op.gain1; }
   else if( cmdLine=="gain2" ){ cmdArgs>>op.gain2;}
   else if( cmdLine=="print" ){os<<"WeightAdd: "<<op.gain1<<" "<<op.gain2<<std::endl; }, 
  "WeightAdd<vector>: \n - gain{1|2} gain.")

/* -------------------------------------------------------------------------- */

struct WeightedDirection
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
typedef BinaryOp< Vector,Vector,Vector,WeightedDirection > weightdir;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(weightdir,vector,weight_dir,"WeightDir")


/* -------------------------------------------------------------------------- */

struct Nullificator
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
typedef BinaryOp< Vector,Vector,Vector,Nullificator > vectNil;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(vectNil,vector,vectnil_,"Nullificator")



/* -------------------------------------------------------------------------- */

#define CMDARGS_INOUT(varname) \
  cmdArgs>>std::ws; if(! cmdArgs.good() ) os << #varname" = " << varname << std::endl; \
  else cmdArgs >> varname

struct VirtualSpring
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
typedef BinaryOp< Vector,Vector,Vector,VirtualSpring > virtspring;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD
(virtspring,vector,virtspring_,
 "VirtualSpring"
 ,else if( cmdLine=="spring" ){  CMDARGS_INOUT(op.spring); }
 ,"VirtualSpring<pos,ref> compute the virtual force of a spring attache "
 "to the reference line <ref>. The eq is: k.(<ref|pos>/<ref|ref>.ref-pos)"
 "Params:\n  - spring: get/set the spring factor.")





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


struct Composer
{
  void operator() ( const ml::Matrix& R,const ml::Vector& t, MatrixHomogeneous& H ) const 
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
typedef BinaryOp<ml::Matrix,ml::Vector,MatrixHomogeneous,Composer > TandRtoH;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(TandRtoH,matrix,vector,composeTR,"Compose_R_and_T")
struct VectorComposerPRPY
{
  void operator() ( const VectorRollPitchYaw& R,const ml::Vector& t, ml::Vector& H ) const 
  { 
    H.resize(6);
    for( int i=0;i<3;++i )
      {
	H(i) = t(i);
	H(i+3) = R(i);
      }
  };
};
typedef BinaryOp<VectorRollPitchYaw,ml::Vector,ml::Vector,VectorComposerPRPY > TandRPYtoV;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(TandRPYtoV,matrix,vector,composeTRPYV,"ComposeVector_RPY_T")

struct VectorScalarMultiplyer
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
typedef BinaryOp<ml::Vector,double,ml::Vector,VectorScalarMultiplyer> VAndScalToV;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(VAndScalToV,vec,scal,newvec,"Multiply_vector_double")

struct MatrixHomeComposerPRPY
{
  void operator() ( const VectorRollPitchYaw& r,const ml::Vector& t, MatrixHomogeneous& Mres ) const 
  { 
    MatrixRotation R;  r.toMatrix(R);
    
    Mres.buildFrom(R,t);

  };
};
typedef BinaryOp<VectorRollPitchYaw,ml::Vector,MatrixHomogeneous,MatrixHomeComposerPRPY > TandRPYtoM;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_G(TandRPYtoM,vectorRPY,vector,matrixHomo,composeTRPYM,"Compose_RPY_and_T")


/* This one is really awkward. It is to change the basis of the 
 * transfo matrix seen as an endomorhpism. The params are the initial transfo M = [ R t ]
 * and the new basis P. 
 * The result is P.M.P' = [ P.R.P' P.t ].
 * Strange, isn't it?
 */
struct EndomorphismBasis
{
  void operator() ( const MatrixHomogeneous& M, 
		    const MatrixRotation& P,
		    MatrixHomogeneous& res ) const 
  { 
    MatrixRotation R; M.extract(R);
    ml::Vector t(3); M.extract(t);

    ml::Vector tres(3); P.multiply(t,tres);
    MatrixRotation PR,PRP;
    P.multiply(R,PR);
    PR.multiply(P.transpose(),PRP);

    res.buildFrom( PRP,tres );
  };
};



typedef BinaryOp<MatrixHomogeneous,MatrixRotation,MatrixHomogeneous,EndomorphismBasis > endoMRM;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(endoMRM,matrixhomo,matrixrotation,endoMRM_,"EndomorphismBasis")



template< typename T1,typename T2 >
struct MultiplierE_F
{
  void operator()( const T1& v1,const T2& m2,T1& res ) const 
  { m2.multiply(v1,res); }
};

typedef BinaryOp<ml::Vector,ml::Matrix,ml::Vector,MultiplierE_F<ml::Vector,ml::Matrix> > multmatrixvector;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(multmatrixvector,vector,matrix,multmatrixvector,"Multiply_vector_matrix")

typedef BinaryOp<ml::Vector,MatrixHomogeneous,ml::Vector,MultiplierE_F<ml::Vector,MatrixHomogeneous> > multmatrixhomovector;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(multmatrixhomovector,vector,matrixHomo,multmatrixhomovector,"Multiply_vector_matrixHomo")



/* --- CONVOLUTION PRODUCT --- */
struct ConvolutionTemporal
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

typedef BinaryOp<ml::Vector,ml::Matrix,ml::Vector,ConvolutionTemporal> convtemp;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExF_E(convtemp,vector,matrix,convtemp,"ConvolutionTemporal")

} /* namespace sot */} /* namespace dynamicgraph */
