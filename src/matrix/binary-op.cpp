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

#include <sot/core/binary-op.hh>

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/all-commands.h>
#include <sot/core/factory.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/matrix-twist.hh>
#include <sot/core/debug.hh>

#include <deque>

namespace dg = ::dynamicgraph;

namespace dynamicgraph {
  namespace sot {
    template< typename TypeRef >
    struct TypeNameHelper
    {
      static const std::string typeName;
    };
    template< typename TypeRef >
    const std::string TypeNameHelper<TypeRef>::typeName = "unspecified";

#define ADD_KNOWN_TYPE( typeid ) \
    template<>const std::string TypeNameHelper<typeid>::typeName = #typeid

    ADD_KNOWN_TYPE(dg::Vector);
    ADD_KNOWN_TYPE(dg::Matrix);
    ADD_KNOWN_TYPE(MatrixRotation);
    ADD_KNOWN_TYPE(MatrixTwist);
    ADD_KNOWN_TYPE(MatrixHomogeneous);
    ADD_KNOWN_TYPE(VectorQuaternion);
    ADD_KNOWN_TYPE(VectorRollPitchYaw);

    template< typename TypeIn1,typename TypeIn2, typename TypeOut >
    struct BinaryOpHeader
    {
      typedef TypeIn1 Tin1;
      typedef TypeIn2 Tin2;
      typedef TypeOut Tout;
      static const std::string & nameTypeIn1(void) { return TypeNameHelper<Tin1>::typeName; }
      static const std::string & nameTypeIn2(void) { return TypeNameHelper<Tin2>::typeName; }
      static const std::string & nameTypeOut(void) { return TypeNameHelper<Tout>::typeName; }
      void addSpecificCommands(Entity&, Entity::CommandMap_t& ) {}
    };


  } /* namespace sot */
} /* namespace dynamicgraph */


#define ADD_COMMAND( name,def )					                 \
    commandMap.insert( std::make_pair( name,def ) )

#define REGISTER_BINARY_OP( OpType,name )                                        \
    template<>                                                                   \
    const std::string BinaryOp< OpType >::CLASS_NAME = std::string(#name);       \
    Entity *regFunction##_##name( const std::string& objname )                   \
    {                                                                            \
      return new BinaryOp< OpType >( objname );                                  \
    }                                                                            \
    EntityRegisterer regObj##_##name( std::string(#name),&regFunction##_##name)

/* ------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------ */


namespace dynamicgraph {
  namespace sot {

    /* --- ADDITION ----------------------------------------------------------------------------- */
    template< typename T>
    struct Adder
      : public BinaryOpHeader<T,T,T>
    {
      double coeff1, coeff2;
      void operator()( const T& v1,const T& v2,T& res ) const { res=v1; res+=v2; }

      void addSpecificCommands(Entity& ent,
       			       Entity::CommandMap_t& commandMap )
      {
	using namespace dynamicgraph::command;
	std::string doc;

	ADD_COMMAND( "setCoeff1",
		     makeDirectSetter(ent,&coeff1,docDirectSetter("coeff1","double")));
	ADD_COMMAND( "setCoeff2",
		     makeDirectSetter(ent,&coeff2,docDirectSetter("coeff2","double")));
      }
    };


    REGISTER_BINARY_OP(Adder<ml::Matrix>,Add_of_matrix);
    REGISTER_BINARY_OP(Adder<ml::Vector>,Add_of_vector);
    REGISTER_BINARY_OP(Adder<double>,Add_of_double);


    /* --- MULTIPLICATION ------------------------------------------------------- */

    template< typename T>
    struct Multiplier
      : public BinaryOpHeader<T,T,T>
    {
      void operator()( const T& v1,const T& v2,T& res ) const { v1.multiply(v2,res); }
    };
    template<> void Multiplier<double>::
    operator()( const double& v1,const double& v2,double& res ) const
    { res=v1; res*=v2; }

    REGISTER_BINARY_OP(Multiplier<ml::Matrix>,Multiply_of_matrix);
    REGISTER_BINARY_OP(Multiplier<ml::Vector>,Multiply_of_vector);
    REGISTER_BINARY_OP(Multiplier<MatrixRotation>,Multiply_of_matrixrotation);
    REGISTER_BINARY_OP(Multiplier<MatrixHomogeneous>,Multiply_of_matrixHomo);
    REGISTER_BINARY_OP(Multiplier<MatrixTwist>,Multiply_of_matrixtwist);
    REGISTER_BINARY_OP(Multiplier<VectorQuaternion>,Multiply_of_quaternion);
    REGISTER_BINARY_OP(Multiplier<double>,Multiply_of_double);

    template< typename F,typename E>
    struct Multiplier_FxE__E
      : public BinaryOpHeader<F,E,E>
    {
      void operator()( const F& f,const E& e, E& res ) const { f.multiply(e,res); }
    };
    template<> void Multiplier_FxE__E<double,ml::Vector>::
    operator()( const double& x,const ml::Vector& v,ml::Vector& res ) const
    { res=v; res*=x; }

    typedef Multiplier_FxE__E<double,ml::Vector> Multiplier_double_vector;
    typedef Multiplier_FxE__E<ml::Matrix,ml::Vector> Multiplier_matrix_vector;
    typedef Multiplier_FxE__E<MatrixHomogeneous,ml::Vector> Multiplier_matrixHomo_vector;
    REGISTER_BINARY_OP( Multiplier_double_vector,Multiply_double_vector);
    REGISTER_BINARY_OP( Multiplier_matrix_vector,Multiply_matrix_vector);
    REGISTER_BINARY_OP( Multiplier_matrixHomo_vector,Multiply_matrixHomo_vector);

    /* --- SUBSTRACTION --------------------------------------------------------- */
    template< typename T>
    struct Substraction
      : public BinaryOpHeader<T,T,T>
    { void operator()( const T& v1,const T& v2,T& r ) const { r=v1; r-=v2; } };

    REGISTER_BINARY_OP(Substraction<ml::Matrix>,Substract_of_matrix);
    REGISTER_BINARY_OP(Substraction<ml::Vector>,Substract_of_vector);
    REGISTER_BINARY_OP(Substraction<double>,Substract_of_double);

    /* --- STACK ---------------------------------------------------------------- */
    struct VectorStack
      : public BinaryOpHeader<ml::Vector,ml::Vector,ml::Vector>
    {
    public:
      unsigned int v1min,v1max;
      unsigned int v2min,v2max;
      void operator()( const ml::Vector& v1,const ml::Vector& v2,ml::Vector& res ) const
      {
	assert( (v1max>=v1min)&&(v1.size()>=v1max) );
	assert( (v2max>=v2min)&&(v2.size()>=v2max) );

	const unsigned int v1size = v1max-v1min, v2size = v2max-v2min;
	res.resize( v1size+v2size );
	for( unsigned int i=0;i<v1size;++i ) { res(i) = v1(i+v1min); }
	for( unsigned int i=0;i<v2size;++i ) { res(v1size+i) = v2(i+v2min); }
      }

      void selec1( const int & m, const int M) { v1min=m; v1max=M; }
      void selec2( const int & m, const int M) { v2min=m; v2max=M; }

      void addSpecificCommands(Entity& ent,
       			       Entity::CommandMap_t& commandMap )
      {
	using namespace dynamicgraph::command;
	std::string doc;

	boost::function< void( const int&, const int& ) > selec1
	  = boost::bind( &VectorStack::selec1,this,_1,_2 );
	boost::function< void( const int&, const int& ) > selec2
	  = boost::bind( &VectorStack::selec2,this,_1,_2 );

	ADD_COMMAND( "selec1",
	 	     makeCommandVoid2(ent,selec1,docCommandVoid2("set the min and max of selection.",
	 							 "int (imin)","int (imax)")));
	ADD_COMMAND( "selec2",
	 	     makeCommandVoid2(ent,selec2,docCommandVoid2("set the min and max of selection.",
	 							 "int (imin)","int (imax)")));
      }
    };
    REGISTER_BINARY_OP(VectorStack,Stack_of_vector);

    /* ---------------------------------------------------------------------- */

    struct Composer
      : public BinaryOpHeader<ml::Matrix,ml::Vector,MatrixHomogeneous>
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
      }
    };
    REGISTER_BINARY_OP(Composer,Compose_R_and_T);

    /* --- CONVOLUTION PRODUCT ---------------------------------------------- */
    struct ConvolutionTemporal
      : public BinaryOpHeader<ml::Vector,ml::Matrix,ml::Vector>
    {
      typedef std::deque<ml::Vector> MemoryType;
      MemoryType memory;

      void convolution( const MemoryType &f1,const ml::Matrix & f2,ml::Vector &res )
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
      }
    };
    REGISTER_BINARY_OP( ConvolutionTemporal,ConvolutionTemporal );

} /* namespace sot */} /* namespace dynamicgraph */



/* --- TODO -----------------------------------------------------------------------*/
// The following commented lines are sot-v1 entities that are still waiting
//   for conversion. Help yourself!

// struct WeightedAdder
// {
// public:
//   double gain1,gain2;
//   void operator()( const ml::Vector& v1,const ml::Vector& v2,ml::Vector& res ) const
//   {
//     res=v1; res*=gain1;
//     res += gain2*v2;
//   }
// };
// typedef BinaryOp< Vector,Vector,Vector,WeightedAdder > weightadd;
// SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD(weightadd,vector,weight_add,"WeightAdd_of_vector",else if( cmdLine=="gain1" ){ cmdArgs>>op.gain1; }
//    else if( cmdLine=="gain2" ){ cmdArgs>>op.gain2;}
//    else if( cmdLine=="print" ){os<<"WeightAdd: "<<op.gain1<<" "<<op.gain2<<std::endl; },
//   "WeightAdd<vector>: \n - gain{1|2} gain.")

// /* -------------------------------------------------------------------------- */

// struct WeightedDirection
// {
// public:
//   void operator()( const ml::Vector& v1,const ml::Vector& v2,ml::Vector& res ) const
//   {
//     const double norm1 = v1.norm();
//     const double norm2 = v2.norm();
//     res=v2; res*=norm1;
//     res*= (1/norm2);
//   }
// };
// typedef BinaryOp< Vector,Vector,Vector,WeightedDirection > weightdir;
// SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(weightdir,vector,weight_dir,"WeightDir")


// /* -------------------------------------------------------------------------- */

// struct Nullificator
// {
// public:
//   void operator()( const ml::Vector& v1,const ml::Vector& v2,ml::Vector& res ) const
//   {
//     const unsigned int s = std::max( v1.size(),v2.size() );
//     res.resize(s);
//     for( unsigned int i=0;i<s;++i )
//       {
// 	if( v1(i)>v2(i) ) res(i)=v1(i)-v2(i);
// 	else 	if( v1(i)<-v2(i) ) res(i)=v1(i)+v2(i);
// 	else res(i)=0;
//       }
//   }
// };
// typedef BinaryOp< Vector,Vector,Vector,Nullificator > vectNil;
// SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(vectNil,vector,vectnil_,"Nullificator")



// /* -------------------------------------------------------------------------- */

// struct VirtualSpring
// {
// public:
//   double spring;

//   void operator()( const ml::Vector& pos,const ml::Vector& ref,ml::Vector& res ) const
//   {
//     double norm = ref.norm();
//     double dist = ref.scalarProduct(pos) / (norm*norm);

//     res.resize( ref.size() );
//     res = ref;  res *= dist; res -= pos;
//     res *= spring;
//   }
// };
// typedef BinaryOp< Vector,Vector,Vector,VirtualSpring > virtspring;
// SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E_CMD
// (virtspring,vector,virtspring_,
//  "VirtualSpring"
//  ,else if( cmdLine=="spring" ){  CMDARGS_INOUT(op.spring); }
//  ,"VirtualSpring<pos,ref> compute the virtual force of a spring attache "
//  "to the reference line <ref>. The eq is: k.(<ref|pos>/<ref|ref>.ref-pos)"
//  "Params:\n  - spring: get/set the spring factor.")

