/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 * Nicolas Mansard
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

#include <boost/function.hpp>

#include <sot/core/unary-op.hh>
#include <sot/core/binary-op.hh>
#include <jrl/mal/boost.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-twist.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>
#include <sot/core/vector-quaternion.hh>

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <dynamic-graph/linear-algebra.h>
#include <sot/core/factory.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/debug.hh>

#include <deque>

namespace dg = ::dynamicgraph;

/* ---------------------------------------------------------------------------*/
/* ------- GENERIC HELPERS -------------------------------------------------- */
/* ---------------------------------------------------------------------------*/

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

    ADD_KNOWN_TYPE(double);
    ADD_KNOWN_TYPE(dg::Vector);
    ADD_KNOWN_TYPE(dg::Matrix);
    ADD_KNOWN_TYPE(MatrixRotation);
    ADD_KNOWN_TYPE(MatrixTwist);
    ADD_KNOWN_TYPE(MatrixHomogeneous);
    ADD_KNOWN_TYPE(VectorQuaternion);
    ADD_KNOWN_TYPE(VectorRollPitchYaw);

    template< typename TypeIn, typename TypeOut >
    struct UnaryOpHeader
    {
      typedef TypeIn Tin;
      typedef TypeOut Tout;
      static const std::string & nameTypeIn(void) { return TypeNameHelper<Tin>::typeName; }
      static const std::string & nameTypeOut(void) { return TypeNameHelper<Tout>::typeName; }
      void addSpecificCommands(Entity&, Entity::CommandMap_t& ) {}
      virtual std::string getDocString () const {
	return std::string
	  ("Undocumented unary operator\n"
	   "  - input  ") + nameTypeIn () +
	  std::string ("\n"
		       "  - output ") + nameTypeOut () +
	  std::string ("\n");
      }
    };


  } /* namespace sot */
} /* namespace dynamicgraph */


#define ADD_COMMAND( name,def )				\
  commandMap.insert( std::make_pair( name,def ) )

#define REGISTER_UNARY_OP( OpType,name )				\
  template<>								\
  const std::string UnaryOp< OpType >::CLASS_NAME = std::string(#name);	\
  Entity *regFunction##_##name( const std::string& objname )		\
  {									\
    return new UnaryOp< OpType >( objname );				\
  }									\
  EntityRegisterer regObj##_##name( std::string(#name),&regFunction##_##name)

/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/

namespace dynamicgraph {
  namespace sot {

    /* ---------------------------------------------------------------------- */
    /* --- ALGEBRA SELECTORS ------------------------------------------------ */
    /* ---------------------------------------------------------------------- */
    struct VectorSelecter
      : public UnaryOpHeader<dg::Vector, dg::Vector>
    {
      void operator()( const Tin& m,Vector& res ) const
      {
	assert( (imin<=imax) && (imax <= m.size()) );
	res.resize( imax-imin );
	for( unsigned int i=imin;i<imax;++i ) res(i-imin)=m(i);
      }

      unsigned int imin,imax;
      void setBounds( const int & m,const int & M ) { imin = m; imax = M; }

      void addSpecificCommands(Entity& ent,
       			       Entity::CommandMap_t& commandMap )
      {
	using namespace dynamicgraph::command;
	std::string doc;

	boost::function< void( const int&, const int& ) > setBound
	  = boost::bind( &VectorSelecter::setBounds,this,_1,_2 );
	doc = docCommandVoid2("Set the bound of the selection [m,M[.","int (min)","int (max)");
	ADD_COMMAND( "selec", makeCommandVoid2(ent,setBound,doc) );
      }
    };
    REGISTER_UNARY_OP( VectorSelecter,Selec_of_vector );

    /* ---------------------------------------------------------------------- */
    /* --- ALGEBRA SELECTORS ------------------------------------------------ */
    /* ---------------------------------------------------------------------- */
    struct VectorComponent
      : public UnaryOpHeader<dg::Vector, double>
    {
      void operator() (const Tin& m, double& res) const
      {
	assert (index < m.size());
	res = m(index);
      }

      unsigned int index;
      void setIndex (const int & m) { index = m; }

      void addSpecificCommands(Entity& ent,
       			       Entity::CommandMap_t& commandMap )
      {
	std::string doc;

	boost::function< void( const int& ) > callback
	  = boost::bind( &VectorComponent::setIndex,this,_1 );
	doc = command::docCommandVoid1("Set the index of the component.",
				       "int (index)");
	ADD_COMMAND( "setIndex",
		     command::makeCommandVoid1 (ent, callback, doc));
      }
      virtual std::string getDocString () const
      {
	std::string docString
	  ("Select a component of a vector\n"
	   "  - input  vector\n""  - output double");
	return docString;
      }

    };
    REGISTER_UNARY_OP (VectorComponent, Component_of_vector);

    /* ---------------------------------------------------------------------- */
    struct MatrixSelector
      : public UnaryOpHeader<dg::Matrix, dg::Matrix>
    {
       void operator()( const Matrix& m,Matrix& res ) const
      {
	assert ((imin<=imax)&&(imax<=m.nbRows()));
	assert ((jmin<=jmax)&&(jmax<=m.nbCols()));
	res.resize( imax-imin,jmax-jmin );
	for( unsigned int i=imin;i<imax;++i )
	  for( unsigned int j=jmin;j<jmax;++j )
	    res(i-imin,j-jmin)=m(i,j);
      }

    public:
      unsigned int imin,imax;
      unsigned int jmin,jmax;

      void setBoundsRow( const int & m,const int & M ) { imin = m; imax = M; }
      void setBoundsCol( const int & m,const int & M ) { jmin = m; jmax = M; }

      void addSpecificCommands(Entity& ent,
       			       Entity::CommandMap_t& commandMap )
      {
	using namespace dynamicgraph::command;
	std::string doc;

	boost::function< void( const int&, const int& ) > setBoundsRow
	  = boost::bind( &MatrixSelector::setBoundsRow,this,_1,_2 );
	boost::function< void( const int&, const int& ) > setBoundsCol
	  = boost::bind( &MatrixSelector::setBoundsCol,this,_1,_2 );

	doc = docCommandVoid2("Set the bound on rows.","int (min)","int (max)");
	ADD_COMMAND( "selecRows", makeCommandVoid2(ent,setBoundsRow,doc) );

	doc = docCommandVoid2("Set the bound on cols [m,M[.","int (min)","int (max)");
	ADD_COMMAND( "selecCols", makeCommandVoid2(ent,setBoundsCol,doc) );
      }
    };
    REGISTER_UNARY_OP( MatrixSelector,Selec_of_matrix );

    /* ---------------------------------------------------------------------- */
    struct MatrixColumnSelector
      : public UnaryOpHeader<dg::Matrix, dg::Vector>
    {
    public:
      void operator()( const Tin& m,Tout& res ) const
      {
	assert ((imin<=imax)&&(imax<=m.nbRows()));
	assert (jcol<m.nbCols());

	res.resize( imax-imin );
	for( unsigned int i=imin;i<imax;++i )
	  res(i-imin)=m(i,jcol);
      }

      unsigned int imin,imax;
      unsigned int jcol;
      void selectCol( const int & m ) { jcol=m; }
      void setBoundsRow( const int & m,const int & M ) { imin = m; imax = M; }

      void addSpecificCommands(Entity& ent,
       			       Entity::CommandMap_t& commandMap )
      {
	using namespace dynamicgraph::command;
	std::string doc;

	boost::function< void( const int&, const int& ) > setBoundsRow
	  = boost::bind( &MatrixColumnSelector::setBoundsRow,this,_1,_2 );
	boost::function< void( const int& ) > selectCol
	  = boost::bind( &MatrixColumnSelector::selectCol,this,_1 );

	doc = docCommandVoid2("Set the bound on rows.","int (min)","int (max)");
	ADD_COMMAND( "selecRows", makeCommandVoid2(ent,setBoundsRow,doc) );

	doc = docCommandVoid1("Select the col to copy.","int (col index)");
	ADD_COMMAND( "selecCols", makeCommandVoid1(ent,selectCol,doc) );
      }
    };
    REGISTER_UNARY_OP( MatrixColumnSelector,Selec_column_of_matrix );

    /* ---------------------------------------------------------------------- */
    struct Diagonalizer
      : public UnaryOpHeader<Vector,Matrix>
    {
      void operator()( const dg::Vector& r,dg::Matrix & res )
      {
	unsigned imax=r.size(),jmax=r.size();
	if(( nbr!=0)&&(nbc!=0)) { imax=nbr; jmax=nbc; }
	res.resize(imax,jmax);
	for( unsigned int i=0;i<imax;++i )
	  for( unsigned int j=0;j<jmax;++j ) if( i==j ) res(i,i)=r(i); else res(i,j)=0;
      }
    public:
      Diagonalizer( void ) : nbr(0),nbc(0) {}
      unsigned int nbr, nbc;
      void resize( const int & r, const int & c ) { nbr=r;nbc=c; }
      void addSpecificCommands(Entity& ent,
       			       Entity::CommandMap_t& commandMap )
      {
	using namespace dynamicgraph::command;
	std::string doc;

	boost::function< void( const int&, const int& ) > resize
	  = boost::bind( &Diagonalizer::resize,this,_1,_2 );

	doc = docCommandVoid2("Set output size.","int (row)","int (col)");
	ADD_COMMAND( "resize", makeCommandVoid2(ent,resize,doc) );
      }
    };
    REGISTER_UNARY_OP(Diagonalizer,MatrixDiagonal);

    /* ---------------------------------------------------------------------- */
    /* --- INVERSION -------------------------------------------------------- */
    /* ---------------------------------------------------------------------- */

    template< typename matrixgen >
    struct Inverser
      : public UnaryOpHeader<matrixgen,matrixgen>
    {
      typedef typename UnaryOpHeader<matrixgen,matrixgen>::Tin Tin;
      typedef typename UnaryOpHeader<matrixgen,matrixgen>::Tout Tout;
      void operator()( const Tin& m,Tout& res ) const
      {	m.inverse(res); }
    };

    REGISTER_UNARY_OP( Inverser<dg::Matrix>, Inverse_of_matrix);
    REGISTER_UNARY_OP( Inverser<MatrixHomogeneous>, Inverse_of_matrixHomo);
    REGISTER_UNARY_OP( Inverser<MatrixTwist>, Inverse_of_matrixtwist);



    struct Normalize
      : public UnaryOpHeader<dg::Vector,double>
    {
      void operator()( const dg::Vector& m, double& res ) const
      { res = m.norm(); }

      virtual std::string getDocString () const
      {
	std::string docString
	  ("Computes the norm of a vector\n"
	   "  - input  vector\n""  - output double");
	return docString;
      }
    };
    REGISTER_UNARY_OP( Normalize, Norm_of_vector);


    struct InverserRotation
      : public UnaryOpHeader<MatrixRotation,MatrixRotation>
    {
      void operator()( const Tin& m,Tout& res )
	const { m.transpose(res); }
    };
    REGISTER_UNARY_OP( InverserRotation, Inverse_of_matrixrotation);

    struct InverserQuaternion
      : public UnaryOpHeader<VectorQuaternion,VectorQuaternion>
    {
      void operator()( const Tin& m,Tout& res )
	const { m.conjugate(res); }
    };
    REGISTER_UNARY_OP( InverserQuaternion, Inverse_of_unitquat);

   /* ----------------------------------------------------------------------- */
   /* --- SE3/SO3 conversions ----------------------------------------------- */
   /* ----------------------------------------------------------------------- */

    struct HomogeneousMatrixToVector
      : public UnaryOpHeader<MatrixHomogeneous,dg::Vector>
    {
      void operator()( const MatrixHomogeneous& M,dg::Vector& res )
      {
	MatrixRotation R; M.extract(R);
	VectorUTheta r; r.fromMatrix(R);
	dg::Vector t(3); M.extract(t);
	res.resize(6);
	for( int i=0;i<3;++i ) res(i)=t(i);
	for( int i=0;i<3;++i ) res(i+3)=r(i);
      }
    };
    REGISTER_UNARY_OP( HomogeneousMatrixToVector,MatrixHomoToPoseUTheta);

    struct SkewSymToVector
      : public UnaryOpHeader<Matrix,Vector>
    {
      void operator()( const Matrix& M,Vector& res )
      {
	res.resize(3);
	res(0) = M(7);
	res(1) = M(2);
	res(2) = M(3);
      }
    };
    REGISTER_UNARY_OP( SkewSymToVector,SkewSymToVector );

    struct PoseUThetaToMatrixHomo
      : public UnaryOpHeader<Vector,MatrixHomogeneous>
    {
      void operator()( const dg::Vector& v,MatrixHomogeneous& res )
      {
	assert( v.size()>=6 );
	VectorUTheta ruth; dg::Vector trans(3);
	for( int i=0;i<3;++i )
	  {
	    trans(i)=v(i);
	    ruth(i)=v(i+3);
	  }

	MatrixRotation R; ruth.toMatrix(R);
	res.buildFrom(R,trans);
      }
    };
    REGISTER_UNARY_OP(PoseUThetaToMatrixHomo,PoseUThetaToMatrixHomo);

    struct MatrixHomoToPoseQuaternion
      : public UnaryOpHeader<MatrixHomogeneous,Vector>
    {
      void operator()( const MatrixHomogeneous& M,Vector& res )
      {
	MatrixRotation R; M.extract(R);
	VectorQuaternion r; r.fromMatrix(R);
	dg::Vector t(3); M.extract(t);
	res.resize(7);
	for( int i=0;i<3;++i ) res(i)=t(i);
	for( int i=0;i<4;++i ) res(i+3)=r(i);
      }
    };
    REGISTER_UNARY_OP(MatrixHomoToPoseQuaternion,MatrixHomoToPoseQuaternion);

    struct MatrixHomoToPoseRollPitchYaw
      : public UnaryOpHeader<MatrixHomogeneous,Vector>
    {
      void operator()( const MatrixHomogeneous& M,dg::Vector& res )
      {
	MatrixRotation R; M.extract(R);
	VectorRollPitchYaw r; r.fromMatrix(R);
	dg::Vector t(3); M.extract(t);
	res.resize(6);
	for( unsigned int i=0;i<3;++i ) res(i)=t(i);
	for( unsigned int i=0;i<3;++i ) res(i+3)=r(i);
      }
    };
    REGISTER_UNARY_OP(MatrixHomoToPoseRollPitchYaw,MatrixHomoToPoseRollPitchYaw);

    struct PoseRollPitchYawToMatrixHomo
      : public UnaryOpHeader<Vector,MatrixHomogeneous>
    {
      void operator()( const dg::Vector& vect, MatrixHomogeneous& Mres )
      {

	VectorRollPitchYaw r;
	for( unsigned int i=0;i<3;++i ) r(i)=vect(i+3);
	MatrixRotation R;  r.toMatrix(R);

	dg::Vector t(3);
	for( unsigned int i=0;i<3;++i ) t(i)=vect(i);
	Mres.buildFrom(R,t);
      }
    };
    REGISTER_UNARY_OP(PoseRollPitchYawToMatrixHomo,PoseRollPitchYawToMatrixHomo);

    struct PoseRollPitchYawToPoseUTheta
      : public UnaryOpHeader<Vector,Vector>
    {
      void operator()( const dg::Vector& vect,dg::Vector& vectres )
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
    REGISTER_UNARY_OP(PoseRollPitchYawToPoseUTheta,PoseRollPitchYawToPoseUTheta);

    struct HomoToMatrix
      : public UnaryOpHeader<MatrixHomogeneous,Matrix>
    {
      void operator()( const MatrixHomogeneous& M,dg::Matrix& res )
      {  res=(dg::Matrix&)M;  }
    };
    REGISTER_UNARY_OP(HomoToMatrix,HomoToMatrix);

    struct MatrixToHomo
      : public UnaryOpHeader<Matrix,MatrixHomogeneous>
    {
      void operator()( const dg::Matrix& M,MatrixHomogeneous& res )
      {  res=M;  }
    };
    REGISTER_UNARY_OP(MatrixToHomo,MatrixToHomo);

    struct HomoToTwist
      : public UnaryOpHeader<MatrixHomogeneous,MatrixTwist>
    {
      void operator()( const MatrixHomogeneous& M,MatrixTwist& res )
      {
	res.buildFrom( M );
      }
    };
    REGISTER_UNARY_OP(HomoToTwist,HomoToTwist);

    struct HomoToRotation
       : public UnaryOpHeader<MatrixHomogeneous,MatrixRotation>
    {
      void operator()( const MatrixHomogeneous& M,MatrixRotation& res )
      {
	M.extract(res);
      }
    };
    REGISTER_UNARY_OP(HomoToRotation,HomoToRotation);

    struct MatrixHomoToPose
       : public UnaryOpHeader<MatrixHomogeneous,Vector>
    {
      void operator()( const MatrixHomogeneous& M,Vector& res )
      {
        res.resize(3);
        M.extract(res);
      }
    };
    REGISTER_UNARY_OP(MatrixHomoToPose,MatrixHomoToPose);

    struct RPYToMatrix
       : public UnaryOpHeader<VectorRollPitchYaw,MatrixRotation>
   {
      void operator()( const VectorRollPitchYaw& r,MatrixRotation& res )
      {
	r.toMatrix(res);
      }
    };
    REGISTER_UNARY_OP(RPYToMatrix,RPYToMatrix);

    struct MatrixToRPY
      : public UnaryOpHeader<MatrixRotation,VectorRollPitchYaw>
    {
      void operator()( const MatrixRotation& r,VectorRollPitchYaw & res )
      {
	res.fromMatrix(r);
      }
    };
    REGISTER_UNARY_OP(MatrixToRPY,MatrixToRPY);

    struct QuaternionToMatrix
      : public UnaryOpHeader<VectorQuaternion,MatrixRotation>
    {
      void operator()( const VectorQuaternion& r,MatrixRotation& res )
      {
	r.toMatrix(res);
      }
    };
    REGISTER_UNARY_OP(QuaternionToMatrix,QuaternionToMatrix);

    struct MatrixToQuaternion
      : public UnaryOpHeader<MatrixRotation,VectorQuaternion>
    {
      void operator()( const MatrixRotation& r,VectorQuaternion & res )
      {
	res.fromMatrix(r);
      }
    };
    REGISTER_UNARY_OP(MatrixToQuaternion,MatrixToQuaternion);

    struct MatrixToUTheta
      : public UnaryOpHeader<MatrixRotation,VectorUTheta>
    {
      void operator()( const MatrixRotation& r,VectorUTheta & res )
      {
	res.fromMatrix(r);
      }
    };
    REGISTER_UNARY_OP(MatrixToUTheta,MatrixToUTheta);

    struct UThetaToQuaternion
      : public UnaryOpHeader<VectorUTheta,VectorQuaternion>
    {
      void operator()( const VectorUTheta& r,VectorQuaternion& res )
      {
	res.fromVector(r);
      }
    };
    REGISTER_UNARY_OP(UThetaToQuaternion,UThetaToQuaternion);

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
      virtual std::string getDocString () const
      {
	return std::string
	  ("Undocumented binary operator\n"
	   "  - input  ") + nameTypeIn1 () +
	  std::string ("\n"
	   "  -        ") + nameTypeIn2 () +
	  std::string ("\n"
		       "  - output ") + nameTypeOut () +
	  std::string ("\n");
      }
    };


  } /* namespace sot */
} /* namespace dynamicgraph */


#define REGISTER_BINARY_OP( OpType,name )				\
  template<>								\
  const std::string BinaryOp< OpType >::CLASS_NAME = std::string(#name); \
  Entity *regFunction##_##name( const std::string& objname )		\
  {									\
    return new BinaryOp< OpType >( objname );				\
  }									\
  EntityRegisterer regObj##_##name( std::string(#name),&regFunction##_##name)

/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/


namespace dynamicgraph {
  namespace sot {

    /* --- ADDITION --------------------------------------------------------- */
    template< typename T>
    struct Adder
      : public BinaryOpHeader<T,T,T>
    {
      double coeff1, coeff2;
      Adder () : coeff1 (1.), coeff2 (1.) {}
      void operator()( const T& v1,const T& v2,T& res ) const
      {
	res=coeff1*v1; res+=coeff2*v2;
      }

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
      virtual std::string getDocString () const
      {
	return std::string
	  ("Linear combination of inputs\n"
	   "  - input  ") + BinaryOpHeader<T,T,T>::nameTypeIn1 () +
	  std::string ("\n"
	   "  -        ") + BinaryOpHeader<T,T,T>::nameTypeIn2 () +
	  std::string ("\n"
		       "  - output ") + BinaryOpHeader<T,T,T>::nameTypeOut () +
	  std::string ("\n""  sout = coeff1 * sin1 + coeff2 * sin2\n"
		       "  Coefficients are set by commands, default value is 1.\n");
      }
    };


    REGISTER_BINARY_OP(Adder<ml::Matrix>,Add_of_matrix);
    REGISTER_BINARY_OP(Adder<ml::Vector>,Add_of_vector);
    REGISTER_BINARY_OP(Adder<double>,Add_of_double);


    /* --- MULTIPLICATION --------------------------------------------------- */

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

    /* --- SUBSTRACTION ----------------------------------------------------- */
    template< typename T>
    struct Substraction
      : public BinaryOpHeader<T,T,T>
    { void operator()( const T& v1,const T& v2,T& r ) const { r=v1; r-=v2; } };

    REGISTER_BINARY_OP(Substraction<ml::Matrix>,Substract_of_matrix);
    REGISTER_BINARY_OP(Substraction<ml::Vector>,Substract_of_vector);
    REGISTER_BINARY_OP(Substraction<double>,Substract_of_double);

    /* --- STACK ------------------------------------------------------------ */
    struct VectorStack
      : public BinaryOpHeader<ml::Vector,ml::Vector,ml::Vector>
    {
    public:
      int v1min,v1max;
      int v2min,v2max;
      bool toEnd1,toEnd2;

      VectorStack():v1min(0),v1max(0),v2min(0),v2max(0),toEnd1(true),toEnd2(true)
      {}

      void operator()( const ml::Vector& v1,const ml::Vector& v2,ml::Vector& res ) const
      {
        int v1minP,v1maxP,v2minP,v2maxP;// positive versions of vi(min/max)
        if (v1min>=0)
          v1minP = v1min;
        else
          v1minP = v1.size()+v1min;

        if (toEnd1)
          v1maxP = v1.size();
        else if (v1max>=0)
          v1maxP = v1max;
        else
          v1maxP = v1.size()+v1max;

        if (v2min>=0)
          v2minP = v2min;
        else
          v2minP = v2.size()+v2min;

        if (toEnd2)
          v2maxP = v2.size();
        else if (v2max>=0)
          v2maxP = v2max;
        else
          v2maxP = v2.size()+v2max;


        assert( (v1maxP>=v1minP)&&(v1.size()>=v1maxP) );
        assert( (v2maxP>=v2minP)&&(v2.size()>=v2maxP) );

        const unsigned int v1size = v1maxP-v1minP, v2size = v2maxP-v2minP;
        res.resize( v1size+v2size );
        for( unsigned int i=0; i<v1size; ++i )
        {
          res(i) = v1(i+v1minP);
        }
        for( unsigned int i=0; i<v2size; ++i )
        {
          res(v1size+i) = v2(i+v2minP);
        }
      }

      void selec1( const int & m, const int M) { v1min=m; v1max=M; toEnd1=false;}
      void selec2( const int & m, const int M) { v2min=m; v2max=M; toEnd2=false;}

      void selecIndextoEnd1( const int & m) { v1min=m; toEnd1=true;}
      void selecIndextoEnd2( const int & m) { v2min=m; toEnd2=true;}

      void addSpecificCommands(Entity& ent,
       			       Entity::CommandMap_t& commandMap )
      {
	using namespace dynamicgraph::command;
	std::string doc;

	boost::function< void( const int&, const int& ) > selec1
	  = boost::bind( &VectorStack::selec1,this,_1,_2 );
	boost::function< void( const int&, const int& ) > selec2
	  = boost::bind( &VectorStack::selec2,this,_1,_2 );

    boost::function< void( const int& ) > selecIndextoEnd1
	  = boost::bind( &VectorStack::selecIndextoEnd1,this,_1 );
	boost::function< void( const int&) > selecIndextoEnd2
	  = boost::bind( &VectorStack::selecIndextoEnd2,this,_1 );

	ADD_COMMAND( "selec1",
	 	     makeCommandVoid2(ent,selec1,docCommandVoid2("set the min and max of selection.",
	 							 "int (imin)","int (imax)")));
	ADD_COMMAND( "selec2",
	 	     makeCommandVoid2(ent,selec2,docCommandVoid2("set the min and max of selection.",
	 							 "int (imin)","int (imax)")));
	ADD_COMMAND( "selec1toEnd",
	 	     makeCommandVoid1(ent,selecIndextoEnd1,docCommandVoid1("set the min of selection (max is vector size).",
	 							 "int (imin)")));
	ADD_COMMAND( "selec2toEnd",
	 	     makeCommandVoid1(ent,selecIndextoEnd2,docCommandVoid1("set the min of selection (max is vector size).",
	 							 "int (imin)")));
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



namespace dynamicgraph {
  namespace sot {

    template< typename T>
    struct WeightedAdder
      : public BinaryOpHeader<T,T,T>
    {
    public:
      double gain1,gain2;
      void operator()( const T& v1,const T& v2, T& res ) const
      {
        res=v1; res*=gain1;
        res += gain2*v2;
      }

      void addSpecificCommands(Entity& ent,
                   Entity::CommandMap_t& commandMap)
      {
        using namespace dynamicgraph::command;
        std::string doc;

        ADD_COMMAND( "setGain1",
            makeDirectSetter(ent,&gain1,docDirectSetter("gain1","double")));
        ADD_COMMAND( "setGain2",
            makeDirectSetter(ent,&gain2,docDirectSetter("gain2","double")));
        ADD_COMMAND( "getGain1",
            makeDirectGetter(ent,&gain1,docDirectGetter("gain1","double")));
        ADD_COMMAND( "getGain2",
            makeDirectGetter(ent,&gain2,docDirectGetter("gain2","double")));
      }

      virtual std::string getDocString () const
      {
        return std::string("Weighted Combination of inputs : \n - gain{1|2} gain.");
      }
    };

    REGISTER_BINARY_OP(WeightedAdder<ml::Matrix>,WeightAdd_of_matrix);
    REGISTER_BINARY_OP(WeightedAdder<ml::Vector>,WeightAdd_of_vector);
    REGISTER_BINARY_OP(WeightedAdder<double>,WeightAdd_of_double);
    }
}

/* --- TODO ------------------------------------------------------------------*/
// The following commented lines are sot-v1 entities that are still waiting
//   for conversion. Help yourself!

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

