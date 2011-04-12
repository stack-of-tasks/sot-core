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

#include <sot/core/unary-op.hh>

#include <boost/function.hpp>

#include <jrl/mal/boost.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-twist.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>
#include <sot/core/vector-quaternion.hh>

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

/* ------------------------------------------------------------------------------------------ */
/* --- GENERIC HELPERS ---------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------ */

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

    template< typename TypeIn, typename TypeOut >
    struct UnaryOpHeader
    {
      typedef TypeIn Tin;
      typedef TypeOut Tout;
      static const std::string & nameTypeIn(void) { return TypeNameHelper<Tin>::typeName; }
      static const std::string & nameTypeOut(void) { return TypeNameHelper<Tout>::typeName; }
      void addSpecificCommands(Entity&, Entity::CommandMap_t& ) {}
    };


  } /* namespace sot */
} /* namespace dynamicgraph */


#define ADD_COMMAND( name,def )					                 \
    commandMap.insert( std::make_pair( name,def ) )

#define REGISTER_UNARY_OP( OpType,name )                                         \
    template<>                                                                   \
    const std::string UnaryOp< OpType >::CLASS_NAME = std::string(#name);        \
    Entity *regFunction##_##name( const std::string& objname )                   \
    {                                                                            \
      return new UnaryOp< OpType >( objname );                                   \
    }                                                                            \
    EntityRegisterer regObj##_##name( std::string(#name),&regFunction##_##name)

/* ------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------ */

namespace dynamicgraph {
  namespace sot {

    /* -------------------------------------------------------------------------------------- */
    /* --- ALGEBRA SELECTORS ---------------------------------------------------------------- */
    /* -------------------------------------------------------------------------------------- */
    struct VectorSelecter
      : public UnaryOpHeader<dg::Vector, dg::Vector>
    {
      void operator()( const Tin& m,Vector& res ) const
      {
	assert( (imax<imin)||(m.size()<imax) );
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

    /* -------------------------------------------------------------------------------------- */
    struct MatrixSelector
      : public UnaryOpHeader<dg::Matrix, dg::Matrix>
    {
       void operator()( const Matrix& m,Matrix& res ) const
      {
	assert( (imax<imin)||(m.nbRows()<imax) );
	assert( (jmax<jmin)||(m.nbCols()<jmax) );
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

    /* -------------------------------------------------------------------------------------- */
    struct MatrixColumnSelector
      : public UnaryOpHeader<dg::Matrix, dg::Vector>
    {
    public:
      void operator()( const Tin& m,Tout& res ) const
      {
	assert( (imax<imin)||(m.nbRows()<imax) );
	assert( m.nbCols()<jcol );

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

    /* -------------------------------------------------------------------------------------- */
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

    /* -------------------------------------------------------------------------------------- */
    /* --- INVERSION ------------------------------------------------------------------------ */
    /* -------------------------------------------------------------------------------------- */

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

   /* -------------------------------------------------------------------------------------- */
   /* --- SE3/SO3 conversions -------------------------------------------------------------- */
   /* -------------------------------------------------------------------------------------- */

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

  } /* namespace sot */
} /* namespace dynamicgraph */



