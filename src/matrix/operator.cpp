/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 * Nicolas Mansard
 * Joseph Mirabel
 *
 * CNRS/AIST
 *
 */

#include "operator.hh"

namespace dg = ::dynamicgraph;

/* ---------------------------------------------------------------------------*/
/* ------- GENERIC HELPERS -------------------------------------------------- */
/* ---------------------------------------------------------------------------*/

#define REGISTER_UNARY_OP(OpType, name)                                        \
  template <>                                                                  \
  const std::string UnaryOp<OpType>::CLASS_NAME = std::string(#name);          \
  Entity *regFunction_##name(const std::string &objname) {                     \
    return new UnaryOp<OpType>(objname);                                       \
  }                                                                            \
  EntityRegisterer regObj_##name(std::string(#name), &regFunction_##name)

/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/

namespace dynamicgraph {
namespace sot {

/* ---------------------------------------------------------------------- */
/* --- ALGEBRA SELECTORS ------------------------------------------------ */
/* ---------------------------------------------------------------------- */
REGISTER_UNARY_OP(VectorSelecter, Selec_of_vector);
REGISTER_UNARY_OP(VectorComponent, Component_of_vector);
REGISTER_UNARY_OP(MatrixSelector, Selec_of_matrix);
REGISTER_UNARY_OP(MatrixColumnSelector, Selec_column_of_matrix);
REGISTER_UNARY_OP(MatrixTranspose, MatrixTranspose);
REGISTER_UNARY_OP(Diagonalizer, MatrixDiagonal);

/* ---------------------------------------------------------------------- */
/* --- INVERSION -------------------------------------------------------- */
/* ---------------------------------------------------------------------- */

REGISTER_UNARY_OP(Inverser<Matrix>, Inverse_of_matrix);
REGISTER_UNARY_OP(Inverser<MatrixHomogeneous>, Inverse_of_matrixHomo);
REGISTER_UNARY_OP(Inverser<MatrixTwist>, Inverse_of_matrixtwist);
REGISTER_UNARY_OP(Normalize, Norm_of_vector);
REGISTER_UNARY_OP(InverserRotation, Inverse_of_matrixrotation);
REGISTER_UNARY_OP(InverserQuaternion, Inverse_of_unitquat);

/* ----------------------------------------------------------------------- */
/* --- SE3/SO3 conversions ----------------------------------------------- */
/* ----------------------------------------------------------------------- */

REGISTER_UNARY_OP(MatrixHomoToPoseUTheta, MatrixHomoToPoseUTheta);
REGISTER_UNARY_OP(MatrixHomoToSE3Vector, MatrixHomoToSE3Vector);
REGISTER_UNARY_OP(SE3VectorToMatrixHomo, SE3VectorToMatrixHomo);
REGISTER_UNARY_OP(SkewSymToVector, SkewSymToVector);
REGISTER_UNARY_OP(PoseUThetaToMatrixHomo, PoseUThetaToMatrixHomo);
REGISTER_UNARY_OP(MatrixHomoToPoseQuaternion, MatrixHomoToPoseQuaternion);
REGISTER_UNARY_OP(PoseQuaternionToMatrixHomo, PoseQuaternionToMatrixHomo);
REGISTER_UNARY_OP(MatrixHomoToPoseRollPitchYaw, MatrixHomoToPoseRollPitchYaw);
REGISTER_UNARY_OP(PoseRollPitchYawToMatrixHomo, PoseRollPitchYawToMatrixHomo);
REGISTER_UNARY_OP(PoseRollPitchYawToPoseUTheta, PoseRollPitchYawToPoseUTheta);
REGISTER_UNARY_OP(HomoToMatrix, HomoToMatrix);
REGISTER_UNARY_OP(MatrixToHomo, MatrixToHomo);
REGISTER_UNARY_OP(HomoToTwist, HomoToTwist);
REGISTER_UNARY_OP(HomoToRotation, HomoToRotation);
REGISTER_UNARY_OP(MatrixHomoToPose, MatrixHomoToPose);
REGISTER_UNARY_OP(RPYToMatrix, RPYToMatrix);
REGISTER_UNARY_OP(MatrixToRPY, MatrixToRPY);
REGISTER_UNARY_OP(RPYToQuaternion, RPYToQuaternion);
REGISTER_UNARY_OP(QuaternionToRPY, QuaternionToRPY);
REGISTER_UNARY_OP(QuaternionToMatrix, QuaternionToMatrix);
REGISTER_UNARY_OP(MatrixToQuaternion, MatrixToQuaternion);
REGISTER_UNARY_OP(MatrixToUTheta, MatrixToUTheta);
REGISTER_UNARY_OP(UThetaToQuaternion, UThetaToQuaternion);

/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/

#define REGISTER_BINARY_OP(OpType, name)                                       \
  template <>                                                                  \
  const std::string BinaryOp<OpType>::CLASS_NAME = std::string(#name);         \
  Entity *regFunction_##name(const std::string &objname) {                     \
    return new BinaryOp<OpType>(objname);                                      \
  }                                                                            \
  EntityRegisterer regObj_##name(std::string(#name), &regFunction_##name)

/* --- MULTIPLICATION --------------------------------------------------- */

REGISTER_BINARY_OP(Multiplier_double_vector, Multiply_double_vector);
REGISTER_BINARY_OP(Multiplier_matrix_vector, Multiply_matrix_vector);
REGISTER_BINARY_OP(Multiplier_matrixHomo_vector, Multiply_matrixHomo_vector);
REGISTER_BINARY_OP(Multiplier_matrixTwist_vector, Multiply_matrixTwist_vector);

/* --- SUBSTRACTION ----------------------------------------------------- */
REGISTER_BINARY_OP(Substraction<dynamicgraph::Matrix>, Substract_of_matrix);
REGISTER_BINARY_OP(Substraction<dynamicgraph::Vector>, Substract_of_vector);
REGISTER_BINARY_OP(Substraction<double>, Substract_of_double);

/* --- STACK ------------------------------------------------------------ */
REGISTER_BINARY_OP(VectorStack, Stack_of_vector);

/* ---------------------------------------------------------------------- */

REGISTER_BINARY_OP(Composer, Compose_R_and_T);

/* --- CONVOLUTION PRODUCT ---------------------------------------------- */
REGISTER_BINARY_OP(ConvolutionTemporal, ConvolutionTemporal);

/* --- BOOLEAN REDUCTION ------------------------------------------------ */
REGISTER_BINARY_OP(Comparison<double>, CompareDouble);
REGISTER_BINARY_OP(MatrixComparison<Vector>, CompareVector);

REGISTER_BINARY_OP(WeightedAdder<dynamicgraph::Matrix>, WeightAdd_of_matrix);
REGISTER_BINARY_OP(WeightedAdder<dynamicgraph::Vector>, WeightAdd_of_vector);
REGISTER_BINARY_OP(WeightedAdder<double>, WeightAdd_of_double);

#define REGISTER_VARIADIC_OP(OpType, name)                                     \
  template <>                                                                  \
  const std::string VariadicOp<OpType>::CLASS_NAME = std::string(#name);       \
  Entity *regFunction_##name(const std::string &objname) {                     \
    return new VariadicOp<OpType>(objname);                                    \
  }                                                                            \
  EntityRegisterer regObj_##name(std::string(#name), &regFunction_##name)

/* --- VectorMix ------------------------------------------------------------ */
REGISTER_VARIADIC_OP(VectorMix, Mix_of_vector);

/* --- ADDITION --------------------------------------------------------- */
REGISTER_VARIADIC_OP(AdderVariadic<Matrix>, Add_of_matrix);
REGISTER_VARIADIC_OP(AdderVariadic<Vector>, Add_of_vector);
REGISTER_VARIADIC_OP(AdderVariadic<double>, Add_of_double);

/* --- MULTIPLICATION --------------------------------------------------- */
REGISTER_VARIADIC_OP(Multiplier<Matrix>, Multiply_of_matrix);
REGISTER_VARIADIC_OP(Multiplier<Vector>, Multiply_of_vector);
REGISTER_VARIADIC_OP(Multiplier<MatrixRotation>, Multiply_of_matrixrotation);
REGISTER_VARIADIC_OP(Multiplier<MatrixHomogeneous>, Multiply_of_matrixHomo);
REGISTER_VARIADIC_OP(Multiplier<MatrixTwist>, Multiply_of_matrixtwist);
REGISTER_VARIADIC_OP(Multiplier<VectorQuaternion>, Multiply_of_quaternion);
REGISTER_VARIADIC_OP(Multiplier<double>, Multiply_of_double);

/* --- BOOLEAN --------------------------------------------------------- */
REGISTER_VARIADIC_OP(BoolOp<0>, And);
REGISTER_VARIADIC_OP(BoolOp<1>, Or);

} // namespace sot
} // namespace dynamicgraph

/* --- TODO ------------------------------------------------------------------*/
// The following commented lines are sot-v1 entities that are still waiting
//   for conversion. Help yourself!

// /* --------------------------------------------------------------------------
// */

// struct WeightedDirection
// {
// public:
//   void operator()( const dynamicgraph::Vector& v1,const dynamicgraph::Vector&
//   v2,dynamicgraph::Vector& res ) const
//   {
//     const double norm1 = v1.norm();
//     const double norm2 = v2.norm();
//     res=v2; res*=norm1;
//     res*= (1/norm2);
//   }
// };
// typedef BinaryOp< Vector,Vector,Vector,WeightedDirection > weightdir;
// SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_ExE_E(weightdir,vector,weight_dir,"WeightDir")

// /* --------------------------------------------------------------------------
// */

// struct Nullificator
// {
// public:
//   void operator()( const dynamicgraph::Vector& v1,const dynamicgraph::Vector&
//   v2,dynamicgraph::Vector& res ) const
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

// /* --------------------------------------------------------------------------
// */

// struct VirtualSpring
// {
// public:
//   double spring;

//   void operator()( const dynamicgraph::Vector& pos,const
//   dynamicgraph::Vector& ref,dynamicgraph::Vector& res ) const
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
