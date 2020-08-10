/// Copyright CNRS 2019
/// author: O. Stasse
#include "../../src/matrix/operator.cpp"
#include <iostream>

namespace dg = ::dynamicgraph;
using namespace dynamicgraph::sot;

#define BOOST_TEST_MODULE test - operator

#include <boost/test/output_test_stream.hpp>
#include <boost/test/unit_test.hpp>

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE(test_vector_selecter) {
  // Test VectorSelecter registered as aSelec_of_vector
  VectorSelecter aSelec_of_vector;

  output_test_stream output;

  output << aSelec_of_vector.nameTypeIn();
  BOOST_CHECK(output.is_equal("Vector"));

  output << aSelec_of_vector.nameTypeOut();
  BOOST_CHECK(output.is_equal("Vector"));

  output << aSelec_of_vector.getDocString();
  BOOST_CHECK(output.is_equal("Undocumented unary operator\n"
                              "  - input  Vector\n"
                              "  - output Vector\n"));
  dg::Vector vIn(10), vOut(10);
  for (unsigned int i = 0; i < 10; i++)
    vIn(i) = i;

  aSelec_of_vector.setBounds(3, 5);
  aSelec_of_vector.addBounds(7, 10);
  aSelec_of_vector(vIn, vOut);
  output << vOut;

  BOOST_CHECK(output.is_equal("3\n4\n7\n8\n9"));

  output << dg::sot::UnaryOp<VectorSelecter>::CLASS_NAME;
  BOOST_CHECK(output.is_equal("Selec_of_vector"));

  dg::Entity *anEntity = regFunction_Selec_of_vector("test_Selec_of_vector");
  dg::sot::UnaryOp<VectorSelecter> *aVectorSelecter =
      dynamic_cast<dg::sot::UnaryOp<VectorSelecter> *>(anEntity);
  output << aVectorSelecter->getTypeInName();
  BOOST_CHECK(output.is_equal("Vector"));

  output << aVectorSelecter->getTypeOutName();
  BOOST_CHECK(output.is_equal("Vector"));

  output << aVectorSelecter->getClassName();
  BOOST_CHECK(output.is_equal("Selec_of_vector"));

  output << aVectorSelecter->getDocString();
  BOOST_CHECK(output.is_equal("Undocumented unary operator\n"
                              "  - input  Vector\n"
                              "  - output Vector\n"));
}

BOOST_AUTO_TEST_CASE(test_vector_component) {
  // Test Vector Component
  VectorComponent aComponent_of_vector;

  output_test_stream output;

  aComponent_of_vector.setIndex(1);
  dg::Vector vIn(3);
  for (unsigned int i = 0; i < 3; i++)
    vIn(i) = i;

  double res;
  aComponent_of_vector(vIn, res);
  BOOST_CHECK(res == 1.0);

  output << aComponent_of_vector.getDocString();
  BOOST_CHECK(output.is_equal("Select a component of a vector\n"
                              "  - input  vector\n"
                              "  - output double"));

  output << aComponent_of_vector.nameTypeIn();
  BOOST_CHECK(output.is_equal("Vector"));
  output << aComponent_of_vector.nameTypeOut();
  BOOST_CHECK(output.is_equal("double"));

  dg::Entity *anEntity =
      regFunction_Component_of_vector("test_Component_of_vector");
  dg::sot::UnaryOp<VectorComponent> *aVectorSelecter =
      dynamic_cast<dg::sot::UnaryOp<VectorComponent> *>(anEntity);
  output << aVectorSelecter->getTypeInName();
  BOOST_CHECK(output.is_equal("Vector"));

  output << aVectorSelecter->getTypeOutName();
  BOOST_CHECK(output.is_equal("double"));

  output << aVectorSelecter->getClassName();
  BOOST_CHECK(output.is_equal("Component_of_vector"));

  output << aVectorSelecter->getDocString();
  BOOST_CHECK(output.is_equal("Select a component of a vector\n"
                              "  - input  vector\n"
                              "  - output double"));
}

BOOST_AUTO_TEST_CASE(test_matrix_selector) {
  MatrixSelector aSelec_of_matrix;
  output_test_stream output;

  aSelec_of_matrix.setBoundsRow(2, 4);
  aSelec_of_matrix.setBoundsCol(2, 4);

  dg::Matrix aMatrix(5, 5);
  for (unsigned int i = 0; i < 5; i++)
    for (unsigned int j = 0; j < 5; j++)
      aMatrix(i, j) = i * 5 + j;

  dg::Matrix resMatrix(2, 2);
  aSelec_of_matrix(aMatrix, resMatrix);
  BOOST_CHECK(resMatrix(0, 0) == 12.0);
  BOOST_CHECK(resMatrix(0, 1) == 13.0);
  BOOST_CHECK(resMatrix(1, 0) == 17.0);
  BOOST_CHECK(resMatrix(1, 1) == 18.0);

  output << aSelec_of_matrix.nameTypeIn();
  BOOST_CHECK(output.is_equal("Matrix"));
  output << aSelec_of_matrix.nameTypeOut();
  BOOST_CHECK(output.is_equal("Matrix"));

  dg::Entity *anEntity = regFunction_Selec_of_matrix("test_Selec_of_matrix");
  dg::sot::UnaryOp<MatrixSelector> *aMatrixSelector =
      dynamic_cast<dg::sot::UnaryOp<MatrixSelector> *>(anEntity);
  output << aMatrixSelector->getTypeInName();
  BOOST_CHECK(output.is_equal("Matrix"));

  output << aMatrixSelector->getTypeOutName();
  BOOST_CHECK(output.is_equal("Matrix"));

  output << aMatrixSelector->getClassName();
  BOOST_CHECK(output.is_equal("Selec_of_matrix"));
}

BOOST_AUTO_TEST_SUITE(test_rotation_conversions)

template <typename type> type random();
template <> VectorRollPitchYaw random<VectorRollPitchYaw>() {
  return VectorRollPitchYaw::Random();
}
template <> VectorQuaternion random<VectorQuaternion>() {
  return VectorQuaternion(Eigen::Vector4d::Random().normalized());
}
template <> MatrixRotation random<MatrixRotation>() {
  return MatrixRotation(random<VectorQuaternion>());
}
template <> MatrixHomogeneous random<MatrixHomogeneous>() {
  MatrixHomogeneous matrix_homo;
  matrix_homo.translation() = Eigen::Vector3d::Random();
  matrix_homo.linear() = random<MatrixRotation>();
  return matrix_homo;
}

template <typename type> bool compare(const type &a, const type &b) {
  return a.isApprox(b);
}
template <>
bool compare<VectorQuaternion>(const VectorQuaternion &a,
                               const VectorQuaternion &b) {
  return a.isApprox(b) || a.coeffs().isApprox(-b.coeffs());
}

template <typename AtoB, typename BtoA> void test_impl() {
  typedef typename AtoB::Tin A;
  typedef typename AtoB::Tout B;

  AtoB a2b;
  BtoA b2a;

  for (std::size_t i = 1; i < 10; ++i) {
    A ain = random<A>(), aout;
    B b;

    a2b(ain, b);
    b2a(b, aout);

    BOOST_CHECK(compare(ain, aout));
  }
}

BOOST_AUTO_TEST_CASE(matrix_rpy) { test_impl<MatrixToRPY, RPYToMatrix>(); }
BOOST_AUTO_TEST_CASE(quaternion_rpy) {
  test_impl<QuaternionToRPY, RPYToQuaternion>();
}
BOOST_AUTO_TEST_CASE(matrix_quaternion) {
  test_impl<MatrixToQuaternion, QuaternionToMatrix>();
}
BOOST_AUTO_TEST_CASE(matrixHomo_poseQuaternion) {
  test_impl<MatrixHomoToPoseQuaternion, PoseQuaternionToMatrixHomo>();
}
BOOST_AUTO_TEST_CASE(matrixHomo_se3Vector) {
  test_impl<HomogeneousMatrixToSE3Vector, SE3VectorToHomogeneousMatrix>();
}

BOOST_AUTO_TEST_SUITE_END()
