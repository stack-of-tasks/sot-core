#include "dynamic-graph/python/module.hh"

#include "operator.hh"

namespace dg = dynamicgraph;
namespace dgs = dynamicgraph::sot;
namespace bp = boost::python;

typedef bp::return_value_policy<bp::reference_existing_object>
    reference_existing_object;

template <typename Operator> void exposeUnaryOp() {
  typedef dgs::UnaryOp<Operator> O_t;
  dg::python::exposeEntity<O_t, bp::bases<dg::Entity>,
                           dg::python::AddCommands>()
      .def_readonly("sin", &O_t::SIN)
      .def_readonly("sout", &O_t::SOUT);
}

template <typename Operator> void exposeBinaryOp() {
  typedef dgs::BinaryOp<Operator> O_t;
  dg::python::exposeEntity<O_t, bp::bases<dg::Entity>,
                           dg::python::AddCommands>()
      .def_readonly("sin1", &O_t::SIN1)
      .def_readonly("sin2", &O_t::SIN2)
      .def_readonly("sout", &O_t::SOUT);
}

template <typename Operator> auto exposeVariadicOpBase() {
  typedef dgs::VariadicOp<Operator> O_t;
  typedef typename O_t::Base B_t;
  return dg::python::exposeEntity<O_t, bp::bases<dg::Entity>,
                                  dg::python::AddCommands>()
      .def_readonly("sout", &O_t::SOUT)
      .def("sin", &B_t::getSignalIn, reference_existing_object())
      .add_property("n_sin", &B_t::getSignalNumber, &B_t::setSignalNumber,
                    "the number of input signal.")

      .def("setSignalNumber", &B_t::setSignalNumber,
           "set the number of input signal.", bp::arg("size"))
      .def("getSignalNumber", &B_t::getSignalNumber,
           "get the number of input signal.", bp::arg("size"));
}

template <typename Operator> struct exposeVariadicOpImpl {
  static void run() { exposeVariadicOpBase<Operator>(); }
};

template <typename T> struct exposeVariadicOpImpl<dgs::AdderVariadic<T> > {
  static void run() {
    typedef dgs::VariadicOp<dgs::AdderVariadic<T> > E_t;
    exposeVariadicOpBase<dgs::AdderVariadic<T> >().add_property(
        "coeffs", +[](E_t &e) { return e.op.coeffs; },
        +[](E_t &e, const dg::Vector &c) { e.op.setCoeffs(c); },
        "the multipliers.");
  }
};

template <typename Operator> void exposeVariadicOp() {
  exposeVariadicOpImpl<Operator>::run();
}

BOOST_PYTHON_MODULE(wrap) {
  using namespace dynamicgraph;
  using namespace dynamicgraph::sot;

  exposeUnaryOp<VectorSelecter>();
  exposeUnaryOp<VectorComponent>();
  exposeUnaryOp<MatrixSelector>();
  exposeUnaryOp<MatrixColumnSelector>();
  exposeUnaryOp<MatrixTranspose>();
  exposeUnaryOp<Diagonalizer>();

  /* ---------------------------------------------------------------------- */
  /* --- INVERSION -------------------------------------------------------- */
  /* ---------------------------------------------------------------------- */
  exposeUnaryOp<Inverser<Matrix> >();
  exposeUnaryOp<Inverser<MatrixHomogeneous> >();
  exposeUnaryOp<Inverser<MatrixTwist> >();
  exposeUnaryOp<Normalize>();
  exposeUnaryOp<InverserRotation>();
  exposeUnaryOp<InverserQuaternion>();

  /* ----------------------------------------------------------------------- */
  /* --- SE3/SO3 conversions ----------------------------------------------- */
  /* ----------------------------------------------------------------------- */

  exposeUnaryOp<SkewSymToVector>();
  exposeUnaryOp<PoseUThetaToMatrixHomo>();
  exposeUnaryOp<MatrixHomoToPoseUTheta>();
  exposeUnaryOp<MatrixHomoToSE3Vector>();
  exposeUnaryOp<SE3VectorToMatrixHomo>();
  exposeUnaryOp<PoseQuaternionToMatrixHomo>();
  exposeUnaryOp<MatrixHomoToPoseQuaternion>();
  exposeUnaryOp<MatrixHomoToPoseRollPitchYaw>();
  exposeUnaryOp<PoseRollPitchYawToMatrixHomo>();
  exposeUnaryOp<PoseRollPitchYawToPoseUTheta>();
  exposeUnaryOp<HomoToMatrix>();
  exposeUnaryOp<MatrixToHomo>();
  exposeUnaryOp<HomoToTwist>();
  exposeUnaryOp<HomoToRotation>();
  exposeUnaryOp<MatrixHomoToPose>();
  exposeUnaryOp<RPYToMatrix>();
  exposeUnaryOp<MatrixToRPY>();
  exposeUnaryOp<RPYToQuaternion>();
  exposeUnaryOp<QuaternionToRPY>();
  exposeUnaryOp<QuaternionToMatrix>();
  exposeUnaryOp<MatrixToQuaternion>();
  exposeUnaryOp<MatrixToUTheta>();
  exposeUnaryOp<UThetaToQuaternion>();

  /* --- MULTIPLICATION --------------------------------------------------- */

  exposeBinaryOp<Multiplier_double_vector>();
  exposeBinaryOp<Multiplier_matrix_vector>();
  exposeBinaryOp<Multiplier_matrixHomo_vector>();
  exposeBinaryOp<Multiplier_matrixTwist_vector>();

  /* --- SUBSTRACTION ----------------------------------------------------- */
  exposeBinaryOp<Substraction<dynamicgraph::Matrix> >();
  exposeBinaryOp<Substraction<dynamicgraph::Vector> >();
  exposeBinaryOp<Substraction<double> >();

  /* --- STACK ------------------------------------------------------------ */
  exposeBinaryOp<VectorStack>();

  /* ---------------------------------------------------------------------- */
  exposeBinaryOp<Composer>();

  /* --- CONVOLUTION PRODUCT ---------------------------------------------- */
  exposeBinaryOp<ConvolutionTemporal>();

  /* --- BOOLEAN REDUCTION ------------------------------------------------ */
  exposeBinaryOp<Comparison<double> >();
  exposeBinaryOp<MatrixComparison<Vector> >();

  exposeBinaryOp<WeightedAdder<dynamicgraph::Matrix> >();
  exposeBinaryOp<WeightedAdder<dynamicgraph::Vector> >();
  exposeBinaryOp<WeightedAdder<double> >();

  /* --- VectorMix ------------------------------------------------------------
   */
  exposeVariadicOp<VectorMix>();

  /* --- ADDITION --------------------------------------------------------- */
  exposeVariadicOp<AdderVariadic<Matrix> >();
  exposeVariadicOp<AdderVariadic<Vector> >();
  exposeVariadicOp<AdderVariadic<double> >();

  /* --- MULTIPLICATION --------------------------------------------------- */
  exposeVariadicOp<Multiplier<Matrix> >();
  exposeVariadicOp<Multiplier<Vector> >();
  exposeVariadicOp<Multiplier<MatrixRotation> >();
  exposeVariadicOp<Multiplier<MatrixHomogeneous> >();
  exposeVariadicOp<Multiplier<MatrixTwist> >();
  exposeVariadicOp<Multiplier<VectorQuaternion> >();
  exposeVariadicOp<Multiplier<double> >();

  /* --- BOOLEAN --------------------------------------------------------- */
  exposeVariadicOp<BoolOp<0> >();
  exposeVariadicOp<BoolOp<1> >();
}
