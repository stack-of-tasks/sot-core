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

#include <boost/function.hpp>

#include <sot/core/binary-op.hh>
#include <sot/core/unary-op.hh>
#include <sot/core/variadic-op.hh>

#include <sot/core/matrix-geometry.hh>

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <boost/numeric/conversion/cast.hpp>
#include <deque>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/debug.hh>
#include <sot/core/factory.hh>

#include "../tools/type-name-helper.hh"

namespace dg = ::dynamicgraph;

/* ---------------------------------------------------------------------------*/
/* ------- GENERIC HELPERS -------------------------------------------------- */
/* ---------------------------------------------------------------------------*/

#define ADD_COMMAND(name, def) commandMap.insert(std::make_pair(name, def))

namespace dynamicgraph {
namespace sot {
template <typename TypeIn, typename TypeOut> struct UnaryOpHeader {
  typedef TypeIn Tin;
  typedef TypeOut Tout;
  static inline std::string nameTypeIn(void) {
    return TypeNameHelper<Tin>::typeName();
  }
  static inline std::string nameTypeOut(void) {
    return TypeNameHelper<Tout>::typeName();
  }
  inline void addSpecificCommands(Entity &, Entity::CommandMap_t &) {}
  inline std::string getDocString() const {
    return std::string("Undocumented unary operator\n"
                       "  - input  ") +
           nameTypeIn() +
           std::string("\n"
                       "  - output ") +
           nameTypeOut() + std::string("\n");
  }
};

/* ---------------------------------------------------------------------- */
/* --- ALGEBRA SELECTORS ------------------------------------------------ */
/* ---------------------------------------------------------------------- */
struct VectorSelecter : public UnaryOpHeader<dg::Vector, dg::Vector> {
  inline void operator()(const Tin &m, Vector &res) const {
    res.resize(size);
    Vector::Index r = 0;
    for (std::size_t i = 0; i < idxs.size(); ++i) {
      const Vector::Index &R = idxs[i].first;
      const Vector::Index &nr = idxs[i].second;
      assert((nr >= 0) && (R + nr <= m.size()));
      res.segment(r, nr) = m.segment(R, nr);
      r += nr;
    }
    assert(r == size);
  }

  typedef std::pair<Vector::Index, Vector::Index> segment_t;
  typedef std::vector<segment_t> segments_t;
  segments_t idxs;
  Vector::Index size;

  inline void setBounds(const int &m, const int &M) {
    idxs = segments_t(1, segment_t(m, M - m));
    size = M - m;
  }
  inline void addBounds(const int &m, const int &M) {
    idxs.push_back(segment_t(m, M - m));
    size += M - m;
  }

  inline void addSpecificCommands(Entity &ent,
                                  Entity::CommandMap_t &commandMap) {
    using namespace dynamicgraph::command;
    std::string doc;

    boost::function<void(const int &, const int &)> setBound =
        boost::bind(&VectorSelecter::setBounds, this, _1, _2);
    doc = docCommandVoid2("Set the bound of the selection [m,M[.", "int (min)",
                          "int (max)");
    ADD_COMMAND("selec", makeCommandVoid2(ent, setBound, doc));
    boost::function<void(const int &, const int &)> addBound =
        boost::bind(&VectorSelecter::addBounds, this, _1, _2);
    doc = docCommandVoid2("Add a segment to be selected [m,M[.", "int (min)",
                          "int (max)");
    ADD_COMMAND("addSelec", makeCommandVoid2(ent, addBound, doc));
  }
  VectorSelecter() : size(0) {}
};

/* ---------------------------------------------------------------------- */
struct VectorComponent : public UnaryOpHeader<dg::Vector, double> {
  inline void operator()(const Tin &m, double &res) const {
    assert(index < m.size());
    res = m(index);
  }

  int index;
  inline void setIndex(const int &m) { index = m; }

  inline void addSpecificCommands(Entity &ent,
                                  Entity::CommandMap_t &commandMap) {
    std::string doc;

    boost::function<void(const int &)> callback =
        boost::bind(&VectorComponent::setIndex, this, _1);
    doc = command::docCommandVoid1("Set the index of the component.",
                                   "int (index)");
    ADD_COMMAND("setIndex", command::makeCommandVoid1(ent, callback, doc));
  }
  inline std::string getDocString() const {
    std::string docString("Select a component of a vector\n"
                          "  - input  vector\n"
                          "  - output double");
    return docString;
  }
};

/* ---------------------------------------------------------------------- */
struct MatrixSelector : public UnaryOpHeader<dg::Matrix, dg::Matrix> {
  inline void operator()(const Matrix &m, Matrix &res) const {
    assert((imin <= imax) && (imax <= m.rows()));
    assert((jmin <= jmax) && (jmax <= m.cols()));
    res.resize(imax - imin, jmax - jmin);
    for (int i = imin; i < imax; ++i)
      for (int j = jmin; j < jmax; ++j)
        res(i - imin, j - jmin) = m(i, j);
  }

public:
  int imin, imax;
  int jmin, jmax;

  inline void setBoundsRow(const int &m, const int &M) {
    imin = m;
    imax = M;
  }
  inline void setBoundsCol(const int &m, const int &M) {
    jmin = m;
    jmax = M;
  }

  inline void addSpecificCommands(Entity &ent,
                                  Entity::CommandMap_t &commandMap) {
    using namespace dynamicgraph::command;
    std::string doc;

    boost::function<void(const int &, const int &)> setBoundsRow =
        boost::bind(&MatrixSelector::setBoundsRow, this, _1, _2);
    boost::function<void(const int &, const int &)> setBoundsCol =
        boost::bind(&MatrixSelector::setBoundsCol, this, _1, _2);

    doc = docCommandVoid2("Set the bound on rows.", "int (min)", "int (max)");
    ADD_COMMAND("selecRows", makeCommandVoid2(ent, setBoundsRow, doc));

    doc = docCommandVoid2("Set the bound on cols [m,M[.", "int (min)",
                          "int (max)");
    ADD_COMMAND("selecCols", makeCommandVoid2(ent, setBoundsCol, doc));
  }
};

/* ---------------------------------------------------------------------- */
struct MatrixColumnSelector : public UnaryOpHeader<dg::Matrix, dg::Vector> {
public:
  inline void operator()(const Tin &m, Tout &res) const {
    assert((imin <= imax) && (imax <= m.rows()));
    assert(jcol < m.cols());

    res.resize(imax - imin);
    for (int i = imin; i < imax; ++i)
      res(i - imin) = m(i, jcol);
  }

  int imin, imax;
  int jcol;
  inline void selectCol(const int &m) { jcol = m; }
  inline void setBoundsRow(const int &m, const int &M) {
    imin = m;
    imax = M;
  }

  inline void addSpecificCommands(Entity &ent,
                                  Entity::CommandMap_t &commandMap) {
    using namespace dynamicgraph::command;
    std::string doc;

    boost::function<void(const int &, const int &)> setBoundsRow =
        boost::bind(&MatrixColumnSelector::setBoundsRow, this, _1, _2);
    boost::function<void(const int &)> selectCol =
        boost::bind(&MatrixColumnSelector::selectCol, this, _1);

    doc = docCommandVoid2("Set the bound on rows.", "int (min)", "int (max)");
    ADD_COMMAND("selecRows", makeCommandVoid2(ent, setBoundsRow, doc));

    doc = docCommandVoid1("Select the col to copy.", "int (col index)");
    ADD_COMMAND("selecCols", makeCommandVoid1(ent, selectCol, doc));
  }
};

/* ---------------------------------------------------------------------- */
struct MatrixTranspose : public UnaryOpHeader<dg::Matrix, dg::Matrix> {
  inline void operator()(const Tin &m, Tout &res) const { res = m.transpose(); }
};

/* ---------------------------------------------------------------------- */
struct Diagonalizer : public UnaryOpHeader<Vector, Matrix> {
  inline void operator()(const dg::Vector &r, dg::Matrix &res) {
    res = r.asDiagonal();
  }

public:
  Diagonalizer(void) : nbr(0), nbc(0) {}
  unsigned int nbr, nbc;
  inline void resize(const int &r, const int &c) {
    nbr = r;
    nbc = c;
  }
  inline void addSpecificCommands(Entity &ent,
                                  Entity::CommandMap_t &commandMap) {
    using namespace dynamicgraph::command;
    std::string doc;

    boost::function<void(const int &, const int &)> resize =
        boost::bind(&Diagonalizer::resize, this, _1, _2);

    doc = docCommandVoid2("Set output size.", "int (row)", "int (col)");
    ADD_COMMAND("resize", makeCommandVoid2(ent, resize, doc));
  }
};

/* ---------------------------------------------------------------------- */
/* --- INVERSION -------------------------------------------------------- */
/* ---------------------------------------------------------------------- */

template <typename matrixgen>
struct Inverser : public UnaryOpHeader<matrixgen, matrixgen> {
  typedef typename UnaryOpHeader<matrixgen, matrixgen>::Tin Tin;
  typedef typename UnaryOpHeader<matrixgen, matrixgen>::Tout Tout;
  inline void operator()(const Tin &m, Tout &res) const { res = m.inverse(); }
};

struct Normalize : public UnaryOpHeader<dg::Vector, double> {
  inline void operator()(const dg::Vector &m, double &res) const {
    res = m.norm();
  }

  inline std::string getDocString() const {
    std::string docString("Computes the norm of a vector\n"
                          "  - input  vector\n"
                          "  - output double");
    return docString;
  }
};

struct InverserRotation : public UnaryOpHeader<MatrixRotation, MatrixRotation> {
  inline void operator()(const Tin &m, Tout &res) const { res = m.transpose(); }
};

struct InverserQuaternion
    : public UnaryOpHeader<VectorQuaternion, VectorQuaternion> {
  inline void operator()(const Tin &m, Tout &res) const { res = m.conjugate(); }
};

/* ----------------------------------------------------------------------- */
/* --- SE3/SO3 conversions ----------------------------------------------- */
/* ----------------------------------------------------------------------- */

struct MatrixHomoToPoseUTheta
    : public UnaryOpHeader<MatrixHomogeneous, dg::Vector> {
  inline void operator()(const MatrixHomogeneous &M, dg::Vector &res) {
    res.resize(6);
    VectorUTheta r(M.linear());
    res.head<3>() = M.translation();
    res.tail<3>() = r.angle() * r.axis();
  }
};

struct SkewSymToVector : public UnaryOpHeader<Matrix, Vector> {
  inline void operator()(const Matrix &M, Vector &res) {
    res.resize(3);
    res(0) = M(7);
    res(1) = M(2);
    res(2) = M(3);
  }
};

struct PoseUThetaToMatrixHomo
    : public UnaryOpHeader<Vector, MatrixHomogeneous> {
  inline void operator()(const dg::Vector &v, MatrixHomogeneous &res) {
    assert(v.size() >= 6);
    res.translation() = v.head<3>();
    double theta = v.tail<3>().norm();
    if (theta > 0)
      res.linear() = Eigen::AngleAxisd(theta, v.tail<3>() / theta).matrix();
    else
      res.linear().setIdentity();
  }
};

struct SE3VectorToMatrixHomo
    : public UnaryOpHeader<dg::Vector, MatrixHomogeneous> {
  void operator()(const dg::Vector &vect, MatrixHomogeneous &Mres) {
    Mres.translation() = vect.head<3>();
    Mres.linear().row(0) = vect.segment(3, 3);
    Mres.linear().row(1) = vect.segment(6, 3);
    Mres.linear().row(2) = vect.segment(9, 3);
  }
};

struct MatrixHomoToSE3Vector
    : public UnaryOpHeader<MatrixHomogeneous, dg::Vector> {
  void operator()(const MatrixHomogeneous &M, dg::Vector &res) {
    res.resize(12);
    res.head<3>() = M.translation();
    res.segment(3, 3) = M.linear().row(0);
    res.segment(6, 3) = M.linear().row(1);
    res.segment(9, 3) = M.linear().row(2);
  }
};

struct PoseQuaternionToMatrixHomo
    : public UnaryOpHeader<Vector, MatrixHomogeneous> {
  void operator()(const dg::Vector &vect, MatrixHomogeneous &Mres) {
    Mres.translation() = vect.head<3>();
    Mres.linear() = VectorQuaternion(vect.tail<4>()).toRotationMatrix();
  }
};

struct MatrixHomoToPoseQuaternion
    : public UnaryOpHeader<MatrixHomogeneous, Vector> {
  inline void operator()(const MatrixHomogeneous &M, Vector &res) {
    res.resize(7);
    res.head<3>() = M.translation();
    Eigen::Map<VectorQuaternion> q(res.tail<4>().data());
    q = M.linear();
  }
};

struct MatrixHomoToPoseRollPitchYaw
    : public UnaryOpHeader<MatrixHomogeneous, Vector> {
  inline void operator()(const MatrixHomogeneous &M, dg::Vector &res) {
    VectorRollPitchYaw r = (M.linear().eulerAngles(2, 1, 0)).reverse();
    dg::Vector t(3);
    t = M.translation();
    res.resize(6);
    for (unsigned int i = 0; i < 3; ++i)
      res(i) = t(i);
    for (unsigned int i = 0; i < 3; ++i)
      res(i + 3) = r(i);
  }
};

struct PoseRollPitchYawToMatrixHomo
    : public UnaryOpHeader<Vector, MatrixHomogeneous> {
  inline void operator()(const dg::Vector &vect, MatrixHomogeneous &Mres) {

    VectorRollPitchYaw r;
    for (unsigned int i = 0; i < 3; ++i)
      r(i) = vect(i + 3);
    MatrixRotation R = (Eigen::AngleAxisd(r(2), Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(r(1), Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(r(0), Eigen::Vector3d::UnitX()))
                           .toRotationMatrix();

    dg::Vector t(3);
    for (unsigned int i = 0; i < 3; ++i)
      t(i) = vect(i);

    // buildFrom(R,t);
    Mres = Eigen::Translation3d(t) * R;
  }
};

struct PoseRollPitchYawToPoseUTheta : public UnaryOpHeader<Vector, Vector> {
  inline void operator()(const dg::Vector &vect, dg::Vector &vectres) {
    VectorRollPitchYaw r;
    for (unsigned int i = 0; i < 3; ++i)
      r(i) = vect(i + 3);
    MatrixRotation R = (Eigen::AngleAxisd(r(2), Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(r(1), Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(r(0), Eigen::Vector3d::UnitX()))
                           .toRotationMatrix();

    VectorUTheta rrot(R);

    vectres.resize(6);
    for (unsigned int i = 0; i < 3; ++i) {
      vectres(i) = vect(i);
      vectres(i + 3) = rrot.angle() * rrot.axis()(i);
    }
  }
};

struct HomoToMatrix : public UnaryOpHeader<MatrixHomogeneous, Matrix> {
  inline void operator()(const MatrixHomogeneous &M, dg::Matrix &res) {
    res = M.matrix();
  }
};

struct MatrixToHomo : public UnaryOpHeader<Matrix, MatrixHomogeneous> {
  inline void operator()(const Eigen::Matrix<double, 4, 4> &M,
                         MatrixHomogeneous &res) {
    res = M;
  }
};

struct HomoToTwist : public UnaryOpHeader<MatrixHomogeneous, MatrixTwist> {
  inline void operator()(const MatrixHomogeneous &M, MatrixTwist &res) {
    Eigen::Vector3d _t = M.translation();
    MatrixRotation R(M.linear());
    Eigen::Matrix3d Tx;
    Tx << 0, -_t(2), _t(1), _t(2), 0, -_t(0), -_t(1), _t(0), 0;

    Eigen::Matrix3d sk;
    sk = Tx * R;
    res.block<3, 3>(0, 0) = R;
    res.block<3, 3>(0, 3) = sk;
    res.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    res.block<3, 3>(3, 3) = R;
  }
};

struct HomoToRotation
    : public UnaryOpHeader<MatrixHomogeneous, MatrixRotation> {
  inline void operator()(const MatrixHomogeneous &M, MatrixRotation &res) {
    res = M.linear();
  }
};

struct MatrixHomoToPose : public UnaryOpHeader<MatrixHomogeneous, Vector> {
  inline void operator()(const MatrixHomogeneous &M, Vector &res) {
    res.resize(3);
    res = M.translation();
  }
};

struct RPYToMatrix : public UnaryOpHeader<VectorRollPitchYaw, MatrixRotation> {
  inline void operator()(const VectorRollPitchYaw &r, MatrixRotation &res) {
    res = (Eigen::AngleAxisd(r(2), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(r(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(r(0), Eigen::Vector3d::UnitX()))
              .toRotationMatrix();
  }
};

struct MatrixToRPY : public UnaryOpHeader<MatrixRotation, VectorRollPitchYaw> {
  inline void operator()(const MatrixRotation &r, VectorRollPitchYaw &res) {
    res = (r.eulerAngles(2, 1, 0)).reverse();
  }
};

struct RPYToQuaternion
    : public UnaryOpHeader<VectorRollPitchYaw, VectorQuaternion> {
  inline void operator()(const VectorRollPitchYaw &r, VectorQuaternion &res) {
    res = (Eigen::AngleAxisd(r(2), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(r(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(r(0), Eigen::Vector3d::UnitX()))
              .toRotationMatrix();
  }
};

struct QuaternionToRPY
    : public UnaryOpHeader<VectorQuaternion, VectorRollPitchYaw> {
  inline void operator()(const VectorQuaternion &r, VectorRollPitchYaw &res) {
    res = (r.toRotationMatrix().eulerAngles(2, 1, 0)).reverse();
  }
};

struct QuaternionToMatrix
    : public UnaryOpHeader<VectorQuaternion, MatrixRotation> {
  inline void operator()(const VectorQuaternion &r, MatrixRotation &res) {
    res = r.toRotationMatrix();
  }
};

struct MatrixToQuaternion
    : public UnaryOpHeader<MatrixRotation, VectorQuaternion> {
  inline void operator()(const MatrixRotation &r, VectorQuaternion &res) {
    res = r;
  }
};

struct MatrixToUTheta : public UnaryOpHeader<MatrixRotation, VectorUTheta> {
  inline void operator()(const MatrixRotation &r, VectorUTheta &res) {
    res = r;
  }
};

struct UThetaToQuaternion
    : public UnaryOpHeader<VectorUTheta, VectorQuaternion> {
  inline void operator()(const VectorUTheta &r, VectorQuaternion &res) {
    res = r;
  }
};

template <typename TypeIn1, typename TypeIn2, typename TypeOut>
struct BinaryOpHeader {
  typedef TypeIn1 Tin1;
  typedef TypeIn2 Tin2;
  typedef TypeOut Tout;
  inline static std::string nameTypeIn1(void) {
    return TypeNameHelper<Tin1>::typeName();
  }
  inline static std::string nameTypeIn2(void) {
    return TypeNameHelper<Tin2>::typeName();
  }
  inline static std::string nameTypeOut(void) {
    return TypeNameHelper<Tout>::typeName();
  }
  inline void addSpecificCommands(Entity &, Entity::CommandMap_t &) {}
  inline std::string getDocString() const {
    return std::string("Undocumented binary operator\n"
                       "  - input  ") +
           nameTypeIn1() +
           std::string("\n"
                       "  -        ") +
           nameTypeIn2() +
           std::string("\n"
                       "  - output ") +
           nameTypeOut() + std::string("\n");
  }
};

} /* namespace sot */
} /* namespace dynamicgraph */

/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/

namespace dynamicgraph {
namespace sot {

/* --- MULTIPLICATION --------------------------------------------------- */

template <typename F, typename E>
struct Multiplier_FxE__E : public BinaryOpHeader<F, E, E> {
  inline void operator()(const F &f, const E &e, E &res) const { res = f * e; }
};

template <>
inline void
Multiplier_FxE__E<dynamicgraph::sot::MatrixHomogeneous, dynamicgraph::Vector>::
operator()(const dynamicgraph::sot::MatrixHomogeneous &f,
           const dynamicgraph::Vector &e, dynamicgraph::Vector &res) const {
  res = f.matrix() * e;
}

template <>
inline void Multiplier_FxE__E<double, dynamicgraph::Vector>::
operator()(const double &x, const dynamicgraph::Vector &v,
           dynamicgraph::Vector &res) const {
  res = v;
  res *= x;
}

typedef Multiplier_FxE__E<double, dynamicgraph::Vector>
    Multiplier_double_vector;
typedef Multiplier_FxE__E<dynamicgraph::Matrix, dynamicgraph::Vector>
    Multiplier_matrix_vector;
typedef Multiplier_FxE__E<MatrixHomogeneous, dynamicgraph::Vector>
    Multiplier_matrixHomo_vector;
typedef Multiplier_FxE__E<MatrixTwist, dynamicgraph::Vector>
    Multiplier_matrixTwist_vector;

/* --- SUBSTRACTION ----------------------------------------------------- */
template <typename T> struct Substraction : public BinaryOpHeader<T, T, T> {
  inline void operator()(const T &v1, const T &v2, T &r) const {
    r = v1;
    r -= v2;
  }
};

/* --- STACK ------------------------------------------------------------ */
struct VectorStack
    : public BinaryOpHeader<dynamicgraph::Vector, dynamicgraph::Vector,
                            dynamicgraph::Vector> {
public:
  int v1min, v1max;
  int v2min, v2max;
  inline void operator()(const dynamicgraph::Vector &v1,
                         const dynamicgraph::Vector &v2,
                         dynamicgraph::Vector &res) const {
    assert((v1max >= v1min) && (v1.size() >= v1max));
    assert((v2max >= v2min) && (v2.size() >= v2max));

    const int v1size = v1max - v1min, v2size = v2max - v2min;
    res.resize(v1size + v2size);
    for (int i = 0; i < v1size; ++i) {
      res(i) = v1(i + v1min);
    }
    for (int i = 0; i < v2size; ++i) {
      res(v1size + i) = v2(i + v2min);
    }
  }

  inline void selec1(const int &m, const int M) {
    v1min = m;
    v1max = M;
  }
  inline void selec2(const int &m, const int M) {
    v2min = m;
    v2max = M;
  }

  inline void addSpecificCommands(Entity &ent,
                                  Entity::CommandMap_t &commandMap) {
    using namespace dynamicgraph::command;
    std::string doc;

    boost::function<void(const int &, const int &)> selec1 =
        boost::bind(&VectorStack::selec1, this, _1, _2);
    boost::function<void(const int &, const int &)> selec2 =
        boost::bind(&VectorStack::selec2, this, _1, _2);

    ADD_COMMAND(
        "selec1",
        makeCommandVoid2(ent, selec1,
                         docCommandVoid2("set the min and max of selection.",
                                         "int (imin)", "int (imax)")));
    ADD_COMMAND(
        "selec2",
        makeCommandVoid2(ent, selec2,
                         docCommandVoid2("set the min and max of selection.",
                                         "int (imin)", "int (imax)")));
  }
};

/* ---------------------------------------------------------------------- */

struct Composer
    : public BinaryOpHeader<dynamicgraph::Matrix, dynamicgraph::Vector,
                            MatrixHomogeneous> {
  inline void operator()(const dynamicgraph::Matrix &R,
                         const dynamicgraph::Vector &t,
                         MatrixHomogeneous &H) const {
    H.linear() = R;
    H.translation() = t;
  }
};

/* --- CONVOLUTION PRODUCT ---------------------------------------------- */
struct ConvolutionTemporal
    : public BinaryOpHeader<dynamicgraph::Vector, dynamicgraph::Matrix,
                            dynamicgraph::Vector> {
  typedef std::deque<dynamicgraph::Vector> MemoryType;
  MemoryType memory;

  inline void convolution(const MemoryType &f1, const dynamicgraph::Matrix &f2,
                          dynamicgraph::Vector &res) {
    const Vector::Index nconv = (Vector::Index)f1.size(), nsig = f2.rows();
    sotDEBUG(15) << "Size: " << nconv << "x" << nsig << std::endl;
    if (nconv > f2.cols())
      return; // TODO: error, this should not happen

    res.resize(nsig);
    res.fill(0);
    unsigned int j = 0;
    for (MemoryType::const_iterator iter = f1.begin(); iter != f1.end();
         iter++) {
      const dynamicgraph::Vector &s_tau = *iter;
      sotDEBUG(45) << "Sig" << j << ": " << s_tau;
      if (s_tau.size() != nsig)
        return; // TODO: error throw;
      for (int i = 0; i < nsig; ++i) {
        res(i) += f2(i, j) * s_tau(i);
      }
      j++;
    }
  }
  inline void operator()(const dynamicgraph::Vector &v1,
                         const dynamicgraph::Matrix &m2,
                         dynamicgraph::Vector &res) {
    memory.push_front(v1);
    while ((Vector::Index)memory.size() > m2.cols())
      memory.pop_back();
    convolution(memory, m2, res);
  }
};

/* --- BOOLEAN REDUCTION ------------------------------------------------ */

template <typename T> struct Comparison : public BinaryOpHeader<T, T, bool> {
  inline void operator()(const T &a, const T &b, bool &res) const {
    res = (a < b);
  }
  inline std::string getDocString() const {
    typedef BinaryOpHeader<T, T, bool> Base;
    return std::string("Comparison of inputs:\n"
                       "  - input  ") +
           Base::nameTypeIn1() +
           std::string("\n"
                       "  -        ") +
           Base::nameTypeIn2() +
           std::string("\n"
                       "  - output ") +
           Base::nameTypeOut() +
           std::string("\n"
                       "  sout = ( sin1 < sin2 )\n");
  }
};

template <typename T1, typename T2 = T1>
struct MatrixComparison : public BinaryOpHeader<T1, T2, bool> {
  // TODO T1 or T2 could be a scalar type.
  inline void operator()(const T1 &a, const T2 &b, bool &res) const {
    if (equal && any)
      res = (a.array() <= b.array()).any();
    else if (equal && !any)
      res = (a.array() <= b.array()).all();
    else if (!equal && any)
      res = (a.array() < b.array()).any();
    else if (!equal && !any)
      res = (a.array() < b.array()).all();
  }
  inline std::string getDocString() const {
    typedef BinaryOpHeader<T1, T2, bool> Base;
    return std::string("Comparison of inputs:\n"
                       "  - input  ") +
           Base::nameTypeIn1() +
           std::string("\n"
                       "  -        ") +
           Base::nameTypeIn2() +
           std::string("\n"
                       "  - output ") +
           Base::nameTypeOut() +
           std::string("\n"
                       "  sout = ( sin1 < sin2 ).op()\n") +
           std::string("\n"
                       "  where op is either any (default) or all. The "
                       "comparison can be made <=.\n");
  }
  MatrixComparison() : any(true), equal(false) {}
  inline void addSpecificCommands(Entity &ent,
                                  Entity::CommandMap_t &commandMap) {
    using namespace dynamicgraph::command;
    ADD_COMMAND(
        "setTrueIfAny",
        makeDirectSetter(ent, &any, docDirectSetter("trueIfAny", "bool")));
    ADD_COMMAND(
        "getTrueIfAny",
        makeDirectGetter(ent, &any, docDirectGetter("trueIfAny", "bool")));
    ADD_COMMAND("setEqual", makeDirectSetter(ent, &equal,
                                             docDirectSetter("equal", "bool")));
    ADD_COMMAND("getEqual", makeDirectGetter(ent, &equal,
                                             docDirectGetter("equal", "bool")));
  }
  bool any, equal;
};

} /* namespace sot */
} /* namespace dynamicgraph */

namespace dynamicgraph {
namespace sot {

template <typename T> struct WeightedAdder : public BinaryOpHeader<T, T, T> {
public:
  double gain1, gain2;
  inline void operator()(const T &v1, const T &v2, T &res) const {
    res = v1;
    res *= gain1;
    res += gain2 * v2;
  }

  inline void addSpecificCommands(Entity &ent,
                                  Entity::CommandMap_t &commandMap) {
    using namespace dynamicgraph::command;
    std::string doc;

    ADD_COMMAND(
        "setGain1",
        makeDirectSetter(ent, &gain1, docDirectSetter("gain1", "double")));
    ADD_COMMAND(
        "setGain2",
        makeDirectSetter(ent, &gain2, docDirectSetter("gain2", "double")));
    ADD_COMMAND(
        "getGain1",
        makeDirectGetter(ent, &gain1, docDirectGetter("gain1", "double")));
    ADD_COMMAND(
        "getGain2",
        makeDirectGetter(ent, &gain2, docDirectGetter("gain2", "double")));
  }

  inline std::string getDocString() const {
    return std::string("Weighted Combination of inputs : \n - gain{1|2} gain.");
  }
};

} // namespace sot
} // namespace dynamicgraph

namespace dynamicgraph {
namespace sot {
template <typename Tin, typename Tout, typename Time>
std::string VariadicAbstract<Tin, Tout, Time>::getTypeInName(void) {
  return TypeNameHelper<Tin>::typeName();
}
template <typename Tin, typename Tout, typename Time>
std::string VariadicAbstract<Tin, Tout, Time>::getTypeOutName(void) {
  return TypeNameHelper<Tout>::typeName();
}

template <typename TypeIn, typename TypeOut> struct VariadicOpHeader {
  typedef TypeIn Tin;
  typedef TypeOut Tout;
  inline static std::string nameTypeIn(void) {
    return TypeNameHelper<Tin>::typeName();
  }
  inline static std::string nameTypeOut(void) {
    return TypeNameHelper<Tout>::typeName();
  }
  template <typename Op>
  inline void initialize(VariadicOp<Op> *, Entity::CommandMap_t &) {}
  inline void updateSignalNumber(const int &) {}
  inline std::string getDocString() const {
    return std::string("Undocumented variadic operator\n"
                       "  - input  " +
                       nameTypeIn() +
                       "\n"
                       "  - output " +
                       nameTypeOut() + "\n");
  }
};

/* --- VectorMix ------------------------------------------------------------ */
struct VectorMix : public VariadicOpHeader<Vector, Vector> {
public:
  typedef VariadicOp<VectorMix> Base;
  struct segment_t {
    Vector::Index index, size, input;
    std::size_t sigIdx;
    segment_t(Vector::Index i, Vector::Index s, std::size_t sig)
        : index(i), size(s), sigIdx(sig) {}
  };
  typedef std::vector<segment_t> segments_t;
  Base *entity;
  segments_t idxs;
  inline void operator()(const std::vector<const Vector *> &vs,
                         Vector &res) const {
    res = *vs[0];
    for (std::size_t i = 0; i < idxs.size(); ++i) {
      const segment_t &s = idxs[i];
      if (s.sigIdx >= vs.size())
        throw std::invalid_argument("Index out of range in VectorMix");
      res.segment(s.index, s.size) = *vs[s.sigIdx];
    }
  }

  inline void addSelec(const int &sigIdx, const int &i, const int &s) {
    idxs.push_back(segment_t(i, s, sigIdx));
  }

  inline void initialize(Base *ent, Entity::CommandMap_t &commandMap) {
    using namespace dynamicgraph::command;
    entity = ent;

    ent->addSignal("default");

    boost::function<void(const int &, const int &, const int &)> selec =
        boost::bind(&VectorMix::addSelec, this, _1, _2, _3);

    commandMap.insert(std::make_pair(
        "addSelec", makeCommandVoid3<Base, int, int, int>(
                        *ent, selec,
                        docCommandVoid3("add selection from a vector.",
                                        "int (signal index >= 1)",
                                        "int (index)", "int (size)"))));
  }
};

/* --- ADDITION --------------------------------------------------------- */
template <typename T> struct AdderVariadic : public VariadicOpHeader<T, T> {
  typedef VariadicOp<AdderVariadic> Base;

  Base *entity;
  Vector coeffs;

  AdderVariadic() : coeffs() {}
  inline void operator()(const std::vector<const T *> &vs, T &res) const {
    assert(vs.size() == (std::size_t)coeffs.size());
    if (vs.size() == 0)
      return;
    res = coeffs[0] * (*vs[0]);
    for (std::size_t i = 1; i < vs.size(); ++i)
      res += coeffs[i] * (*vs[i]);
  }

  inline void setCoeffs(const Vector &c) {
    if (entity->getSignalNumber() != c.size())
      throw std::invalid_argument("Invalid coefficient size.");
    coeffs = c;
  }
  inline void updateSignalNumber(const int &n) { coeffs = Vector::Ones(n); }

  inline void initialize(Base *ent, Entity::CommandMap_t &) {
    entity = ent;
    entity->setSignalNumber(2);
  }

  inline std::string getDocString() const {
    return "Linear combination of inputs\n"
           "  - input  " +
           VariadicOpHeader<T, T>::nameTypeIn() +
           "\n"
           "  - output " +
           VariadicOpHeader<T, T>::nameTypeOut() +
           "\n"
           "  sout = sum ([coeffs[i] * sin[i] for i in range(n) ])\n"
           "  Coefficients are set by commands, default value is 1.\n";
  }
};

/* --- MULTIPLICATION --------------------------------------------------- */
template <typename T> struct Multiplier : public VariadicOpHeader<T, T> {
  typedef VariadicOp<Multiplier> Base;

  inline void operator()(const std::vector<const T *> &vs, T &res) const {
    if (vs.size() == 0)
      setIdentity(res);
    else {
      res = *vs[0];
      for (std::size_t i = 1; i < vs.size(); ++i)
        res *= *vs[i];
    }
  }

  inline void setIdentity(T &res) const { res.setIdentity(); }

  inline void initialize(Base *ent, Entity::CommandMap_t &) {
    ent->setSignalNumber(2);
  }
};
template <> inline void Multiplier<double>::setIdentity(double &res) const {
  res = 1;
}
template <>
inline void Multiplier<MatrixHomogeneous>::
operator()(const std::vector<const MatrixHomogeneous *> &vs,
           MatrixHomogeneous &res) const {
  if (vs.size() == 0)
    setIdentity(res);
  else {
    res = *vs[0];
    for (std::size_t i = 1; i < vs.size(); ++i)
      res = res * *vs[i];
  }
}
template <>
inline void Multiplier<Vector>::
operator()(const std::vector<const Vector *> &vs, Vector &res) const {
  if (vs.size() == 0)
    res.resize(0);
  else {
    res = *vs[0];
    for (std::size_t i = 1; i < vs.size(); ++i)
      res.array() *= vs[i]->array();
  }
}

/* --- BOOLEAN --------------------------------------------------------- */
template <int operation> struct BoolOp : public VariadicOpHeader<bool, bool> {
  typedef VariadicOp<BoolOp> Base;

  inline void operator()(const std::vector<const bool *> &vs, bool &res) const {
    // TODO computation could be optimized with lazy evaluation of the
    // signals. When the output result is know, the remaining signals are
    // not computed.
    if (vs.size() == 0)
      return;
    res = *vs[0];
    for (std::size_t i = 1; i < vs.size(); ++i)
      switch (operation) {
      case 0:
        if (!res)
          return;
        res = *vs[i];
        break;
      case 1:
        if (res)
          return;
        res = *vs[i];
        break;
      }
  }
};

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
