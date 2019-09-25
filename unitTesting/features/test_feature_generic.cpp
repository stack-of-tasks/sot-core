/*
 * Copyright 2019,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <iostream>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/sample-models.hpp>

#include <pinocchio/multibody/liegroup/liegroup.hpp>

#define BOOST_TEST_MODULE features
#include <boost/test/unit_test.hpp>

#include <sot/core/sot.hh>
#include <sot/core/feature-generic.hh>
#include <sot/core/feature-transformation.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/debug.hh>
#include <sot/core/task.hh>
#include <sot/core/gain-adaptive.hh>
#include <dynamic-graph/linear-algebra.h>
#include <Eigen/SVD>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#define EIGEN_VECTOR_IS_APPROX(Va, Vb, precision)                              \
  BOOST_CHECK_MESSAGE((Va).isApprox(Vb, precision),                            \
      "check " #Va ".isApprox(" #Vb ") failed "                                \
      "[\n" << (Va).transpose() << "\n!=\n" << (Vb).transpose() << "\n]")
#define EIGEN_MATRIX_IS_APPROX(Va, Vb, precision)                              \
  BOOST_CHECK_MESSAGE((Va).isApprox(Vb, precision),                            \
      "check " #Va ".isApprox(" #Vb ") failed "                                \
      "[\n" << (Va) << "\n!=\n" << (Vb) << "\n]")

class FeatureTestBase
{
  public:
    Task task_;
    int time_;
    dynamicgraph::Vector expectedTaskOutput_;

    FeatureTestBase(unsigned dim,const std::string &name)
      : task_("task"+name),
      time_(0)
    {
      expectedTaskOutput_.resize(dim);
    }

    virtual void init ()
    {
      task_.addFeature(featureAbstract());
      task_.setWithDerivative(true);
    }

    void computeExpectedTaskOutput (const Vector& error, const Vector& errorDrift)
    {
      double gain = task_.controlGainSIN;
      expectedTaskOutput_ = - gain * error - errorDrift;
    }

    template <typename LG_t>
    void computeExpectedTaskOutput (const Vector& s, const Vector& sdes,
        const Vector& sDesDot, const LG_t& lg)
    {
      Vector s_sd (lg.nv());
      lg.difference (sdes, s, s_sd);

      Eigen::Matrix<typename LG_t::Scalar, LG_t::NV, LG_t::NV> Jminus (lg.nv(),lg.nv());
      lg.template dDifference<pinocchio::ARG0>(sdes, s, Jminus);

      computeExpectedTaskOutput (s_sd, Jminus * sDesDot);
    }

    void checkTaskOutput ()
    {
      BOOST_REQUIRE_EQUAL (task_.taskSOUT.accessCopy().size(), expectedTaskOutput_.size());
      Vector taskOutput (expectedTaskOutput_.size());
      for( int i=0;i<expectedTaskOutput_.size(); ++i )
        taskOutput[i] = task_.taskSOUT.accessCopy()[i].getSingleBound();
      EIGEN_VECTOR_IS_APPROX(taskOutput, expectedTaskOutput_, 1e-6);
    }

    void setGain (double gain)
    {
      task_.controlGainSIN = gain;
      task_.controlGainSIN.access(time_);
      task_.controlGainSIN.setReady();
    }

    template <typename SignalType, typename ValueType>
    void setSignal (SignalType& sig, const ValueType& v)
    {
      sig = v;
      sig.access(time_);
      sig.setReady();
    }

    virtual FeatureAbstract& featureAbstract() = 0;

    virtual void setInputs () = 0;

    virtual void printInputs() {}

    virtual void recompute()
    {
      task_.taskSOUT.recompute(time_);

      // Check that recomputing went fine.
      FeatureAbstract& f (featureAbstract());
      BOOST_CHECK_EQUAL (time_, f.errorSOUT.getTime());
      BOOST_CHECK_EQUAL (time_, f.errordotSOUT.getTime());
      BOOST_CHECK_EQUAL (time_, task_.errorSOUT.getTime());
      BOOST_CHECK_EQUAL (time_, task_.errorTimeDerivativeSOUT.getTime());

      //TODO check that the output is correct
      //computeExpectedTaskOutput (s - sd, - vd);
    }

    virtual void printOutputs() {}

    virtual void runTest ()
    {
      time_++;
      setInputs();
      printInputs();
      recompute();
      printOutputs();
    }
};

class TestFeatureGeneric : public FeatureTestBase
{
  public:
    FeatureGeneric feature_,featureDes_;
    int dim_;

    TestFeatureGeneric(unsigned dim,const std::string &name):
      FeatureTestBase (dim, name),
      feature_("feature"+name),
      featureDes_("featureDes"+name),
      dim_ (dim)
    {
      feature_.setReference(&featureDes_);
      feature_.selectionSIN=Flags(true);

      dynamicgraph::Matrix Jq(dim,dim);
      Jq.setIdentity();
      feature_.jacobianSIN.setReference(&Jq);

      init();
    }

    FeatureAbstract& featureAbstract()
    {
      return feature_;
    }

    void setInputs()
    {
      dynamicgraph::Vector  s(dim_), sd(dim_), vd(dim_);
      double gain;

      switch (time_) {
        case 0:
          BOOST_TEST_MESSAGE(" ----- Test Velocity -----");
          s.setZero();
          sd.setZero();
          vd.setZero();
          gain=0.0;
          break;
        case 1:
          BOOST_TEST_MESSAGE(" ----- Test Position -----");
          s.setZero();
          sd.setConstant(2.);
          vd.setZero();
          gain=1.0;
          break;
        case 2:
          BOOST_TEST_MESSAGE(" ----- Test both -----");
          s.setZero();
          sd.setConstant(2.);
          vd.setConstant(1.);
          gain=3.0;
          break;
        default:
          s.setRandom();
          sd.setRandom();
          vd.setRandom();
          gain=1.0;
      }

      feature_.errorSIN = s;
      feature_.errorSIN.access(time_);
      feature_.errorSIN.setReady();

      featureDes_.errorSIN = sd;
      featureDes_.errorSIN.access(time_);
      featureDes_.errorSIN.setReady();

      featureDes_.errordotSIN = vd;
      featureDes_.errordotSIN.access(time_);
      featureDes_.errordotSIN.setReady();

      setGain(gain);
    }

    void printInputs()
    {
      BOOST_TEST_MESSAGE("----- inputs -----");
      BOOST_TEST_MESSAGE("feature_.errorSIN: " << feature_.errorSIN(time_).transpose());
      BOOST_TEST_MESSAGE("featureDes_.errorSIN: " << featureDes_.errorSIN(time_).transpose());
      BOOST_TEST_MESSAGE("featureDes_.errordotSIN: " << featureDes_.errordotSIN(time_).transpose());
      BOOST_TEST_MESSAGE("task.controlGain: " << task_.controlGainSIN(time_));
    }

    void recompute()
    {
      FeatureTestBase::recompute ();

      dynamicgraph::Vector s = feature_.errorSIN;
      dynamicgraph::Vector sd = featureDes_.errorSIN;
      dynamicgraph::Vector vd = featureDes_.errordotSIN;

      computeExpectedTaskOutput (s - sd, - vd);

      checkTaskOutput();
    }

    void printOutputs()
    {
      BOOST_TEST_MESSAGE("----- output -----");
      BOOST_TEST_MESSAGE("time: " << time_);
      BOOST_TEST_MESSAGE("feature.errorSOUT: " << feature_.errorSOUT(time_).transpose());
      BOOST_TEST_MESSAGE("feature.errordotSOUT: " << feature_.errordotSOUT(time_).transpose());
      BOOST_TEST_MESSAGE("task.taskSOUT: " << task_.taskSOUT(time_));    
      //BOOST_TEST_MESSAGE("task.errorSOUT: " << task_.errorSOUT(time_));
      //BOOST_TEST_MESSAGE("task.errordtSOUT: " << task_.errorTimeDerivativeSOUT(time_));
      BOOST_TEST_MESSAGE("Expected task output: " << expectedTaskOutput_.transpose());
    }
};

BOOST_AUTO_TEST_CASE (feature_generic)
{
  std::string srobot("test");
  unsigned int dim=6;
  
  TestFeatureGeneric testFeatureGeneric(dim,srobot);
  
  for (int i = 0; i < 10; ++i)
    testFeatureGeneric.runTest();
}

MatrixHomogeneous randomM()
{
  MatrixHomogeneous M;
  M.translation().setRandom();
  Eigen::Vector3d w (Eigen::Vector3d::Random());
  M.linear() = Eigen::AngleAxisd (w.norm(), w.normalized()).matrix();
  return M;
}

typedef pinocchio::SE3 SE3;
typedef pinocchio::SpecialEuclideanOperationTpl<3, double> SE3_t;
typedef pinocchio::CartesianProductOperation <
  pinocchio::VectorSpaceOperationTpl<3, double>,
  pinocchio::SpecialOrthogonalOperationTpl<3, double> > R3xSO3_t;
typedef SE3_t LieGroup_t;

Vector7 toVector (const pinocchio::SE3& M)
{
  Vector7 v;
  v.head<3>() = M.translation();
  QuaternionMap(v.tail<4>().data()) = M.rotation();
  return v;
}

Vector toVector (const std::vector<MultiBound>& in)
{
  Vector out (in.size());
  for (int i = 0; i < (int)in.size(); ++i)
    out[i] = in[i].getSingleBound();
  return out;
}

class TestFeatureTransformation : public FeatureTestBase
{
 public:
  FeatureTransformation feature_;
  bool relative_;
  pinocchio::Model model_;
  pinocchio::Data  data_;
  pinocchio::JointIndex ja_, jb_;
  pinocchio::FrameIndex fa_, fb_;

  TestFeatureTransformation (bool relative, const std::string &name):
    FeatureTestBase (6, name),
    feature_("feature"+name),
    relative_ (relative),
    data_ (model_)
  {
    pinocchio::buildModels::humanoid (model_, true); // use freeflyer
    model_.lowerPositionLimit.head<3>().setConstant(-1.);
    model_.upperPositionLimit.head<3>().setConstant( 1.);
    jb_ = model_.getJointId ("rarmwrist2_joint");
    fb_ = model_.addFrame (pinocchio::Model::Frame (
          "frame_b",
          jb_,
          model_.getFrameId ("rarmwrist2_joint", pinocchio::JOINT),
          SE3::Identity(),
          pinocchio::OP_FRAME));
    if (relative) {
      ja_ = model_.getJointId ("larmwrist2_joint");
      fa_ = model_.addFrame (pinocchio::Model::Frame (
            "frame_a",
            ja_,
            model_.getFrameId ("larmwrist2_joint", pinocchio::JOINT),
            SE3::Identity(),
            pinocchio::OP_FRAME));
    } else {
      ja_ = 0;
      fa_ = model_.addFrame (pinocchio::Model::Frame (
            "frame_a",
            0,
            0,
            SE3::Identity(),
            pinocchio::OP_FRAME));
    }
    data_ = pinocchio::Data (model_);

    init();

    setJointFrame();
  }

  void _setFrame ()
  {
    setSignal (feature_.jaMfa,
        MatrixHomogeneous(model_.frames[fa_].placement.toHomogeneousMatrix()));
    setSignal (feature_.jbMfb,
        MatrixHomogeneous(model_.frames[fb_].placement.toHomogeneousMatrix()));
  }

  void setJointFrame ()
  {
    model_.frames[fa_].placement.setIdentity();
    model_.frames[fb_].placement.setIdentity();
    _setFrame();
  }

  void setRandomFrame ()
  {
    model_.frames[fa_].placement.setRandom();
    model_.frames[fb_].placement.setRandom();
    _setFrame();
  }

  FeatureAbstract& featureAbstract()
  {
    return feature_;
  }

  void setInputs()
  {
    Vector q (pinocchio::randomConfiguration (model_));
    pinocchio::framesForwardKinematics (model_, data_, q);
    pinocchio::computeJointJacobians (model_, data_);

    // Poses
    setSignal (feature_.oMjb, MatrixHomogeneous(data_.oMi[jb_].toHomogeneousMatrix()));
    if (relative_) {
      setSignal (feature_.oMja, MatrixHomogeneous(data_.oMi[ja_].toHomogeneousMatrix()));
    }

    // Jacobians
    Matrix J (6, model_.nv);
    J.setZero();
    pinocchio::getJointJacobian (model_, data_, jb_, pinocchio::LOCAL, J);
    setSignal (feature_.jbJjb, J);
    if (relative_) {
      J.setZero();
      pinocchio::getJointJacobian (model_, data_, ja_, pinocchio::LOCAL, J);
      setSignal (feature_.jaJja, J);
    }

    // Desired
    setSignal (feature_.faMfbDes, randomM());
    setSignal (feature_.faMfbDesDot, Vector::Random(6));

    double gain = 0;
    //if (time_ % 5 != 0)
      //gain = 2 * (double)rand() / RAND_MAX;
    if (time_ % 2 != 0)
      gain = 1;
    setGain(gain);
  }

  void printInputs()
  {
    BOOST_TEST_MESSAGE("----- inputs -----");
    //BOOST_TEST_MESSAGE("feature_.errorSIN: " << feature_.errorSIN(time_).transpose());
    BOOST_TEST_MESSAGE("task.controlGain: " << task_.controlGainSIN(time_));
  }

  void recompute()
  {
    FeatureTestBase::recompute ();

    const SE3
      oMfb = data_.oMf[fb_],
      oMfa = data_.oMf[fa_],
      faMfbDes (feature_.faMfbDes.accessCopy().matrix());

    computeExpectedTaskOutput (
        toVector(oMfb),
        toVector(oMfa * faMfbDes),
        faMfbDes.toActionMatrixInverse() * feature_.faMfbDesDot.accessCopy(),
        LieGroup_t());

    checkTaskOutput();
  }

  void printOutputs()
  {
    BOOST_TEST_MESSAGE("----- output -----");
    BOOST_TEST_MESSAGE("time: " << time_);
    BOOST_TEST_MESSAGE("feature.errorSOUT: " << feature_.errorSOUT(time_).transpose());
    BOOST_TEST_MESSAGE("feature.errordotSOUT: " << feature_.errordotSOUT(time_).transpose());
    BOOST_TEST_MESSAGE("task.taskSOUT: " << task_.taskSOUT(time_));    
    //BOOST_TEST_MESSAGE("task.errorSOUT: " << task_.errorSOUT(time_));
    //BOOST_TEST_MESSAGE("task.errordtSOUT: " << task_.errorTimeDerivativeSOUT(time_));
    BOOST_TEST_MESSAGE("Expected task output: " << expectedTaskOutput_.transpose());
  }

  virtual void checkFeedForward ()
  {
    setGain(0.);
    // random config
    // set inputs
    // compute e = task_.taskSOUT and J = task_.jacobianSOUT
    // check that e (q + eps*qdot) - e (q) ~= eps * J * qdot
    // compute qdot = J^+ * e
    // check that faMfb (q+eps*qdot) ~= faMfb(q) + eps * faMfbDesDot

    Vector q (pinocchio::randomConfiguration (model_));
    pinocchio::framesForwardKinematics (model_, data_, q);
    pinocchio::computeJointJacobians (model_, data_);

    SE3 faMfb = data_.oMf[fa_].actInv(data_.oMf[fb_]);
    Vector7 q_faMfb = toVector(faMfb);

    // Poses
    setSignal (feature_.oMjb, MatrixHomogeneous(data_.oMi[jb_].toHomogeneousMatrix()));
    if (relative_) {
      setSignal (feature_.oMja, MatrixHomogeneous(data_.oMi[ja_].toHomogeneousMatrix()));
    }

    // Jacobians
    Matrix J (6, model_.nv);
    J.setZero();
    pinocchio::getJointJacobian (model_, data_, jb_, pinocchio::LOCAL, J);
    setSignal (feature_.jbJjb, J);
    if (relative_) {
      J.setZero();
      pinocchio::getJointJacobian (model_, data_, ja_, pinocchio::LOCAL, J);
      setSignal (feature_.jaJja, J);
    }

    Matrix faJfafb;
    J.setZero();
    pinocchio::getFrameJacobian (model_, data_, fb_, pinocchio::LOCAL, J);
    faJfafb = faMfb.toActionMatrix() * J;
    J.setZero();
    pinocchio::getFrameJacobian (model_, data_, fa_, pinocchio::LOCAL, J);
    faJfafb -= J;

    // Desired
    setSignal (feature_.faMfbDes, randomM());

    // Get task jacobian
    task_.jacobianSOUT.recompute(time_);
    J = task_.jacobianSOUT.accessCopy();
    Eigen::JacobiSVD<Matrix> svd (J, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Vector faMfbDesDot (Vector::Zero(6));
    double eps = 1e-6;
    for (int i = 0; i < 6; ++i)
    {
      time_++;
      faMfbDesDot(i) = eps;
      setSignal (feature_.faMfbDesDot, faMfbDesDot);
      task_.taskSOUT.recompute(time_);

      Vector qdot = svd.solve(toVector (task_.taskSOUT.accessCopy()));

      Vector faVfafb = faJfafb * qdot;
      EIGEN_VECTOR_IS_APPROX(faMfbDesDot, faVfafb, eps);
      
      // Check with finite difference.
      Vector q_qdot = pinocchio::integrate (model_, q, qdot);
      pinocchio::framesForwardKinematics (model_, data_, q_qdot);

      SE3 faMfb_next = data_.oMf[fa_].actInv(data_.oMf[fb_]);
      Vector7 q_faMfb_next = toVector(faMfb_next);
      Vector diff (6);
      LieGroup_t().difference (q_faMfb, q_faMfb_next, diff);
      Vector faVfafb_fd = faMfb.toActionMatrix() * diff;
      EIGEN_VECTOR_IS_APPROX(faMfbDesDot, faVfafb_fd, eps);

      faMfbDesDot(i) = 0.;
    }
    time_++;
  }
};

BOOST_AUTO_TEST_CASE (feature_transformation_absolute)
{
  TestFeatureTransformation testAbsolute(false,"abs");
  for (int i = 0; i < 10; ++i)
    testAbsolute.runTest();
  testAbsolute.checkFeedForward();
}

BOOST_AUTO_TEST_CASE (feature_transformation_relative)
{
  TestFeatureTransformation testRelative(true ,"rel");
  for (int i = 0; i < 10; ++i)
    testRelative.runTest();
  testRelative.checkFeedForward();
}
