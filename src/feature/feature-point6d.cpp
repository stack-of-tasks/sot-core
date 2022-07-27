/*
 * Copyright 2010, 2011, 2012
 * François Bleibel,
 * Olivier Stasse,
 * Florent Lamiraux
 * Nicolas Mansard
 *
 * CNRS/AIST
 *
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
//#define VP_DEBUG
//#define VP_DEBUG_MODE 45
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>

#include <Eigen/LU>
#include <sot/core/debug.hh>
#include <sot/core/exception-feature.hh>
#include <sot/core/feature-point6d.hh>
#include <sot/core/macros.hh>

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

#include <sot/core/factory.hh>
SOT_CORE_DISABLE_WARNING_PUSH
SOT_CORE_DISABLE_WARNING_DEPRECATED
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePoint6d, "FeaturePoint6d");
SOT_CORE_DISABLE_WARNING_POP

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const FeaturePoint6d::ComputationFrameType
    FeaturePoint6d::COMPUTATION_FRAME_DEFAULT = FRAME_DESIRED;

FeaturePoint6d::FeaturePoint6d(const string &pointName)
    : FeatureAbstract(pointName),
      computationFrame_(COMPUTATION_FRAME_DEFAULT),
      positionSIN(
          NULL, "sotFeaturePoint6d(" + name + ")::input(matrixHomo)::position"),
      velocitySIN(NULL,
                  "sotFeaturePoint6d(" + name + ")::input(vector)::velocity"),
      articularJacobianSIN(
          NULL, "sotFeaturePoint6d(" + name + ")::input(matrix)::Jq"),
      error_th_(),
      R_(),
      Rref_(),
      Rt_(),
      Rreft_(),
      P_(3, 3),
      Pinv_(3, 3),
      accuracy_(1e-8) {
  jacobianSOUT.addDependency(positionSIN);
  jacobianSOUT.addDependency(articularJacobianSIN);

  errorSOUT.addDependency(positionSIN);

  signalRegistration(positionSIN << articularJacobianSIN);
  signalRegistration(errordotSOUT << velocitySIN);
  errordotSOUT.setFunction(
      boost::bind(&FeaturePoint6d::computeErrordot, this, _1, _2));
  errordotSOUT.addDependency(velocitySIN);
  errordotSOUT.addDependency(positionSIN);
  errordotSOUT.addDependency(errorSOUT);

  // Commands
  //
  {
    using namespace dynamicgraph::command;
    std::string docstring;
    // Set computation frame
    docstring =
        "Set computation frame\n"
        "\n"
        "  Input:\n"
        "    a string: 'current' or 'desired'.\n"
        "      If 'current', the error is defined as the rotation "
        "vector (VectorUTheta)\n"
        "      corresponding to the position of the reference in the "
        "current frame:\n"
        "                         -1 *\n"
        "        error = utheta (M  M )\n"
        "      If 'desired',      *-1\n"
        "        error = utheta (M   M)\n";
    addCommand("frame",
               new dynamicgraph::command::Setter<FeaturePoint6d, std::string>(
                   *this, &FeaturePoint6d::computationFrame, docstring));
    docstring =
        "Get frame of computation of the error\n"
        "\n"
        "  See command 'frame' for definition.\n";
    addCommand("getFrame",
               new dynamicgraph::command::Getter<FeaturePoint6d, std::string>(
                   *this, &FeaturePoint6d::computationFrame, docstring));
    addCommand(
        "keep",
        makeCommandVoid0(
            *this, &FeaturePoint6d::servoCurrentPosition,
            docCommandVoid0(
                "modify the desired position to servo at current pos.")));
  }
}

void FeaturePoint6d::addDependenciesFromReference(void) {
  assert(isReferenceSet());
  errorSOUT.addDependency(getReference()->positionSIN);
  jacobianSOUT.addDependency(getReference()->positionSIN);
}

void FeaturePoint6d::removeDependenciesFromReference(void) {
  assert(isReferenceSet());
  errorSOUT.removeDependency(getReference()->positionSIN);
  jacobianSOUT.removeDependency(getReference()->positionSIN);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void FeaturePoint6d::computationFrame(const std::string &inFrame) {
  if (inFrame == "current")
    computationFrame_ = FRAME_CURRENT;
  else if (inFrame == "desired")
    computationFrame_ = FRAME_DESIRED;
  else {
    std::string msg("FeaturePoint6d::computationFrame: " + inFrame +
                    ": invalid argument,\n"
                    "expecting 'current' or 'desired'");
    throw ExceptionFeature(ExceptionFeature::GENERIC, msg);
  }
}

/// \brief Get computation frame
std::string FeaturePoint6d::computationFrame() const {
  switch (computationFrame_) {
    case FRAME_CURRENT:
      return "current";
    case FRAME_DESIRED:
      return "desired";
  }
  assert(false && "Case not handled");
  return "Case not handled";
}
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int &FeaturePoint6d::getDimension(unsigned int &dim, int time) {
  sotDEBUG(25) << "# In {" << endl;

  const Flags &fl = selectionSIN.access(time);

  dim = 0;
  for (int i = 0; i < 6; ++i)
    if (fl(i)) dim++;

  sotDEBUG(25) << "# Out }" << endl;
  return dim;
}

/** Compute the interaction matrix from a subset of
 * the possible features.
 */
Matrix &FeaturePoint6d::computeJacobian(Matrix &J, int time) {
  sotDEBUG(15) << "# In {" << endl;

  const Matrix &Jq = articularJacobianSIN(time);
  const int &dim = dimensionSOUT(time);
  const Flags &fl = selectionSIN(time);

  sotDEBUG(25) << "dim = " << dimensionSOUT(time) << " time:" << time << " "
               << dimensionSOUT.getTime() << " " << dimensionSOUT.getReady()
               << endl;
  sotDEBUG(25) << "selec = " << selectionSIN(time) << " time:" << time << " "
               << selectionSIN.getTime() << " " << selectionSIN.getReady()
               << endl;

  sotDEBUG(15) << "Dimension=" << dim << std::endl;

  const Matrix::Index cJ = Jq.cols();
  J.resize(dim, cJ);
  Matrix LJq(6, cJ);

  if (FRAME_CURRENT == computationFrame_) {
    /* The Jacobian on rotation is equal to Jr = - hdRh Jr6d.
     * The Jacobian in translation is equalt to Jt = [hRw(wthd-wth)]x Jr - Jt.
     */
    const MatrixHomogeneous &wMh = positionSIN(time);
    MatrixRotation wRh;
    wRh = wMh.linear();
    MatrixRotation wRhd;
    Vector hdth(3), Rhdth(3);

    if (isReferenceSet()) {
      const MatrixHomogeneous &wMhd = getReference()->positionSIN(time);
      wRhd = wMhd.linear();
      for (unsigned int i = 0; i < 3; ++i) hdth(i) = wMhd(i, 3) - wMh(i, 3);
    } else {
      wRhd.setIdentity();
      for (unsigned int i = 0; i < 3; ++i) hdth(i) = -wMh(i, 3);
    }
    Rhdth = (wRh.inverse()) * hdth;
    MatrixRotation hdRh;
    hdRh = (wRhd.inverse()) * wRh;

    Matrix Lx(6, 6);
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++) {
        if (i == j) {
          Lx(i, j) = -1;
        } else {
          Lx(i, j) = 0;
        }
        Lx(i + 3, j) = 0;
        Lx(i + 3, j + 3) = -hdRh(i, j);
      }
    }
    const double &X = Rhdth(0), &Y = Rhdth(1), &Z = Rhdth(2);
    Lx(0, 4) = -Z;
    Lx(0, 5) = Y;
    Lx(1, 3) = Z;
    Lx(1, 5) = -X;
    Lx(2, 3) = -Y;
    Lx(2, 4) = X;
    Lx(0, 3) = 0;
    Lx(1, 4) = 0;
    Lx(2, 5) = 0;
    sotDEBUG(15) << "Lx= " << Lx << endl;

    LJq = Lx * Jq;
  } else {
    /* The Jacobian in rotation is equal to Jr = hdJ = hdRh Jr.
     * The Jacobian in translation is equal to Jr = hdJ = hdRh Jr. */
    const MatrixHomogeneous &wMh = positionSIN(time);
    MatrixRotation wRh;
    wRh = wMh.linear();
    MatrixRotation hdRh;

    if (isReferenceSet()) {
      const MatrixHomogeneous &wMhd = getReference()->positionSIN(time);
      MatrixRotation wRhd;
      wRhd = wMhd.linear();
      hdRh = (wRhd.inverse()) * wRh;
    } else {
      hdRh = wRh;
    }

    LJq.fill(0);
    for (unsigned int i = 0; i < 3; i++)
      for (unsigned int j = 0; j < cJ; j++) {
        for (unsigned int k = 0; k < 3; k++) {
          LJq(i, j) += hdRh(i, k) * Jq(k, j);
          LJq(i + 3, j) += hdRh(i, k) * Jq(k + 3, j);
        }
      }
  }

  /* Select the active line of Jq. */
  unsigned int rJ = 0;
  for (unsigned int r = 0; r < 6; ++r)
    if (fl(r)) {
      for (unsigned int c = 0; c < cJ; ++c) J(rJ, c) = LJq(r, c);
      rJ++;
    }

  sotDEBUG(15) << "# Out }" << endl;
  return J;
}

#define SOT_COMPUTE_H1MH2(wMh, wMhd, hMhd)     \
  {                                            \
    MatrixHomogeneous hMw;                     \
    hMw = wMh.inverse(Eigen::Affine);          \
    sotDEBUG(15) << "hMw = " << hMw << endl;   \
    hMhd = hMw * wMhd;                         \
    sotDEBUG(15) << "hMhd = " << hMhd << endl; \
  }

/** Compute the error between two visual features from a subset
 * a the possible features.
 */
Vector &FeaturePoint6d::computeError(Vector &error, int time) {
  sotDEBUGIN(15);

  const Flags &fl = selectionSIN(time);
  const MatrixHomogeneous &wMh = positionSIN(time);
  sotDEBUG(15) << "wMh = " << wMh << endl;

  /* Computing only translation:                                        *
   * trans( hMw wMhd ) = htw + hRw wthd                                 *
   *                   = -hRw wth + hrW wthd                            *
   *                   = hRw ( wthd - wth )                             *
   * The second line is obtained by writting hMw as the inverse of wMh. */

  MatrixHomogeneous hMhd;
  if (isReferenceSet()) {
    const MatrixHomogeneous &wMhd = getReference()->positionSIN(time);
    sotDEBUG(15) << "wMhd = " << wMhd << endl;
    switch (computationFrame_) {
      case FRAME_CURRENT:
        SOT_COMPUTE_H1MH2(wMh, wMhd, hMhd);
        break;
      case FRAME_DESIRED:
        SOT_COMPUTE_H1MH2(wMhd, wMh, hMhd);
        break;  // Compute hdMh indeed.
    };
  } else {
    switch (computationFrame_) {
      case FRAME_CURRENT:
        hMhd = wMh.inverse();
        break;
      case FRAME_DESIRED:
        hMhd = wMh;
        break;  // Compute hdMh indeed.
    };
  }

  sotDEBUG(25) << "dim = " << dimensionSOUT(time) << " time:" << time << " "
               << dimensionSOUT.getTime() << " " << dimensionSOUT.getReady()
               << endl;
  sotDEBUG(25) << "selec = " << selectionSIN(time) << " time:" << time << " "
               << selectionSIN.getTime() << " " << selectionSIN.getReady()
               << endl;

  error.resize(dimensionSOUT(time));
  unsigned int cursor = 0;
  for (unsigned int i = 0; i < 3; ++i) {
    if (fl(i)) error(cursor++) = hMhd(i, 3);
  }

  if (fl(3) || fl(4) || fl(5)) {
    MatrixRotation hRhd;
    hRhd = hMhd.linear();
    error_th_.fromRotationMatrix(hRhd);
    for (unsigned int i = 0; i < 3; ++i) {
      if (fl(i + 3)) error(cursor++) = error_th_.angle() * error_th_.axis()(i);
    }
  }

  sotDEBUGOUT(15);
  return error;
}

void FeaturePoint6d::inverseJacobianRodrigues() {
  const double &r1 = error_th_.angle() * error_th_.axis()(0);
  const double &r2 = error_th_.angle() * error_th_.axis()(1);
  const double &r3 = error_th_.angle() * error_th_.axis()(2);
  double r1_2 = r1 * r1;
  double r2_2 = r2 * r2;
  double r3_2 = r3 * r3;
  double r1_3 = r1 * r1_2;
  double r2_3 = r2 * r2_2;
  double r3_3 = r3 * r3_2;
  double r1_4 = r1_2 * r1_2;
  double r2_4 = r2_2 * r2_2;
  double r3_4 = r3_2 * r3_2;
  double norm_2 = r3_2 + r2_2 + r1_2;

  if (norm_2 < accuracy_) {
    P_.setIdentity();
  } else {
    // This code has been generated by maxima software
    P_(0, 0) =
        ((r3_2 + r2_2) * sqrt(norm_2) * sin(sqrt(norm_2)) + r1_2 * r3_2 +
         r1_2 * r2_2 + r1_4) /
        (r3_4 + (2 * r2_2 + 2 * r1_2) * r3_2 + r2_4 + 2 * r1_2 * r2_2 + r1_4);
    P_(0, 1) =
        -(r1 * r2 * sqrt(norm_2) * sin(sqrt(norm_2)) +
          (r3_3 + (r2_2 + r1_2) * r3) * cos(sqrt(norm_2)) - r3_3 -
          r1 * r2 * r3_2 + (-r2_2 - r1_2) * r3 - r1 * r2_3 - r1_3 * r2) /
        (r3_4 + (2 * r2_2 + 2 * r1_2) * r3_2 + r2_4 + 2 * r1_2 * r2_2 + r1_4);
    P_(0, 2) =
        -(r1 * r3 * sqrt(norm_2) * sin(sqrt(norm_2)) +
          (-r2 * r3_2 - r2_3 - r1_2 * r2) * cos(sqrt(norm_2)) - r1 * r3_3 +
          r2 * r3_2 + (-r1 * r2_2 - r1_3) * r3 + r2_3 + r1_2 * r2) /
        (r3_4 + (2 * r2_2 + 2 * r1_2) * r3_2 + r2_4 + 2 * r1_2 * r2_2 + r1_4);
    P_(1, 0) =
        -(r1 * r2 * sqrt(norm_2) * sin(sqrt(norm_2)) +
          ((-r2_2 - r1_2) * r3 - r3_3) * cos(sqrt(norm_2)) + r3_3 -
          r1 * r2 * r3_2 + (r2_2 + r1_2) * r3 - r1 * r2_3 - r1_3 * r2) /
        (r3_4 + (2 * r2_2 + 2 * r1_2) * r3_2 + r2_4 + 2 * r1_2 * r2_2 + r1_4);
    P_(1, 1) =
        ((r3_2 + r1_2) * sqrt(norm_2) * sin(sqrt(norm_2)) + r2_2 * r3_2 + r2_4 +
         r1_2 * r2_2) /
        (r3_4 + (2 * r2_2 + 2 * r1_2) * r3_2 + r2_4 + 2 * r1_2 * r2_2 + r1_4);
    P_(1, 2) =
        -(r2 * r3 * sqrt(norm_2) * sin(sqrt(norm_2)) +
          (r1 * r3_2 + r1 * r2_2 + r1_3) * cos(sqrt(norm_2)) - r2 * r3_3 -
          r1 * r3_2 + (-r2_3 - r1_2 * r2) * r3 - r1 * r2_2 - r1_3) /
        (r3_4 + (2 * r2_2 + 2 * r1_2) * r3_2 + r2_4 + 2 * r1_2 * r2_2 + r1_4);
    P_(2, 0) =
        -(r1 * r3 * sqrt(norm_2) * sin(sqrt(norm_2)) +
          (r2 * r3_2 + r2_3 + r1_2 * r2) * cos(sqrt(norm_2)) - r1 * r3_3 -
          r2 * r3_2 + (-r1 * r2_2 - r1_3) * r3 - r2_3 - r1_2 * r2) /
        (r3_4 + (2 * r2_2 + 2 * r1_2) * r3_2 + r2_4 + 2 * r1_2 * r2_2 + r1_4);
    P_(2, 1) =
        -(r2 * r3 * sqrt(norm_2) * sin(sqrt(norm_2)) +
          (-r1 * r3_2 - r1 * r2_2 - r1_3) * cos(sqrt(norm_2)) - r2 * r3_3 +
          r1 * r3_2 + (-r2_3 - r1_2 * r2) * r3 + r1 * r2_2 + r1_3) /
        (r3_4 + (2 * r2_2 + 2 * r1_2) * r3_2 + r2_4 + 2 * r1_2 * r2_2 + r1_4);
    P_(2, 2) =
        ((r2_2 + r1_2) * sqrt(norm_2) * sin(sqrt(norm_2)) + r3_4 +
         (r2_2 + r1_2) * r3_2) /
        (r3_4 + (2 * r2_2 + 2 * r1_2) * r3_2 + r2_4 + 2 * r1_2 * r2_2 + r1_4);
  }
  Pinv_ = P_.inverse();
}

Vector &FeaturePoint6d::computeErrordot(Vector &errordot, int time) {
  if (isReferenceSet()) {
    const Vector &velocity = getReference()->velocitySIN(time);
    const MatrixHomogeneous &M = positionSIN(time);
    const MatrixHomogeneous &Mref = getReference()->positionSIN(time);
    // Linear velocity if the reference frame
    v_(0) = velocity(0);
    v_(1) = velocity(1);
    v_(2) = velocity(2);
    // Angular velocity if the reference frame
    omega_(0) = velocity(3);
    omega_(1) = velocity(4);
    omega_(2) = velocity(5);
    R_ = M.linear();
    t_ = M.translation();
    Rt_ = R_.transpose();
    Rref_ = Mref.linear();
    tref_ = Mref.translation();
    Rreft_ = Rref_.transpose();
    errorSOUT.recompute(time);
    inverseJacobianRodrigues();
    switch (computationFrame_) {
      case FRAME_CURRENT:
        // \dot{e}_{t} = R^{T} v
        errordot_t_ = Rt_ * v_;
        // \dot{e}_{\theta} = P^{-1}(e_{theta})R^{*T}\omega
        Rreftomega_ = Rreft_ * omega_;
        errordot_th_ = Pinv_ * Rreftomega_;
        break;
      case FRAME_DESIRED:
        errordot_t_ = Rreft_ * (omega_.cross(tref_ - t_) - v_);
        errordot_th_ = -Pinv_ * (Rt_ * omega_);
        break;
    }
  } else {
    errordot_t_.setZero();
    errordot_th_.setZero();
  }

  const Flags &fl = selectionSIN(time);
  errordot.resize(dimensionSOUT(time));
  unsigned int cursor = 0;
  for (unsigned int i = 0; i < 3; ++i) {
    if (fl(i)) {
      errordot(cursor++) = errordot_t_(i);
    }
  }

  if (fl(3) || fl(4) || fl(5)) {
    for (unsigned int i = 0; i < 3; ++i) {
      if (fl(i + 3)) {
        errordot(cursor++) = errordot_th_(i);
      }
    }
  }

  return errordot;
}

/* Modify the value of the reference (sdes) so that it corresponds
 * to the current position. The effect on the servo is to maintain the
 * current position and correct any drift. */
void FeaturePoint6d::servoCurrentPosition(void) {
  sotDEBUGIN(15);

  if (!isReferenceSet()) {
    sotERROR << "The reference is not set, this function should not be called"
             << std::endl;
    throw ExceptionFeature(
        ExceptionFeature::GENERIC,
        "The reference is not set, this function should not be called");
  }
  getReference()->positionSIN = positionSIN.accessCopy();

  sotDEBUGOUT(15);
}

static const char *featureNames[] = {"X ", "Y ", "Z ", "RX", "RY", "RZ"};
void FeaturePoint6d::display(std::ostream &os) const {
  os << "Point6d <" << name << ">: (";

  try {
    const Flags &fl = selectionSIN.accessCopy();
    bool first = true;
    for (int i = 0; i < 6; ++i)
      if (fl(i)) {
        if (first) {
          first = false;
        } else {
          os << ",";
        }
        os << featureNames[i];
      }
    os << ") ";
  } catch (ExceptionAbstract e) {
    os << " selectSIN not set.";
  }
}
