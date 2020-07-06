/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
#include <sot/core/debug.hh>
#include <sot/core/exception-feature.hh>
#include <sot/core/feature-point6d-relative.hh>

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/pool.h>
#include <sot/core/matrix-geometry.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;
namespace dg = dynamicgraph;

#include <sot/core/factory.hh>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePoint6dRelative,
                                   "FeaturePoint6dRelative");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeaturePoint6dRelative::FeaturePoint6dRelative(const string &pointName)
    : FeaturePoint6d(pointName),
      positionReferenceSIN(NULL, "sotFeaturePoint6dRelative(" + name +
                                     ")::input(matrixHomo)::positionRef"),
      articularJacobianReferenceSIN(NULL, "sotFeaturePoint6dRelative(" + name +
                                              ")::input(matrix)::JqRef"),
      dotpositionSIN(NULL, "sotFeaturePoint6dRelative(" + name +
                               ")::input(matrixHomo)::dotposition"),
      dotpositionReferenceSIN(NULL,
                              "sotFeaturePoint6dRelative(" + name +
                                  ")::input(matrixHomo)::dotpositionRef") {
  jacobianSOUT.addDependency(positionReferenceSIN);
  jacobianSOUT.addDependency(articularJacobianReferenceSIN);

  errorSOUT.addDependency(positionReferenceSIN);

  errordotSOUT.addDependency(dotpositionReferenceSIN);

  signalRegistration(positionReferenceSIN << articularJacobianReferenceSIN);

  signalRegistration(dotpositionSIN << dotpositionReferenceSIN << errordotSOUT);

  errordotSOUT.setFunction(
      boost::bind(&FeaturePoint6dRelative::computeErrordot, this, _1, _2));
  initCommands();
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/** Compute the interaction matrix from a subset of
 * the possible features.
 */
Matrix &FeaturePoint6dRelative::computeJacobian(Matrix &Jres, int time) {
  sotDEBUG(15) << "# In {" << endl;

  const Matrix &Jq = articularJacobianSIN(time);
  const Matrix &JqRef = articularJacobianReferenceSIN(time);
  const MatrixHomogeneous &wMp = positionSIN(time);
  const MatrixHomogeneous &wMpref = positionReferenceSIN(time);

  const Matrix::Index cJ = Jq.cols();
  Matrix J(6, cJ);
  {
    MatrixHomogeneous pMw;
    pMw = wMp.inverse(Eigen::Affine);
    MatrixHomogeneous pMpref;
    pMpref = pMw * wMpref;

    MatrixTwist pVpref;
    buildFrom(pMpref, pVpref);
    J = pVpref * JqRef;
    J -= Jq;
  }

  const Flags &fl = selectionSIN(time);
  const int dim = dimensionSOUT(time);
  sotDEBUG(15) << "Dimension=" << dim << std::endl;
  Jres.resize(dim, cJ);

  unsigned int rJ = 0;
  for (unsigned int r = 0; r < 6; ++r)
    if (fl(r)) {
      for (unsigned int c = 0; c < cJ; ++c)
        Jres(rJ, c) = J(r, c);
      rJ++;
    }

  sotDEBUG(15) << "# Out }" << endl;
  return Jres;
}

/** Compute the error between two visual features from a subset
 * a the possible features.
 */
Vector &FeaturePoint6dRelative::computeError(Vector &error, int time) {
  sotDEBUGIN(15);

  //   /* TODO */
  //   error.resize(6); error.fill(.0);

  const MatrixHomogeneous &wMp = positionSIN(time);
  const MatrixHomogeneous &wMpref = positionReferenceSIN(time);

  MatrixHomogeneous pMw;
  pMw = wMp.inverse(Eigen::Affine);
  MatrixHomogeneous pMpref;
  pMpref = pMw * wMpref;

  MatrixHomogeneous Merr;
  try {
    if (isReferenceSet()) {
      // TODO: Deal with the case of FeaturePoint6dRelative reference without
      // dcast
      FeaturePoint6dRelative *sdes6d =
          dynamic_cast<FeaturePoint6dRelative *>(getReference());
      if (NULL != sdes6d) {
        const MatrixHomogeneous &wMp_des = sdes6d->positionSIN(time);
        const MatrixHomogeneous &wMpref_des =
            sdes6d->positionReferenceSIN(time);
        MatrixHomogeneous pMw_des;
        pMw_des = wMp_des.inverse(Eigen::Affine);
        MatrixHomogeneous pMpref_des;
        pMpref_des = pMw_des * wMpref_des;
        MatrixHomogeneous Minv;
        Minv = pMpref_des.inverse(Eigen::Affine);
        Merr = pMpref * Minv;
      } else {
        const MatrixHomogeneous &Mref = getReference()->positionSIN(time);
        MatrixHomogeneous Minv;
        Minv = Mref.inverse(Eigen::Affine);
        Merr = pMpref * Minv;
      }
    } else {
      Merr = pMpref;
    }
  } catch (...) {
    Merr = pMpref;
  }

  MatrixRotation Rerr;
  Rerr = Merr.linear();
  VectorUTheta rerr(Rerr);

  const Flags &fl = selectionSIN(time);
  error.resize(dimensionSOUT(time));
  unsigned int cursor = 0;
  for (unsigned int i = 0; i < 3; ++i) {
    if (fl(i))
      error(cursor++) = Merr(i, 3);
  }
  for (unsigned int i = 0; i < 3; ++i) {
    if (fl(i + 3))
      error(cursor++) = rerr.angle() * rerr.axis()(i);
  }

  sotDEBUGOUT(15);
  return error;
}

/** Compute the error between two visual features from a subset
 * a the possible features.
 *
 * This is computed by the desired feature.
 */
Vector &FeaturePoint6dRelative::computeErrorDot(Vector &errordot, int time) {
  sotDEBUGIN(15);

  //   /* TODO */
  //   error.resize(6); error.fill(.0);
  const MatrixHomogeneous &wMp = positionSIN(time);
  const MatrixHomogeneous &wMpref = positionReferenceSIN(time);
  const MatrixHomogeneous &wdMp = dotpositionSIN(time);
  const MatrixHomogeneous &wdMpref = dotpositionReferenceSIN(time);

  sotDEBUG(15) << "wdMp :" << wdMp << endl;
  sotDEBUG(15) << "wdMpref :" << wdMpref << endl;

  MatrixRotation dRerr;
  Vector dtrerr;

  try {
    MatrixRotation wRp;
    wRp = wMp.linear();
    MatrixRotation wRpref;
    wRpref = wMpref.linear();
    MatrixRotation wdRp;
    wdRp = wdMp.linear();
    MatrixRotation wdRpref;
    wdRpref = wdMpref.linear();

    Vector trp(3);
    trp = wMp.translation();
    Vector trpref(3);
    trpref = wMpref.translation();
    Vector trdp(3);
    trdp = wdMp.translation();
    Vector trdpref(3);
    trdpref = wdMpref.translation();

    sotDEBUG(15) << "Everything is extracted" << endl;
    MatrixRotation wdRpt, wRpt, op1, op2;
    wdRpt = wdRp.transpose();
    op1 = wdRpt * wRpref;
    wRpt = wRp.transpose();
    op2 = wRpt * wdRpref;
    dRerr = op1 + op2;

    sotDEBUG(15) << "dRerr" << dRerr << endl;
    Vector trtmp1(3), vop1(3), vop2(3);
    trtmp1 = trpref - trp;
    vop1 = wdRpt * trtmp1;
    trtmp1 = trdpref - trdp;
    vop2 = wRpt * trtmp1;
    dtrerr = vop1 - vop2;

    sotDEBUG(15) << "dtrerr" << dtrerr << endl;

  } catch (...) {
    sotDEBUG(15) << "You've got a problem with errordot." << std::endl;
  }

  VectorUTheta rerr(dRerr);

  const Flags &fl = selectionSIN(time);
  errordot.resize(dimensionSOUT(time));
  unsigned int cursor = 0;
  for (unsigned int i = 0; i < 3; ++i) {
    if (fl(i))
      errordot(cursor++) = dtrerr(i);
  }
  for (unsigned int i = 0; i < 3; ++i) {
    if (fl(i + 3))
      errordot(cursor++) = rerr.angle() * rerr.axis()(i);
  }

  sotDEBUGOUT(15);
  return errordot;
}

static const char *featureNames[] = {"X ", "Y ", "Z ", "RX", "RY", "RZ"};
void FeaturePoint6dRelative::display(std::ostream &os) const {
  os << "Point6dRelative <" << name << ">: (";

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

void FeaturePoint6dRelative::initCommands(void) {
  using namespace command;
  addCommand("initSdes", makeCommandVoid1(
                             *this, &FeaturePoint6dRelative::initSdes,
                             docCommandVoid1("Initialize the desired feature.",
                                             "string (desired feature name)")));
}

/* Initialise the reference value: set up the sdes signal of the current
 * feature, and freezes the position and position-reference of the desired
 * feature.
 */
void FeaturePoint6dRelative::initSdes(const std::string &nameSdes) {
  FeaturePoint6dRelative &sdes = dynamic_cast<FeaturePoint6dRelative &>(
      dg::PoolStorage::getInstance()->getEntity(nameSdes));

  setReference(&sdes);

  const int timeCurr = positionSIN.getTime() + 1;
  positionSIN.recompute(timeCurr);
  positionReferenceSIN.recompute(timeCurr);

  sdes.positionSIN.setConstant(positionSIN.accessCopy());
  sdes.positionReferenceSIN.setConstant(positionReferenceSIN.accessCopy());
}
