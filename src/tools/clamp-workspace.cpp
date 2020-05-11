/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <cmath>

#include <sot/core/clamp-workspace.hh>

using namespace std;

#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ClampWorkspace, "ClampWorkspace");

ClampWorkspace::ClampWorkspace(const string &fName)
    : Entity(fName), positionrefSIN(NULL, "ClampWorkspace(" + name +
                                              ")::input(double)::positionref"),
      positionSIN(NULL, "ClampWorkspace(" + name + ")::input(double)::position")

      ,
      alphaSOUT(boost::bind(&ClampWorkspace::computeOutput, this, _1, _2),
                positionrefSIN << positionSIN,
                "ClampWorkspace(" + name + ")::output(vector)::alpha")

      ,
      alphabarSOUT(boost::bind(&ClampWorkspace::computeOutputBar, this, _1, _2),
                   positionrefSIN << positionSIN,
                   "ClampWorkspace(" + name + ")::output(vector)::alphabar")

      ,
      handrefSOUT(boost::bind(&ClampWorkspace::computeRef, this, _1, _2),
                  positionrefSIN << positionSIN,
                  "ClampWorkspace(" + name + ")::output(vector)::ref")

      ,
      timeUpdate(0)

      ,
      alpha(6, 6), alphabar(6, 6)

      ,
      pd(3)

      ,
      beta(1), scale(0), dm_min(0.019), dm_max(0.025), dm_min_yaw(0.019),
      dm_max_yaw(0.119), theta_min(-30. * 3.14159 / 180.),
      theta_max(5. * 3.14159 / 180.), mode(1), frame(FRAME_POINT)

{
  alpha.setZero();
  alphabar.fill(1.);
  bounds[0] = std::make_pair(0.15, 0.5);
  bounds[1] = std::make_pair(-0.4, -0.25);
  bounds[2] = std::make_pair(0.15, 0.55);

  signalRegistration(positionrefSIN << positionSIN << alphaSOUT << alphabarSOUT
                                    << handrefSOUT);
}

void ClampWorkspace::update(int time) {
  if (time <= timeUpdate) {
    return;
  }

  alpha.setZero();
  alphabar.setIdentity();

  const MatrixHomogeneous &posref = positionrefSIN.access(time);
  const MatrixHomogeneous &pos = positionSIN.access(time);

  MatrixHomogeneous prefMw = posref.inverse(Eigen::Affine);
  prefMp = prefMw * pos;
  Vector x(prefMp.translation());

  for (int i = 0; i < 3; ++i) {
    double check_min = std::max(x(i) - bounds[i].first, 0.);
    double check_max = std::max(bounds[i].second - x(i), 0.);
    double dm = std::min(check_min, check_max);

    double Y = (dm - dm_min) / (dm_max - dm_min);
    if (Y < 0) {
      Y = 0;
    }
    if (Y > 1) {
      Y = 1;
    }

    switch (mode) {
    case 0:
      alpha(i, i) = 0;
      alphabar(i, i) = 1;
      break;
    case 1:
      alpha(i, i) = 1;
      alphabar(i, i) = 0;
      break;
    case 2:
    default:
      alpha(i, i) = 0.5 * (1 + tanh(1 / Y - 1 / (1 - Y)));
      alphabar(i, i) = 1 - alpha(i);
    }

    if (i == 2) {
      Y = (dm - dm_min_yaw) / (dm_max_yaw - dm_min_yaw);
      if (Y < 0) {
        Y = 0;
      }
      if (Y > 1) {
        Y = 1;
      }
      if (mode == 2) {
        alpha(i + 3, i + 3) = 0.5 * (1 + tanh(1 / Y - 1 / (1 - Y)));
        alphabar(i + 3, i + 3) = 1 - alpha(i + 3);
      }
    }
  }

  if (frame == FRAME_POINT) {
    MatrixHomogeneous prefMp_tmp = prefMp;
    MatrixHomogeneous pMpref = prefMp.inverse(Eigen::Affine);
    for (int i = 0; i < 3; ++i) {
      pMpref(i, 3) = 0;
      prefMp_tmp(i, 3) = 0;
    }

    MatrixTwist pTpref;
    buildFrom(pMpref, pTpref);
    MatrixTwist prefTp;
    buildFrom(prefMp_tmp, prefTp);

    Matrix tmp;
    tmp = alpha * prefTp;
    alpha = pTpref * tmp;

    tmp = alphabar * prefTp;
    alphabar = pTpref * tmp;
  }

  for (int i = 0; i < 3; ++i) {
    pd(i) = 0.5 * (bounds[i].first + bounds[i].second);
  }

  VectorRollPitchYaw rpy;
  rpy(0) = 0;
  rpy(1) = -3.14159256 / 2;
  rpy(2) = theta_min + (theta_max - theta_min) * (x(1) - bounds[1].first) /
                           (bounds[1].second - bounds[1].first);

  Eigen::Affine3d _Rd(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()));
  Rd = _Rd.linear();

  Eigen::Affine3d _tmpaffine;
  _tmpaffine = Eigen::Translation3d(pd);

  handref = (_tmpaffine * _Rd);
  timeUpdate = time;
}

Matrix &ClampWorkspace::computeOutput(Matrix &res, int time) {
  update(time);
  res = alpha;
  return res;
}

Matrix &ClampWorkspace::computeOutputBar(Matrix &res, int time) {
  update(time);
  res = alphabar;
  return res;
}

MatrixHomogeneous &ClampWorkspace::computeRef(MatrixHomogeneous &res,
                                              int time) {
  update(time);
  res = handref;
  return res;
}

void ClampWorkspace::display(std::ostream &os) const {
  os << "ClampWorkspace<" << name << ">" << endl << endl;
  os << "alpha: " << alpha << endl;
  os << "pos in ws: " << prefMp << endl;
  os << "bounds: " << bounds[0].first << " " << bounds[0].second << " "
     << bounds[1].first << " " << bounds[1].second << " " << bounds[2].first
     << " " << bounds[2].second << endl;
}
