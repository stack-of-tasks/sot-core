/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <sot/core/debug.hh>
#include <sot/core/matrix-geometry.hh>

using namespace std;
using namespace dynamicgraph::sot;

static const double ANGLE_MINIMUM = 0.0001;
static const double SINC_MINIMUM = 1e-8;
static const double COSC_MINIMUM = 2.5e-4;

VectorRotation &VectorQuaternion::fromMatrix(const MatrixRotation &rot) {
  sotDEBUGIN(15);

  const dynamicgraph::Matrix &rotmat = rot;

  double d0 = rotmat(0, 0), d1 = rotmat(1, 1), d2 = rotmat(2, 2);

  // The trace determines the method of decomposition
  double rr = 1.0 + d0 + d1 + d2;

  double &_x = vector(1);
  double &_y = vector(2);
  double &_z = vector(3);
  double &_r = vector(0);

  if (rr > 0) {
    double s = 0.5 / sqrt(rr);
    _x = (rotmat(2, 1) - rotmat(1, 2)) * s;
    _y = (rotmat(0, 2) - rotmat(2, 0)) * s;
    _z = (rotmat(1, 0) - rotmat(0, 1)) * s;
    _r = 0.25 / s;
  } else {
    // Trace is less than zero, so need to determine which
    // major diagonal is largest
    if ((d0 > d1) && (d0 > d2)) {
      double s = 0.5 / sqrt(1 + d0 - d1 - d2);
      _x = 0.5 * s;
      _y = (rotmat(0, 1) + rotmat(1, 0)) * s;
      _z = (rotmat(0, 2) + rotmat(2, 0)) * s;
      _r = (rotmat(1, 2) + rotmat(2, 1)) * s;
    } else if (d1 > d2) {
      double s = 0.5 / sqrt(1 + d0 - d1 - d2);
      _x = (rotmat(0, 1) + rotmat(1, 0)) * s;
      _y = 0.5 * s;
      _z = (rotmat(1, 2) + rotmat(2, 1)) * s;
      _r = (rotmat(0, 2) + rotmat(2, 0)) * s;
    } else {
      double s = 0.5 / sqrt(1 + d0 - d1 - d2);
      _x = (rotmat(0, 2) + rotmat(2, 0)) * s;
      _y = (rotmat(1, 2) + rotmat(2, 1)) * s;
      _z = 0.5 * s;
      _r = (rotmat(0, 1) + rotmat(1, 0)) * s;
    }
  }

  sotDEBUGOUT(15);
  return *this;
}

VectorRotation &VectorQuaternion::fromVector(const VectorUTheta &ut) {
  sotDEBUGIN(15);

  double theta = sqrt(ut(0) * ut(0) + ut(1) * ut(1) + ut(2) * ut(2));
  double si = sin(theta);
  double co = cos(theta);
  vector(0) = ut(0) / si;
  vector(1) = ut(1) / si;
  vector(2) = ut(2) / si;
  vector(3) = co;

  sotDEBUGOUT(15);
  return *this;
}

MatrixRotation &VectorQuaternion::toMatrix(MatrixRotation &rot) const {
  sotDEBUGIN(15);

  dynamicgraph::Matrix &rotmat = rot;

  const double &_x = vector(1);
  const double &_y = vector(2);
  const double &_z = vector(3);
  const double &_r = vector(0);

  double x2 = _x * _x;
  double y2 = _y * _y;
  double z2 = _z * _z;
  double r2 = _r * _r;

  rotmat(0, 0) = r2 + x2 - y2 - z2; // fill diagonal terms
  rotmat(1, 1) = r2 - x2 + y2 - z2;
  rotmat(2, 2) = r2 - x2 - y2 + z2;

  double xy = _x * _y;
  double yz = _y * _z;
  double zx = _z * _x;
  double rx = _r * _x;
  double ry = _r * _y;
  double rz = _r * _z;

  rotmat(0, 1) = 2 * (xy - rz); // fill off diagonal terms
  rotmat(0, 2) = 2 * (zx + ry);
  rotmat(1, 0) = 2 * (xy + rz);
  rotmat(1, 2) = 2 * (yz - rx);
  rotmat(2, 0) = 2 * (zx - ry);
  rotmat(2, 1) = 2 * (yz + rx);

  sotDEBUGOUT(15);
  return rot;
}

VectorQuaternion &VectorQuaternion::conjugate(VectorQuaternion &res) const {
  res.vector(0) = vector(0);
  res.vector(1) = -vector(1);
  res.vector(2) = -vector(2);
  res.vector(3) = -vector(3);
  return res;
}

VectorQuaternion &VectorQuaternion::multiply(const VectorQuaternion &q2,
                                             VectorQuaternion &res) const {
  double &a1 = vector(0);
  double &b1 = vector(1);
  double &c1 = vector(2);
  double &d1 = vector(3);

  double &a2 = q2.vector(0);
  double &b2 = q2.vector(1);
  double &c2 = q2.vector(2);
  double &d2 = q2.vector(3);

  res.vector(0) = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2;
  res.vector(1) = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2;
  res.vector(2) = a1 * c2 + c1 * a2 + d1 * b2 - b1 * d2;
  res.vector(3) = a1 * d2 + d1 * a2 + b1 * c2 - c1 * b2;

  return res;
}
