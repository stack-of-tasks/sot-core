/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <sot/core/integrator-euler.hh>

#include "integrator-euler.t.cpp"

#include <sot/core/integrator-euler-impl.hh>

#ifdef WIN32
IntegratorEulerVectorMatrix::IntegratorEulerVectorMatrix(
    const std::string &name)
    : IntegratorEuler<Vector, Matrix>(name) {}
std::string IntegratorEulerVectorMatrix::getTypeName(void) {
  return "IntegratorEulerVectorMatrix";
}

IntegratorEulerVectorDouble::IntegratorEulerVectorDouble(
    const std::string &name)
    : IntegratorEuler<Vector, double>(name) {}
std::string IntegratorEulerVectorDouble::getTypeName(void) {
  return "IntegratorEulerVectorDouble";
}
#endif // WIN32
