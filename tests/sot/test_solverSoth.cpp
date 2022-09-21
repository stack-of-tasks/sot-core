/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#define VP_DEBUG_MODE 45
#include <fstream>
#include <sot/core/debug.hh>
#include <sot/core/solver-hierarchical-inequalities.hh>

#ifndef WIN32
#include <sys/time.h>
#else /*WIN32*/
#include <sot/core/utils-windows.hh>
#endif /*WIN32*/
//#define WITH_CHRONO

using namespace dynamicgraph::sot;

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */

void parseTest(const std::string filename) {
  using namespace std;
  std::ifstream off(filename.c_str());
  std::string bs;

  bubMatrix Rh;
  SolverHierarchicalInequalities::ConstraintList constraintH;
  std::vector<bubMatrix> Jes;
  std::vector<bubVector> ees;
  std::vector<bubMatrix> Jis;
  std::vector<bubVector> eiInfs;
  std::vector<bubVector> eiSups;
  std::vector<ConstraintMem::BoundSideVector> bounds;

  int nJ;
  int me, mi;
  bubMatrix Je, Ji;
  bubVector ee, eiInf, eiSup;
  ConstraintMem::BoundSideVector eiBoundSide;

  off >> bs;
  if (bs != "variable") {
    cerr << "!! '" << bs << "'" << endl;
    return;
  }
  off >> bs;
  if (bs != "size") {
    cerr << "!! '" << bs << "'" << endl;
    return;
  }
  off >> nJ;

  sotRotationComposedInExtenso Qh(nJ);
  std::deque<SolverHierarchicalInequalities *> solvers;

  for (unsigned int level = 0;; ++level) {
    /* --- Parse egalities --- */
    off >> bs;
    if (bs == "end") {
      break;
    } else if (bs != "level") {
      cerr << "!! '" << bs << "'" << endl;
      return;
    }
    off >> bs;
    if (bs != "equalities") {
      cerr << "!! '" << bs << "'" << endl;
      return;
    }
    off >> me;
    Je.resize(me, nJ);
    ee.resize(me);
    if (me > 0)
      for (int i = 0; i < me; ++i) {
        for (int j = 0; j < nJ; ++j) off >> Je(i, j);
        off >> ee(i);
      }

    /* --- Parse inequalities --- */
    off >> bs;
    if (bs != "inequalities") {
      cerr << "!! '" << bs << "'" << endl;
      return;
    }
    off >> mi;
    Ji.resize(mi, nJ);
    eiInf.resize(mi);
    eiSup.resize(mi);
    eiBoundSide.resize(mi);
    if (mi > 0)
      for (int i = 0; i < mi; ++i) {
        for (int j = 0; j < nJ; ++j) off >> Ji(i, j);
        std::string number;
        eiBoundSide[i] = ConstraintMem::BOUND_VOID;
        off >> number;
        if (number != "X") {
          eiBoundSide[i] = (ConstraintMem::BoundSideType)(
              eiBoundSide[i] | ConstraintMem::BOUND_INF);
          eiInf(i) = atof(number.c_str());
        } else {
          eiInf(i) = 1e-66;
        }
        off >> number;
        if (number != "X") {
          eiBoundSide[i] = (ConstraintMem::BoundSideType)(
              eiBoundSide[i] | ConstraintMem::BOUND_SUP);
          eiSup(i) = atof(number.c_str());
        } else {
          eiSup(i) = 1e-66;
        }
      }

    struct timeval t0, t1;
    double dtsolver;
    sotDEBUG(15)
        << "/* ----------------------------------------------------- */"
        << std::endl;
    sotDEBUG(15)
        << "/* ----------------------------------------------------- */"
        << std::endl;
    sotDEBUG(15)
        << "/* ----------------------------------------------------- */"
        << std::endl;
    sotDEBUG(5) << "ee" << level << " = " << (MATLAB)ee << endl;
    sotDEBUG(5) << "eiInf" << level << " = " << (MATLAB)eiInf << endl;
    sotDEBUG(5) << "eiSup" << level << " = " << (MATLAB)eiSup << endl;
    sotDEBUG(5) << "Je" << level << " = " << (MATLAB)Je << endl;
    sotDEBUG(5) << "Ji" << level << " = " << (MATLAB)Ji << endl;
    gettimeofday(&t0, NULL);

    Jes.push_back(Je);
    Jis.push_back(Ji);
    ees.push_back(ee);
    eiInfs.push_back(eiInf);
    eiSups.push_back(eiSup);
    bounds.push_back(eiBoundSide);

    sotDEBUG(1) << "--- Level " << level << std::endl;
    SolverHierarchicalInequalities *solver =
        new SolverHierarchicalInequalities(nJ, Qh, Rh, constraintH);
    solver->initConstraintSize(Je.size1() + Ji.size1());
    if (solvers.size() == 0)
      solver->setInitialConditionVoid();
    else {
      SolverHierarchicalInequalities *solverPrec = solvers.back();
      solver->setInitialCondition(solverPrec->u0, solverPrec->rankh);
    }
    solver->recordInitialConditions();
    solvers.push_back(solver);

    solver->solve(Je, ee, Ji, eiInf, eiSup, eiBoundSide);

    /*!*/ gettimeofday(&t1, NULL);
    /*!*/ dtsolver = (t1.tv_sec - t0.tv_sec) * 1000. +
                     (t1.tv_usec - t0.tv_usec + 0.) / 1000.;
    /*!*/ sotDEBUG(1) << "u" << level << " = " << (MATLAB)solver->u0 << endl;
    /*!*/ cout << "dtSOLV_" << level << " = " << dtsolver << "%ms" << endl;

    solver->computeDifferentialCondition();
#ifdef VP_DEBUG
    solver->printDifferentialCondition(sotDEBUGFLOW.outputbuffer);
#endif
  }

  for (unsigned int repet = 0; repet < 1; ++repet) {
    cout << "Repet = " << repet << std::endl;
    struct timeval t0, t1;
    double dtsolver;
    constraintH.resize(0);
    Rh *= 0;
    Qh.clear();
    for (unsigned int level = 0; level < solvers.size(); ++level) {
      gettimeofday(&t0, NULL);
      sotDEBUG(1) << "--- Level " << level << std::endl;
      SolverHierarchicalInequalities *solver = solvers[level];
      if (level == 0)
        solver->setInitialConditionVoid();
      else {
        SolverHierarchicalInequalities *solverPrec = solvers[level - 1];
        solver->setInitialCondition(solverPrec->u0, solverPrec->rankh);
      }

      solver->recordInitialConditions();
#ifdef WITH_WARM_START
      solver->warmStart();
#endif
      /*!*/ gettimeofday(&t1, NULL);
      /*!*/ dtsolver = (t1.tv_sec - t0.tv_sec) * 1000. +
                       (t1.tv_usec - t0.tv_usec + 0.) / 1000.;
      /*!*/ sotDEBUG(1) << "dtWS_" << level << " = " << dtsolver << "%ms"
                        << endl;
      /*!*/ cout << " " << dtsolver;
      /*!*/ gettimeofday(&t0, NULL);

      solver->solve(Jes[level], ees[level], Jis[level], eiInfs[level],
                    eiSups[level], bounds[level], solver->getSlackActiveSet());
      sotDEBUG(1) << "u" << level << " = " << (MATLAB)solver->u0 << endl;
      /*!*/ gettimeofday(&t1, NULL);
      /*!*/ dtsolver = (t1.tv_sec - t0.tv_sec) * 1000. +
                       (t1.tv_usec - t0.tv_usec + 0.) / 1000.;
      /*!*/ cout << " " << dtsolver;
      /*!*/ sotDEBUG(1) << "dtSOLV_" << level << " = " << dtsolver << "%ms"
                        << endl;
      /*!*/ gettimeofday(&t0, NULL);

      solver->computeDifferentialCondition();
#ifdef VP_DEBUG
      solver->printDifferentialCondition(sotDEBUGFLOW.outputbuffer);
#endif

      /*!*/ gettimeofday(&t1, NULL);
      /*!*/ dtsolver = (t1.tv_sec - t0.tv_sec) * 1000. +
                       (t1.tv_usec - t0.tv_usec + 0.) / 1000.;
      /*!*/ cout << " " << dtsolver;
      /*!*/ sotDEBUG(1) << "dtREC_" << level << " = " << dtsolver << "%ms"
                        << endl;
    }
    std::cout << std::endl;
  }

  for (unsigned int level = 0; level < solvers.size(); ++level) {
    delete solvers[level];
    solvers[level] = NULL;
  }
  solvers.clear();
}

/* ---------------------------------------------------------- */
void deparse(std::vector<bubMatrix> Jes, std::vector<bubVector> ees,
             std::vector<bubMatrix> Jis, std::vector<bubVector> eiInfs,
             std::vector<bubVector> eiSups,
             std::vector<ConstraintMem::BoundSideVector> bounds) {
  using namespace std;
  cout << "variable size " << Jes[0].size2() << endl;

  for (unsigned int i = 0; i < Jes.size(); ++i) {
    bubMatrix &Je = Jes[i];
    bubMatrix &Ji = Jis[i];
    bubVector &ee = ees[i];
    bubVector &eiInf = eiInfs[i];
    bubVector &eiSup = eiSups[i];
    ConstraintMem::BoundSideVector &boundSide = bounds[i];

    cout << endl
         << endl
         << "level" << endl
         << endl
         << "equalities " << ee.size() << endl;
    if (ee.size() > 0)
      for (unsigned int i = 0; i < ee.size(); ++i) {
        for (unsigned int j = 0; j < Je.size2(); ++j) cout << Je(i, j) << " ";
        cout << "\t" << ee(i) << endl;
      }

    unsigned int nbIneq = 0;
    for (unsigned int i = 0; i < boundSide.size(); ++i) {
      if (boundSide[i] & ConstraintMem::BOUND_INF) nbIneq++;
      if (boundSide[i] & ConstraintMem::BOUND_SUP) nbIneq++;
    }

    cout << endl << "inequalities " << nbIneq << endl;
    if (eiInf.size() > 0)
      for (unsigned int i = 0; i < eiInf.size(); ++i) {
        if (boundSide[i] & ConstraintMem::BOUND_INF) {
          for (unsigned int j = 0; j < Ji.size2(); ++j)
            cout << -Ji(i, j) << " ";
          cout << "\t"
               << " X " << -eiInf(i) << endl;
        }
        if (boundSide[i] & ConstraintMem::BOUND_SUP) {
          for (unsigned int j = 0; j < Ji.size2(); ++j) cout << Ji(i, j) << " ";
          cout << "\t"
               << " X " << eiSup(i) << endl;
        }
      }
  }
  sotDEBUG(15) << endl << endl << "end" << endl;
}

void convertDoubleToSingle(const std::string filename) {
  using namespace std;
  std::ifstream off(filename.c_str());
  std::string bs;

  int nJ;
  int me, mi;
  bubMatrix Je, Ji;
  bubVector ee, eiInf, eiSup;
  ConstraintMem::BoundSideVector eiBoundSide;

  off >> bs;
  if (bs != "variable") {
    cerr << "!! '" << bs << "'" << endl;
    return;
  }
  off >> bs;
  if (bs != "size") {
    cerr << "!! '" << bs << "'" << endl;
    return;
  }
  off >> nJ;

  /* --- Set config file --- */
  std::vector<bubMatrix> Jes;
  std::vector<bubVector> ees;
  std::vector<bubMatrix> Jis;
  std::vector<bubVector> eiInfs;
  std::vector<bubVector> eiSups;
  std::vector<ConstraintMem::BoundSideVector> bounds;

  for (unsigned int level = 0;; ++level) {
    off >> bs;
    if (bs == "end") {
      break;
    } else if (bs != "level") {
      cerr << "!! '" << bs << "'" << endl;
      return;
    }
    off >> bs;
    if (bs != "equalities") {
      cerr << "!! '" << bs << "'" << endl;
      return;
    }
    off >> me;
    Je.resize(me, nJ);
    ee.resize(me);
    if (me > 0)
      for (int i = 0; i < me; ++i) {
        for (int j = 0; j < nJ; ++j) off >> Je(i, j);
        off >> ee(i);
      }

    off >> bs;
    if (bs != "inequalities") {
      cerr << "!! '" << bs << "'" << endl;
      return;
    }
    off >> mi;
    Ji.resize(mi, nJ);
    eiInf.resize(mi);
    eiSup.resize(mi);
    eiBoundSide.resize(mi);
    if (mi > 0)
      for (int i = 0; i < mi; ++i) {
        for (int j = 0; j < nJ; ++j) off >> Ji(i, j);
        std::string number;
        eiBoundSide[i] = ConstraintMem::BOUND_VOID;
        off >> number;  // std::cout << "toto '" << number << "'" << std::endl;
        if (number != "X") {
          eiBoundSide[i] = (ConstraintMem::BoundSideType)(
              eiBoundSide[i] | ConstraintMem::BOUND_INF);
          eiInf(i) = atof(number.c_str());
        } else {
          eiInf(i) = 1e-66;
        }
        off >> number;
        if (number != "X") {
          eiBoundSide[i] = (ConstraintMem::BoundSideType)(
              eiBoundSide[i] | ConstraintMem::BOUND_SUP);
          eiSup(i) = atof(number.c_str());
        } else {
          eiSup(i) = 1e-66;
        }
      }
    // Ji*=-1; eiInf*=-1;

    Jes.push_back(Je);
    ees.push_back(ee);
    Jis.push_back(Ji);
    eiInfs.push_back(eiInf);
    eiSups.push_back(eiSup);
    bounds.push_back(eiBoundSide);
  }

  deparse(Jes, ees, Jis, eiInfs, eiSups, bounds);
}

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
void randBound(ConstraintMem::BoundSideVector &M, const unsigned int row) {
  M.resize(row);
  for (unsigned int i = 0; i < row; ++i) {
    double c = ((rand() + 0.0) / RAND_MAX * 2) - 1.;
    if (c < 0)
      M[i] = ConstraintMem::BOUND_INF;
    else
      M[i] = ConstraintMem::BOUND_SUP;
  }
}

void randTest(const unsigned int nJ, const bool enableSolve[]) {
  bubVector eiInf0(nJ), ee0(1), eiSup0(nJ);
  bubMatrix Ji0(nJ, nJ), Je0(1, nJ);
  ConstraintMem::BoundSideVector bound0(nJ, ConstraintMem::BOUND_INF);
  Ji0.assign(bub::identity_matrix<double>(nJ));
  eiInf0.assign(bub::zero_vector<double>(nJ));
  Je0.assign(bub::zero_matrix<double>(1, nJ));
  // std::fill(ee0.data().begin(),ee0.data().end(),1);
  ee0(0) = 0;
  Je0(0, 0) = 1;

  bubVector ee1, eiInf1, eiSup1;
  bubMatrix Je1, Ji1;
  ConstraintMem::BoundSideVector bound1;
  randVector(ee1, 3);
  sotDEBUG(15) << "ee1 = " << (MATLAB)ee1 << std::endl;
  randVector(eiInf1, 3);
  sotDEBUG(15) << "eiInf1 = " << (MATLAB)eiInf1 << std::endl;
  randVector(eiSup1, 3);
  sotDEBUG(15) << "eiSup1 = " << (MATLAB)eiSup1 << std::endl;
  randBound(bound1, 3);
  randMatrix(Je1, 3,
             nJ);  // sotDEBUG(15) << "Je1 = " << (MATLAB)Je1 << std::endl;
  randMatrix(Ji1, 3, nJ);
  sotDEBUG(15) << "Ji1 = " << (MATLAB)Ji1 << std::endl;
  bubMatrix xhi;
  randMatrix(xhi, 1, 3);
  xhi(0, 2) = 0;
  bub::project(Je1, bub::range(2, 3), bub::range(0, nJ)) = bub::prod(xhi, Je1);
  sotDEBUG(15) << "Je1 = " << (MATLAB)Je1 << std::endl;
  // randMatrix(xhi,3,3);
  //   bub::noalias(Ji1) = bub::prod(xhi,Je1);
  //  sotDEBUG(15) << "Ji1 = " << (MATLAB)Ji1 << std::endl;

  bubVector ee2, eiInf2, eiSup2;
  bubMatrix Je2, Ji2;
  ConstraintMem::BoundSideVector bound2;
  randVector(ee2, 3);
  sotDEBUG(15) << "ee2 = " << (MATLAB)ee2 << std::endl;
  randVector(eiInf2, 3);
  sotDEBUG(15) << "eiInf2 = " << (MATLAB)eiInf2 << std::endl;
  randVector(eiSup2, 3);
  sotDEBUG(15) << "eiSup2 = " << (MATLAB)eiSup2 << std::endl;
  randBound(bound1, 3);
  randMatrix(Je2, 3, nJ);
  sotDEBUG(15) << "Je2 = " << (MATLAB)Je2 << std::endl;
  randMatrix(Ji2, 3, nJ);
  sotDEBUG(15) << "Ji2 = " << (MATLAB)Ji2 << std::endl;

  bubVector ee3, eiInf3, eiSup3;
  bubMatrix Je3, Ji3;
  ConstraintMem::BoundSideVector bound3;
  randVector(ee3, 3);
  sotDEBUG(15) << "ee3 = " << (MATLAB)ee3 << std::endl;
  randVector(eiInf3, 3);
  sotDEBUG(15) << "eiInf3 = " << (MATLAB)eiInf3 << std::endl;
  randVector(eiSup3, 3);
  sotDEBUG(15) << "eiSup3 = " << (MATLAB)eiSup3 << std::endl;
  randBound(bound1, 3);
  randMatrix(Je3, 3, nJ);
  sotDEBUG(15) << "Je3 = " << (MATLAB)Je3 << std::endl;
  randMatrix(Ji3, 3, nJ);
  sotDEBUG(15) << "Ji3 = " << (MATLAB)Ji3 << std::endl;

  bubVector ee4(nJ), eiInf4(1), eiSup4(1);
  bubMatrix Je4(nJ, nJ), Ji4(1, nJ);
  ConstraintMem::BoundSideVector bound4(1, ConstraintMem::BOUND_INF);
  Je4.assign(bub::identity_matrix<double>(nJ));
  ee4.assign(bub::zero_vector<double>(nJ));
  Ji4.assign(bub::zero_matrix<double>(1, nJ));
  eiInf4.assign(bub::zero_vector<double>(1));
  eiSup4.assign(bub::zero_vector<double>(1));

  /* --- Set config file --- */
  std::vector<bubMatrix> Jes;
  std::vector<bubVector> ees;
  std::vector<bubMatrix> Jis;
  std::vector<bubVector> eiInfs;
  std::vector<bubVector> eiSups;
  std::vector<ConstraintMem::BoundSideVector> bounds;

  if (enableSolve[0]) {
    Jes.push_back(Je0);
    ees.push_back(ee0);
    Jis.push_back(Ji0);
    eiInfs.push_back(eiInf0);
    eiSups.push_back(eiSup0);
    bounds.push_back(bound0);
  }

  if (enableSolve[1]) {
    Jes.push_back(Je1);
    ees.push_back(ee1);
    Jis.push_back(Ji1);
    eiInfs.push_back(eiInf1);
    eiSups.push_back(eiSup1);
    bounds.push_back(bound1);
  }
  if (enableSolve[2]) {
    Jes.push_back(Je2);
    ees.push_back(ee2);
    Jis.push_back(Ji2);
    eiInfs.push_back(eiInf2);
    eiSups.push_back(eiSup2);
    bounds.push_back(bound2);
  }
  if (enableSolve[3]) {
    Jes.push_back(Je3);
    ees.push_back(ee3);
    Jis.push_back(Ji3);
    eiInfs.push_back(eiInf3);
    eiSups.push_back(eiSup3);
    bounds.push_back(bound3);
  }
  if (enableSolve[4]) {
    Jes.push_back(Je4);
    ees.push_back(ee4);
    Jis.push_back(Ji4);
    eiInfs.push_back(eiInf4);
    eiSups.push_back(eiSup4);
    bounds.push_back(bound4);
  }
  deparse(Jes, ees, Jis, eiInfs, eiSups, bounds);

  sotRotationComposedInExtenso Qh(nJ);
  bubMatrix Rh;
  SolverHierarchicalInequalities::ConstraintList constraintH;
  SolverHierarchicalInequalities solver(nJ, Qh, Rh, constraintH);
  solver.initConstraintSize((40 + 6 + 6 + 6 + 40) + 2);

  /* ---------------------------------------------------------- */

  if (enableSolve[0]) solver.solve(Je0, ee0, Ji0, eiInf0, eiSup0, bound0);
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  if (enableSolve[1]) solver.solve(Je1, ee1, Ji1, eiInf1, eiSup1, bound1);
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  if (enableSolve[2]) solver.solve(Je2, ee2, Ji2, eiInf2, eiSup2, bound2);
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  if (enableSolve[3]) solver.solve(Je3, ee3, Ji3, eiInf3, eiSup3, bound3);
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  sotDEBUG(15) << "/* ----------------------------------------------------- */"
               << std::endl;
  if (enableSolve[4]) solver.solve(Je4, ee4, Ji4, eiInf4, eiSup4, bound4);

  /* ---------------------------------------------------------- */

  return;
}

/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
/* ---------------------------------------------------------- */
int main(void) {
  //  convertDoubleToSingle("/home/nmansard/src/StackOfTasks/tests/tools/testFR.txt");
  //  exit(0);

#ifndef VP_DEBUG
#ifndef WITH_CHRONO
  for (int i = 0; i < 10; ++i)
#endif
#endif
    parseTest("/home/nmansard/src/StackOfTasks/tests//t.txt");
  //   bool enable [5] ={ 1,1,0,0,0};
  //   randTest(9,enable );
}
