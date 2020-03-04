/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef WIN32
#include <sys/time.h>
#endif
#include <dynamic-graph/linear-algebra.h>
#include <iostream>
#include <sot/core/debug.hh>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/matrix-svd.hh>
#include <sot/core/memory-task-sot.hh>

#ifndef WIN32
#include <unistd.h>
#else
#include <sot/core/utils-windows.hh>
#endif
#include <list>

using namespace dynamicgraph::sot;
using namespace std;

#define INIT_CHRONO(name)                                                      \
  struct timeval t0##_##name, t1##_##name;                                     \
  double dt##_##name
#define START_CHRONO(name)                                                     \
  gettimeofday(&t0##_##name, NULL);                                            \
  sotDEBUG(25) << "t0 " << #name << ": " << t0##_##name.tv_sec << " - "        \
               << t0##_##name.tv_usec << std::endl
#define STOP_CHRONO(name, commentaire)                                         \
  gettimeofday(&t1##_##name, NULL);                                            \
  dt##_##name = ((double)(t1##_##name.tv_sec - t0##_##name.tv_sec) * 1000. +   \
                 (double)(t1##_##name.tv_usec - t0##_##name.tv_usec) / 1000.); \
  sotDEBUG(25) << "t1 " << #name << ": " << t1##_##name.tv_sec << " - "        \
               << t1##_##name.tv_usec << std::endl;                            \
  sotDEBUG(1) << "Time spent " << #name " " commentaire << " = "               \
              << dt##_##name << std::endl

/* ----------------------------------------------------------------------- */
/* ----------------------------------------------------------------------- */
/* ----------------------------------------------------------------------- */
/* ----------------------------------------------------------------------- */

double timerCounter;

// static void inverseCounter( ublas::matrix<double>& matrix,
// dynamicgraph::Matrix& invMatrix )
// {
//   INIT_CHRONO(inv);

//      ublas::matrix<double,ublas::column_major> I = matrix;
//      ublas::matrix<double,ublas::column_major>
//      U(matrix.size1(),matrix.size1());
//      ublas::matrix<double,ublas::column_major>
//      VT(matrix.size2(),matrix.size2()); ublas::vector<double>
//      s(std::min(matrix.size1(),matrix.size2())); char Jobu='A'; /* Compute
//      complete U Matrix */ char Jobvt='A'; /* Compute complete VT Matrix */
//      char Lw; Lw='O'; /* Compute the optimal size for the working vector */

// #ifdef WITH_OPENHRP

//      /* Presupposition: an external function jrlgesvd is defined
//       * and implemented in the calling library.
//       */

//      // get workspace size for svd
//      int lw=-1;
//      {
//        const int m = matrix.size1();
//        const int n = matrix.size2();
//        double vw;
//        int linfo;
//        int lda = std::max(m,n);
//        ublas::matrix<double> tmp(m,n); // matrix is const!
//        jrlgesvd_(&Jobu, &Jobvt, &m, &n, traits::matrix_storage(tmp), &lda,
// 		 0, 0, &m, 0, &n, &vw, &lw, &linfo);
//        lw = int(vw);
//      }
// #else //#ifdef WITH_OPENHRP
//      int lw;
//      if( matrix.size1()>matrix.size2() )
//        {
// 	 ublas::matrix<double,ublas::column_major> matrixtranspose;
// 	 matrixtranspose = trans(matrix);
// 	 lw = lapack::gesvd_work(Lw,Jobu,Jobvt,matrixtranspose);
//        } else {
// 	 lw = lapack::gesvd_work(Lw,Jobu,Jobvt,matrix);
//        }

// #endif //#ifdef WITH_OPENHRP

//      ublas::vector<double> w(lw);
//   gettimeofday(&t0_inv,NULL);
//      lapack::gesvd(Jobu, Jobvt,I,s,U,VT,w);
//   gettimeofday(&t1_inv,NULL);

//      const unsigned int nsv = s.size();
//      ublas::vector<double> sp(nsv);
//      for( unsigned int i=0;i<nsv;++i )
//        if( fabs(s(i))>1e-6 ) sp(i)=1/s(i); else sp(i)=0.;

//      invMatrix.matrix.clear();
//      for( unsigned int i=0;i<VT.size2();++i )
//        for( unsigned int j=0;j<U.size1();++j )
// 	 for( unsigned int k=0;k<nsv;++k )
// 	   invMatrix.matrix(i,j)+=VT(k,i)*sp(k)*U(j,k);

//      dt_inv = ( (t1_inv.tv_sec-t0_inv.tv_sec) * 1000.
// 	       + (t1_inv.tv_usec-t0_inv.tv_usec+0.) / 1000. );
//      timerCounter+=dt_inv;

//      return ;
//    }

/* ----------------------------------------------------------------------- */
/* ----------------------------------------------------------------------- */
/* ----------------------------------------------------------------------- */

int main(int argc, char **argv) {
  if (sotDEBUG_ENABLE(1))
    DebugTrace::openFile();

  //   const unsigned int r=1;
  //   const unsigned int c=30;
  unsigned int r = 1;
  if (argc > 1)
    r = atoi(argv[1]);
  unsigned int c = 30;
  if (argc > 2)
    c = atoi(argv[2]);
  static const int BENCH = 100;

  dynamicgraph::Matrix M(r, c);
  dynamicgraph::Matrix M1(r, c);
  dynamicgraph::Matrix Minv(c, r);

  dynamicgraph::Matrix U, V, S;

  unsigned int nbzeros = 0;
  for (unsigned int j = 0; j < c; ++j) {
    if ((rand() + 1.) / RAND_MAX > .8) {
      for (unsigned int i = 0; i < r; ++i)
        M(i, j) = 0.;
      nbzeros++;
    } else
      for (unsigned int i = 0; i < r; ++i)
        M(i, j) = (rand() + 1.) / RAND_MAX * 2 - 1;
  }
  for (unsigned int i = 0; i < r; ++i)
    for (unsigned int j = 0; j < c; ++j)
      M1(i, j) = M(i, j); //+ ((rand()+1.) / RAND_MAX*2-1) * 1e-28 ;

  // sotDEBUG(15) << dynamicgraph::MATLAB <<"M = "<< M <<endl;
  sotDEBUG(15) << "M1 = " << M1 << endl;
  sotDEBUG(5) << "Nb zeros = " << nbzeros << endl;

  INIT_CHRONO(inv);

  START_CHRONO(inv);
  for (int i = 0; i < BENCH; ++i)
    dynamicgraph::pseudoInverse(M, Minv);
  STOP_CHRONO(inv, "init");
  sotDEBUG(15) << "Minv = " << Minv << endl;

  START_CHRONO(inv);
  for (int i = 0; i < BENCH; ++i)
    dynamicgraph::pseudoInverse(M, Minv);
  STOP_CHRONO(inv, "M+standard");
  cout << dt_inv << endl;

  //   START_CHRONO(inv);
  //   for( int i=0;i<BENCH;++i ) M.pseudoInverse( Minv,1e-6,&U,&S,&V );
  //   STOP_CHRONO(inv,"M+");
  //   sotDEBUG(15) << dynamicgraph::MATLAB <<"Minv = "<< Minv <<endl;

  //   timerCounter=0;
  //   START_CHRONO(inv);
  //   for( int i=0;i<BENCH;++i ) inverseCounter( M1.matrix,Minv );
  //   STOP_CHRONO(inv,"M1+");
  //   sotDEBUG(5) << "Counter: " << timerCounter << endl;
  //   sotDEBUG(15) << dynamicgraph::MATLAB <<"M1inv = "<< Minv <<endl;

  //   dynamicgraph::Matrix M1diag = U.transpose()*M1*V;
  //   timerCounter=0;
  //   START_CHRONO(inv);
  //   for( int i=0;i<BENCH;++i ) inverseCounter( M1diag.matrix,Minv );
  //   STOP_CHRONO(inv,"M1diag+");
  //   sotDEBUG(5) << "Counter: " << timerCounter << endl;
  //   sotDEBUG(8) << dynamicgraph::MATLAB <<"M1diag = "<< M1diag <<endl;
  //   sotDEBUG(15) << dynamicgraph::MATLAB <<"M1diaginv = "<< Minv <<endl;

  START_CHRONO(inv);
  std::list<unsigned int> nonzeros;
  dynamicgraph::Matrix Mcreuse;
  dynamicgraph::Matrix Mcreuseinv;
  for (int ib = 0; ib < BENCH; ++ib) {

    double sumsq;
    unsigned int parc = 0;
    if (!ib) {
      nonzeros.clear();
      for (unsigned int j = 0; j < c; ++j) {
        sumsq = 0.;
        for (unsigned int i = 0; i < r; ++i)
          sumsq += M(i, j) * M(i, j);
        if (sumsq > 1e-6) {
          nonzeros.push_back(j);
          parc++;
        }
      }

      Mcreuse.resize(r, parc);
    }

    // dynamicgraph::Matrix Mcreuse( r,parc );

    parc = 0;
    for (std::list<unsigned int>::iterator iter = nonzeros.begin();
         iter != nonzeros.end(); ++iter) {
      for (unsigned int i = 0; i < r; ++i) {
        Mcreuse(i, parc) = M(i, *iter);
      }
      parc++;
    }

    // dynamicgraph::Matrix Mcreuseinv( Mcreuse.nbCols(),r );
    Mcreuseinv.resize(Mcreuse.cols(), r);
    dynamicgraph::pseudoInverse(Mcreuse, Mcreuseinv);
    parc = 0;
    Minv.fill(0.);
    for (std::list<unsigned int>::iterator iter = nonzeros.begin();
         iter != nonzeros.end(); ++iter) {
      for (unsigned int i = 0; i < r; ++i)
        Minv(*iter, i) = Mcreuseinv(parc, i);
      parc++;
    }

    if (!ib) {
      //	  sotDEBUG(15) << dynamicgraph::MATLAB <<"M = "<< M <<endl;
      //	  sotDEBUG(15) << dynamicgraph::MATLAB <<"Mcreuse = "<< Mcreuse
      //<<endl; 	  sotDEBUG(15) << dynamicgraph::MATLAB <<"Minvnc = "<<
      // Minv
      //<<endl;
    }
  }
  STOP_CHRONO(inv, "M+creuse");
  // sotDEBUG(15) << dynamicgraph::MATLAB <<"Minv = "<< Minv <<endl;

  {

    double sumsq;
    nonzeros.clear();
    unsigned int parc = 0;
    for (unsigned int j = 0; j < c; ++j) {
      sumsq = 0.;
      for (unsigned int i = 0; i < r; ++i)
        sumsq += M(i, j) * M(i, j);
      if (sumsq > 1e-6) {
        nonzeros.push_back(j);
        parc++;
      }
    }

    dynamicgraph::Matrix Mcreuse(r, parc);
    parc = 0;
    for (std::list<unsigned int>::iterator iter = nonzeros.begin();
         iter != nonzeros.end(); ++iter) {
      for (unsigned int i = 0; i < r; ++i) {
        Mcreuse(i, parc) = M(i, *iter);
      }
      parc++;
    }

    dynamicgraph::Matrix Mcreuseinv(Mcreuse.cols(), r);
    START_CHRONO(inv);
    for (int ib = 0; ib < BENCH; ++ib) {
      dynamicgraph::pseudoInverse(Mcreuse, Mcreuseinv);
    }
    STOP_CHRONO(inv, "M+creuseseule");

    parc = 0;
    Minv.fill(0.);
    for (std::list<unsigned int>::iterator iter = nonzeros.begin();
         iter != nonzeros.end(); ++iter) {
      for (unsigned int i = 0; i < r; ++i)
        Minv(*iter, i) = Mcreuseinv(parc, i);
      parc++;
    }

    {
      //	  sotDEBUG(15) << dynamicgraph::MATLAB <<"M = "<< M <<endl;
      //	  sotDEBUG(15) << dynamicgraph::MATLAB <<"Mcreuse = "<< Mcreuse
      //<<endl; 	  sotDEBUG(15) << dynamicgraph::MATLAB <<"Minvnc = "<<
      // Minv
      //<<endl;
    }
  }

  return 0;
}
