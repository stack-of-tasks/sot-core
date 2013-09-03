/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SOT_MEMORY_TASK_HH
#define __SOT_MEMORY_TASK_HH


#include <sot/core/task-abstract.hh>
#include "sot/core/api.hh"


extern "C"
{
  void dgesvd_(char const* jobu, char const* jobvt,
	       int const* m, int const* n, double* a, int const* lda,
	       double* s, double* u, int const* ldu,
	       double* vt, int const* ldvt,
	       double* work, int const* lwork, int* info);
}

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*** Pseudo inverse ***/
dg::Matrix& pseudoInverse( dg::Matrix& matrix,
                                dg::Matrix& invMatrix,
			        const double threshold = 1e-6,
			        dg::Matrix* Uref = NULL,
			        dg::Vector* Sref = NULL,
			        dg::Matrix* Vref = NULL)
{
  unsigned int NR,NC;
  bool toTranspose;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> I;
  if( matrix.rows()>matrix.cols() )
  {
    toTranspose=false ;  NR=matrix.rows(); NC=matrix.cols();
    I=matrix;
    invMatrix.resize(I.cols(),I.rows());
  }
  else
  {
    toTranspose=true; NR=matrix.cols(); NC=matrix.rows();
    I = matrix.transpose();
    invMatrix.resize(I.cols(),I.rows()); // Resize the inv of the transpose.
  }

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> U(NR,NR);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> VT(NC,NC);
  Eigen::Matrix<double, Eigen::Dynamic, 1> s(std::min(NR,NC));
  char Jobu='A'; // Compute complete U Matrix
  char Jobvt='A'; // Compute complete VT Matrix
  char Lw; Lw='O'; // Compute the optimal size for the working vector


  {
    double vw;
    int lw=-1;

    int linfo; const int n=NR,m=NC;
    int lda = std::max(n,m);;
    int lu = NR;
    int lvt = NC;

    dgesvd_(&Jobu, &Jobvt, &m, &n,
		I.data(), &lda,
		0, 0, &m, 0, &n, &vw, &lw, &linfo);
    lw = int(vw)+5;

    Eigen::Matrix<double, Eigen::Dynamic, 1> w(lw);
    dgesvd_(&Jobu, &Jobvt,&n,&m,
		      I.data(),
		      &lda,
		      s.data(),
		      U.data(),
		      &lu,
		      VT.data(),
		      &lvt,
		      w.data(),&lw,&linfo);

  }


  const unsigned int nsv = s.size();
  unsigned int rankJ = 0;
  Eigen::Matrix<double, Eigen::Dynamic, 1> sp(nsv);
  for( unsigned int i=0;i<nsv;++i )
    if( fabs(s(i))>threshold ) { sp(i)=1/s(i); rankJ++; }
    else sp(i)=0.;
  invMatrix.Zero(invMatrix.rows(), invMatrix.cols());
  {
    double * pinv = invMatrix.data();
    double * uptr;
    double * uptrRow;
    double * vptr;
    double * vptrRow = VT.data();

    double * spptr;

    for( unsigned int i=0;i<NC;++i )
    {
      uptrRow = U.data();
      for( unsigned int j=0;j<NR;++j )
      {
        uptr = uptrRow;  vptr = vptrRow;
        spptr = sp.data();
        for( unsigned int k=0;k<rankJ;++k )
	{
	  (*pinv) += (*vptr) * (*spptr) * (*uptr);
	  uptr+=NR; vptr++; spptr++;
	}
	pinv++; uptrRow++;
      }
      vptrRow += NC;
    }
  }
  if( toTranspose )
  {
    invMatrix.transposeInPlace();
    if( Uref ) *Uref = VT;
    if( Vref ) *Vref = U.transpose();
    if( Sref ) *Sref = s;
  }
  else
  {
    if( Uref ) *Uref = U;
    if( Vref ) *Vref = VT.transpose();
    if( Sref ) *Sref = s;
  }
  return invMatrix;
}

/**** CLASS ***/
class SOT_CORE_EXPORT MemoryTaskSOT
: public TaskAbstract::MemoryTaskAbstract, public dg::Entity
{
 public://   protected:
  /* Internal memory to reduce the dynamic allocation at task resolution. */
  dg::Matrix Jt;  //( nJ,mJ );
  dg::Matrix Jp;
  dg::Matrix PJp;

  dg::Matrix Jff; //( nJ,FF_SIZE ); // Free-floating part
  dg::Matrix Jact; //( nJ,mJ );     // Activated part
  dg::Matrix JK; //(nJ,mJ);

  dg::Matrix U,V;
  dg::Vector S;

 public:
  /* mJ is the number of actuated joints, nJ the number of feature in the task,
   * and ffsize the number of unactuated DOF. */
  MemoryTaskSOT( const std::string & name,const unsigned int nJ=0,
                    const unsigned int mJ=0,const unsigned int ffsize =0 );

  virtual void initMemory( const unsigned int nJ,
                           const unsigned int mJ,
                           const unsigned int ffsize,
			   bool atConstruction = false);

 public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display( std::ostream& os ) const;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public: /* --- SIGNALS --- */
  dg::Signal< dg::Matrix,int > jacobianInvSINOUT;
  dg::Signal< dg::Matrix,int > jacobianConstrainedSINOUT;
  dg::Signal< dg::Matrix,int > jacobianProjectedSINOUT;
  dg::Signal< dg::Matrix,int > singularBaseImageSINOUT;
  dg::Signal< unsigned int,int > rankSINOUT;

 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
			    std::ostream& os );
};

} /* namespace sot */} /* namespace dynamicgraph */

#endif // __SOT_MEMORY_TASK_HH
