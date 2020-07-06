/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_MEMORY_TASK_HH
#define __SOT_MEMORY_TASK_HH

#include "sot/core/api.hh"
#include <sot/core/matrix-svd.hh>
#include <sot/core/task-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

class SOT_CORE_EXPORT MemoryTaskSOT : public TaskAbstract::MemoryTaskAbstract {
public: //   protected:
  typedef Eigen::Map<Matrix, Eigen::internal::traits<Matrix>::Alignment>
      Kernel_t;
  typedef Eigen::Map<const Matrix, Eigen::internal::traits<Matrix>::Alignment>
      KernelConst_t;

  /* Internal memory to reduce the dynamic allocation at task resolution. */
  dynamicgraph::Vector err, tmpTask, tmpVar, tmpControl;
  dynamicgraph::Matrix Jt; //( nJ,mJ );

  dynamicgraph::Matrix JK; //(nJ,mJ);

  SVD_t svd;
  Kernel_t kernel;

  void resizeKernel(const Matrix::Index r, const Matrix::Index c) {
    if (kernel.rows() != r || kernel.cols() != c) {
      if (kernelMem.size() < r * c)
        kernelMem.resize(r, c);
      new (&kernel) Kernel_t(kernelMem.data(), r, c);
    }
  }

  Kernel_t &getKernel(const Matrix::Index r, const Matrix::Index c) {
    resizeKernel(r, c);
    return kernel;
  }

public:
  /**
   * \param mJ is the number of joints
   * \param nJ the number of feature in the task
   **/
  MemoryTaskSOT(const Matrix::Index nJ = 0, const Matrix::Index mJ = 0);

  void display(std::ostream &os) const;

private:
  void initMemory(const Matrix::Index nJ, const Matrix::Index mJ);

  Matrix kernelMem;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif // __SOT_MEMORY_TASK_HH
