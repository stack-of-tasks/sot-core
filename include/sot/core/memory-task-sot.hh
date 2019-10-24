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
#include <sot/core/task-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {
namespace dg = dynamicgraph;

class SOT_CORE_EXPORT MemoryTaskSOT : public TaskAbstract::MemoryTaskAbstract,
                                      public dg::Entity {
public: //   protected:
  /* Internal memory to reduce the dynamic allocation at task resolution. */
  dg::Vector err;
  dg::Matrix Jt; //( nJ,mJ );
  dg::Matrix Jp;
  dg::Matrix PJp;

  dg::Matrix Jff;  //( nJ,FF_SIZE ); // Free-floating part
  dg::Matrix Jact; //( nJ,mJ );     // Activated part
  dg::Matrix JK;   //(nJ,mJ);

  dg::Matrix Proj;

  typedef Eigen::JacobiSVD<dg::Matrix> SVD_t;
  SVD_t svd;

public:
  /* mJ is the number of actuated joints, nJ the number of feature in the task,
   * and ffsize the number of unactuated DOF. */
  MemoryTaskSOT(const std::string &name, const Matrix::Index nJ = 0,
                const Matrix::Index mJ = 0, const Matrix::Index ffsize = 0);

  virtual void initMemory(const Matrix::Index nJ, const Matrix::Index mJ,
                          const Matrix::Index ffsize,
                          bool atConstruction = false);

public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display(std::ostream &os) const;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

public: /* --- SIGNALS --- */
  dg::Signal<dg::Matrix, int> jacobianInvSINOUT;
  dg::Signal<dg::Matrix, int> jacobianConstrainedSINOUT;
  dg::Signal<dg::Matrix, int> jacobianProjectedSINOUT;
  dg::Signal<dg::Matrix, int> singularBaseImageSINOUT;
  dg::Signal<unsigned int, int> rankSINOUT;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif // __SOT_MEMORY_TASK_HH
