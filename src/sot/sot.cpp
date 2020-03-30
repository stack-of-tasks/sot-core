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

//#define VP_DEBUG
//#define VP_DEBUG_MODE 45
#include <sot/core/debug.hh>

/* SOT */
#ifdef VP_DEBUG
class sotSOT__INIT {
public:
  sotSOT__INIT(void) { dynamicgraph::sot::DebugTrace::openFile(); }
};
sotSOT__INIT sotSOT_initiator;
#endif //#ifdef VP_DEBUG

#include <sot/core/sot.hh>

#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <sot/core/factory.hh>
#include <sot/core/feature-posture.hh>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/matrix-svd.hh>
#include <sot/core/memory-task-sot.hh>
#include <sot/core/pool.hh>
#include <sot/core/task.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#include "../src/sot/sot-command.h"

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Sot, "SOT");

const double Sot::INVERSION_THRESHOLD_DEFAULT = 1e-4;

/* --------------------------------------------------------------------- */
/* --- CONSTRUCTION ---------------------------------------------------- */
/* --------------------------------------------------------------------- */
Sot::Sot(const std::string &name)
    : Entity(name), stack(), nbJoints(0), enablePostureTaskAcceleration(false),
      q0SIN(NULL, "sotSOT(" + name + ")::input(double)::q0"),
      proj0SIN(NULL, "sotSOT(" + name + ")::input(double)::proj0"),
      inversionThresholdSIN(NULL,
                            "sotSOT(" + name + ")::input(double)::damping"),
      controlSOUT(boost::bind(&Sot::computeControlLaw, this, _1, _2),
                  inversionThresholdSIN << q0SIN << proj0SIN,
                  "sotSOT(" + name + ")::output(vector)::control") {
  inversionThresholdSIN = INVERSION_THRESHOLD_DEFAULT;

  signalRegistration(inversionThresholdSIN << controlSOUT << q0SIN << proj0SIN);

  // Commands
  //
  std::string docstring;

  docstring = "    \n"
              "    setNumberDofs.\n"
              "    \n"
              "      Input:\n"
              "        - a positive integer : number of degrees of freedom of "
              "the robot.\n"
              "    \n";
  addCommand("setSize", new dynamicgraph::command::Setter<Sot, unsigned int>(
                            *this, &Sot::defineNbDof, docstring));

  docstring = "    \n"
              "    getNumberDofs.\n"
              "    \n"
              "      Output:\n"
              "        - a positive integer : number of degrees of freedom of "
              "the robot.\n"
              "    \n";
  addCommand("getSize",
             new dynamicgraph::command::Getter<Sot, const unsigned int &>(
                 *this, &Sot::getNbDof, docstring));

  addCommand("enablePostureTaskAcceleration",
             dynamicgraph::command::makeDirectSetter(
                 *this, &enablePostureTaskAcceleration,
                 dynamicgraph::command::docDirectSetter(
                     "option to bypass SVD computation for the posture task at "
                     "the last"
                     "level",
                     "boolean")));

  addCommand("isPostureTaskAccelerationEnabled",
             dynamicgraph::command::makeDirectGetter(
                 *this, &enablePostureTaskAcceleration,
                 dynamicgraph::command::docDirectGetter(
                     "option to bypass SVD computation for the posture task at "
                     "the last"
                     "level",
                     "boolean")));

  docstring = "    \n"
              "    push a task into the stack.\n"
              "    \n"
              "      Input:\n"
              "        - a string : Name of the task.\n"
              "    \n";
  addCommand("push", new command::classSot::Push(*this, docstring));

  docstring = "    \n"
              "    remove a task into the stack.\n"
              "    \n"
              "      Input:\n"
              "        - a string : Name of the task.\n"
              "    \n";
  addCommand("remove", new command::classSot::Remove(*this, docstring));

  docstring = "    \n"
              "    up a task into the stack.\n"
              "    \n"
              "      Input:\n"
              "        - a string : Name of the task.\n"
              "    \n";
  addCommand("up", new command::classSot::Up(*this, docstring));

  docstring = "    \n"
              "    down a task into the stack.\n"
              "    \n"
              "      Input:\n"
              "        - a string : Name of the task.\n"
              "    \n";
  addCommand("down", new command::classSot::Down(*this, docstring));

  // Display
  docstring = "    \n"
              "    display the list of tasks pushed inside the stack.\n"
              "    \n";
  addCommand("display", new command::classSot::Display(*this, docstring));

  // Clear
  docstring = "    \n"
              "    clear the list of tasks pushed inside the stack.\n"
              "    \n";
  addCommand("clear", new command::classSot::Clear(*this, docstring));

  // List
  docstring = "    \n"
              "    returns the list of tasks pushed inside the stack.\n"
              "    \n";
  addCommand("list", new command::classSot::List(*this, docstring));
}

/* --------------------------------------------------------------------- */
/* --- STACK MANIPULATION --- */
/* --------------------------------------------------------------------- */
void Sot::push(TaskAbstract &task) {
  if (nbJoints == 0)
    throw std::logic_error("Set joint size of " + getClassName() + " \"" +
                           getName() + "\" first");
  stack.push_back(&task);
  controlSOUT.addDependency(task.taskSOUT);
  controlSOUT.addDependency(task.jacobianSOUT);
  controlSOUT.setReady();
}
TaskAbstract &Sot::pop(void) {
  TaskAbstract *res = stack.back();
  stack.pop_back();
  controlSOUT.removeDependency(res->taskSOUT);
  controlSOUT.removeDependency(res->jacobianSOUT);
  controlSOUT.setReady();
  return *res;
}
bool Sot::exist(const TaskAbstract &key) {
  StackType::iterator it;
  for (it = stack.begin(); stack.end() != it; ++it) {
    if (*it == &key) {
      return true;
    }
  }
  return false;
}
void Sot::remove(const TaskAbstract &key) {
  bool find = false;
  StackType::iterator it;
  for (it = stack.begin(); stack.end() != it; ++it) {
    if (*it == &key) {
      find = true;
      break;
    }
  }
  if (!find) {
    return;
  }

  stack.erase(it);
  removeDependency(key);
}

void Sot::removeDependency(const TaskAbstract &key) {
  controlSOUT.removeDependency(key.taskSOUT);
  controlSOUT.removeDependency(key.jacobianSOUT);
  controlSOUT.setReady();
}

void Sot::up(const TaskAbstract &key) {
  bool find = false;
  StackType::iterator it;
  for (it = stack.begin(); stack.end() != it; ++it) {
    if (*it == &key) {
      find = true;
      break;
    }
  }
  if (stack.begin() == it) {
    return;
  }
  if (!find) {
    return;
  }

  StackType::iterator pos = it;
  pos--;
  TaskAbstract *task = *it;
  stack.erase(it);
  stack.insert(pos, task);
  controlSOUT.setReady();
}
void Sot::down(const TaskAbstract &key) {
  bool find = false;
  StackType::iterator it;
  for (it = stack.begin(); stack.end() != it; ++it) {
    if (*it == &key) {
      find = true;
      break;
    }
  }
  if (stack.end() == it) {
    return;
  }
  if (!find) {
    return;
  }

  StackType::iterator pos = it;
  pos++;
  TaskAbstract *task = *it;
  stack.erase(it);
  if (stack.end() == pos) {
    stack.push_back(task);
  } else {
    pos++;
    stack.insert(pos, task);
  }
  controlSOUT.setReady();
}

void Sot::clear(void) {
  for (StackType::iterator it = stack.begin(); stack.end() != it; ++it) {
    removeDependency(**it);
  }
  stack.clear();
  controlSOUT.setReady();
}

void Sot::defineNbDof(const unsigned int &nbDof) {
  nbJoints = nbDof;
  controlSOUT.setReady();
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const Matrix &computeJacobianActivated(TaskAbstract *Ta, Task *T, Matrix &Jmem,
                                       const int &iterTime) {
  if (T != NULL) {
    const Flags &controlSelec = T->controlSelectionSIN(iterTime);
    sotDEBUG(25) << "Control selection = " << controlSelec << endl;
    if (controlSelec) {
      if (!controlSelec) {
        Jmem = Ta->jacobianSOUT.accessCopy();
        for (int i = 0; i < Jmem.cols(); ++i)
          if (!controlSelec(i))
            Jmem.col(i).setZero();
        return Jmem;
      } else
        return Ta->jacobianSOUT.accessCopy();
    } else {
      sotDEBUG(15) << "Task not activated." << endl;
      const Matrix &Jac = Ta->jacobianSOUT.accessCopy();
      Jmem = Matrix::Zero(Jac.rows(), Jac.cols());
      return Jmem;
    }
  } else /* No selection specification: nothing to do. */
    return Ta->jacobianSOUT.accessCopy();
}

typedef MemoryTaskSOT::Kernel_t Kernel_t;
typedef MemoryTaskSOT::KernelConst_t KernelConst_t;

template <typename MapType, typename MatrixType>
inline void makeMap(MapType &map, MatrixType &m) {
  // There is not memory allocation here.
  // See https://eigen.tuxfamily.org/dox/group__TutorialMapClass.html
  new (&map) KernelConst_t(m.data(), m.rows(), m.cols());
}

void updateControl(MemoryTaskSOT *mem, const Matrix::Index rankJ,
                   bool has_kernel, const KernelConst_t &kernel,
                   Vector &control) {
  const SVD_t &svd(mem->svd);
  Vector &tmpTask(mem->tmpTask);
  Vector &tmpVar(mem->tmpVar);
  const Vector &err(mem->err);

  // tmpTask <- S^-1 * U^T * err
  tmpTask.head(rankJ).noalias() = svd.matrixU().leftCols(rankJ).adjoint() * err;
  tmpTask.head(rankJ).array() *=
      svd.singularValues().head(rankJ).array().inverse();

  // control <- kernel * (V * S^-1 * U^T * err)
  if (has_kernel) {
    tmpVar.head(kernel.cols()).noalias() =
        svd.matrixV().leftCols(rankJ) * tmpTask.head(rankJ);
    control.noalias() += kernel * tmpVar.head(kernel.cols());
  } else
    control.noalias() += svd.matrixV().leftCols(rankJ) * tmpTask.head(rankJ);
}

bool isFullPostureTask(Task *task, const Matrix::Index &nDof,
                       const int &iterTime) {
  if (task == NULL || task->getFeatureList().size() != 1 ||
      !task->controlSelectionSIN(iterTime))
    return false;
  FeaturePosture *posture =
      dynamic_cast<FeaturePosture *>(task->getFeatureList().front());

  assert(posture->dimensionSOUT(iterTime) <= nDof - 6);
  return posture != NULL && posture->dimensionSOUT(iterTime) == nDof - 6;
}

MemoryTaskSOT *getMemory(TaskAbstract &t, const Matrix::Index &tDim,
                         const Matrix::Index &nDof) {
  MemoryTaskSOT *mem = dynamic_cast<MemoryTaskSOT *>(t.memoryInternal);
  if (NULL == mem) {
    if (NULL != t.memoryInternal)
      delete t.memoryInternal;
    mem = new MemoryTaskSOT(tDim, nDof);
    t.memoryInternal = mem;
  }
  return mem;
}

/* --------------------------------------------------------------------- */
/* --- CONTROL --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

//#define WITH_CHRONO

#ifdef WITH_CHRONO
#ifndef WIN32
#include <sys/time.h>
#else /*WIN32*/
#include <sot/core/utils-windows.hh>
#endif /*WIN32*/
#endif /*WITH_CHRONO*/

#ifdef WITH_CHRONO
#define sotINIT_CHRONO1                                                        \
  struct timespec t0, t1;                                                      \
  double dt
#define sotSTART_CHRONO1 clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t0)
#define sotCHRONO1                                                             \
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t1);                                 \
  dt = ((double)(t1.tv_sec - t0.tv_sec) * 1e6 +                                \
        (double)(t1.tv_nsec - t0.tv_nsec) / 1e3);                              \
  sotDEBUG(1) << "dT " << (long int)dt << std::endl
#define sotINITPARTCOUNTERS struct timespec tpart0
#define sotSTARTPARTCOUNTERS clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tpart0)
#define sotCOUNTER(nbc1, nbc2)                                                 \
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tpart##nbc2);                        \
  dt##nbc2 = ((double)(tpart##nbc2.tv_sec - tpart##nbc1.tv_sec) * 1e6 +        \
              (double)(tpart##nbc2.tv_nsec - tpart##nbc1.tv_nsec) / 1e3)
#define sotINITCOUNTER(nbc1)                                                   \
  struct timespec tpart##nbc1;                                                 \
  double dt##nbc1 = 0;
#define sotPRINTCOUNTER(nbc1)                                                  \
  sotDEBUG(1) << "dt" << iterTask << '_' << nbc1 << ' ' << (long int)dt##nbc1  \
              << ' '
#else // #ifdef  WITH_CHRONO
#define sotINIT_CHRONO1
#define sotSTART_CHRONO1
#define sotCHRONO1
#define sotINITPARTCOUNTERS
#define sotSTARTPARTCOUNTERS
#define sotCOUNTER(nbc1, nbc2)
#define sotINITCOUNTER(nbc1)
#define sotPRINTCOUNTER(nbc1)
#endif // #ifdef  WITH_CHRONO

void Sot::taskVectorToMlVector(const VectorMultiBound &taskVector,
                               Vector &res) {
  res.resize(taskVector.size());
  unsigned int i = 0;

  for (VectorMultiBound::const_iterator iter = taskVector.begin();
       iter != taskVector.end(); ++iter, ++i) {
    res(i) = iter->getSingleBound();
  }
}

dynamicgraph::Vector &Sot::computeControlLaw(dynamicgraph::Vector &control,
                                             const int &iterTime) {
  sotDEBUGIN(15);

  sotINIT_CHRONO1;
  sotINITPARTCOUNTERS;
  sotINITCOUNTER(1);
  sotINITCOUNTER(2);
  sotINITCOUNTER(3);
  sotINITCOUNTER(4);
  sotINITCOUNTER(5);
  sotINITCOUNTER(6);

  sotSTART_CHRONO1;

  const double &th = inversionThresholdSIN(iterTime);

  bool controlIsZero = true;
  if (q0SIN.isPlugged()) {
    control = q0SIN(iterTime);
    controlIsZero = false;
    if (control.size() != nbJoints) {
      std::ostringstream oss;
      oss << "SOT(" << getName() << "): q0SIN value length is "
          << control.size() << "while the expected lenth is " << nbJoints;
      throw std::length_error(oss.str());
    }
  } else {
    if (stack.size() == 0) {
      std::ostringstream oss;
      oss << "SOT(" << getName()
          << ") contains no task and q0SIN is not plugged.";
      throw std::logic_error(oss.str());
    }
    control = Vector::Zero(nbJoints);
    sotDEBUG(25) << "No initial velocity." << endl;
  }

  sotDEBUGF(5, " --- Time %d -------------------", iterTime);
  unsigned int iterTask = 0;
  KernelConst_t kernel(NULL, 0, 0);
  bool has_kernel = false;
  // Get initial projector if any.
  if (proj0SIN.isPlugged()) {
    const Matrix &K = proj0SIN.access(iterTime);
    makeMap(kernel, K);
    has_kernel = true;
  }
  for (StackType::iterator iter = stack.begin(); iter != stack.end(); ++iter) {
    sotSTARTPARTCOUNTERS;

    sotDEBUGF(5, "Rank %d.", iterTask);
    TaskAbstract &taskA = **iter;
    Task *task = dynamic_cast<Task *>(*iter);

    bool last = (iterTask + 1 == stack.size());
    bool fullPostureTask = (last && enablePostureTaskAcceleration &&
                            isFullPostureTask(task, nbJoints, iterTime));

    sotDEBUG(15) << "Task: e_" << taskA.getName() << std::endl;

    /// Computing first the jacobian may be a little faster overall.
    if (!fullPostureTask)
      taskA.jacobianSOUT.recompute(iterTime);
    taskA.taskSOUT.recompute(iterTime);
    const Matrix::Index dim = taskA.taskSOUT.accessCopy().size();
    sotCOUNTER(0, 1); // Direct Dynamic

    /* Init memory. */
    MemoryTaskSOT *mem = getMemory(taskA, dim, nbJoints);
    /***/ sotCOUNTER(1, 2); // first allocs

    Matrix::Index rankJ = -1;
    taskVectorToMlVector(taskA.taskSOUT(iterTime), mem->err);

    if (fullPostureTask) {
      /***/ sotCOUNTER(2, 3); // compute JK*S
      /***/ sotCOUNTER(3, 4); // compute Jt

      // Jp = kernel.transpose()
      rankJ = kernel.cols();
      /***/ sotCOUNTER(4, 5); // SVD and rank

      /* --- COMPUTE QDOT AND P --- */
      if (!controlIsZero)
        mem->err.noalias() -= control.tail(nbJoints - 6);
      mem->tmpVar.head(rankJ).noalias() =
          kernel.transpose().rightCols(nbJoints - 6) * mem->err;
      control.noalias() += kernel * mem->tmpVar.head(rankJ);
      controlIsZero = false;
    } else {
      assert(taskA.jacobianSOUT.accessCopy().cols() == nbJoints);

      /* --- COMPUTE S * JK --- */
      const Matrix &JK =
          computeJacobianActivated(&taskA, task, mem->JK, iterTime);
      /***/ sotCOUNTER(2, 3); // compute JK*S

      /* --- COMPUTE Jt --- */
      const Matrix *Jt = &mem->Jt;
      if (has_kernel)
        mem->Jt.noalias() = JK * kernel;
      else
        Jt = &JK;
      /***/ sotCOUNTER(3, 4); // compute Jt

      /* --- SVD and RANK--- */
      SVD_t &svd = mem->svd;
      if (last)
        svd.compute(*Jt, Eigen::ComputeThinU | Eigen::ComputeThinV);
      else
        svd.compute(*Jt, Eigen::ComputeThinU | Eigen::ComputeFullV);
      rankJ = 0;
      while (rankJ < svd.singularValues().size() &&
             th < svd.singularValues()[rankJ])
        ++rankJ;
      /***/ sotCOUNTER(4, 5); // SVD and rank

      /* --- COMPUTE QDOT AND P --- */
      if (!controlIsZero)
        mem->err.noalias() -= JK * control;

      updateControl(mem, rankJ, has_kernel, kernel, control);
      controlIsZero = false;

      if (!last) {
        Matrix::Index cols = svd.matrixV().cols() - rankJ;
        if (has_kernel)
          mem->getKernel(nbJoints, cols).noalias() =
              kernel * svd.matrixV().rightCols(cols);
        else
          mem->getKernel(nbJoints, cols).noalias() =
              svd.matrixV().rightCols(cols);
        makeMap(kernel, mem->kernel);
        has_kernel = true;
      }
    }
    /***/ sotCOUNTER(5, 6); // QDOT + Projector

    sotDEBUG(2) << "Proj non optimal (rankJ= " << rankJ
                << ", iterTask =" << iterTask << ")";

    iterTask++;

    sotPRINTCOUNTER(1);
    sotPRINTCOUNTER(2);
    sotPRINTCOUNTER(3);
    sotPRINTCOUNTER(4);
    sotPRINTCOUNTER(5);
    sotPRINTCOUNTER(6);
    if (last || kernel.cols() == 0)
      break;
  }

  sotCHRONO1;

  sotDEBUGOUT(15);
  return control;
}

/* --------------------------------------------------------------------- */
/* --- DISPLAY --------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void Sot::display(std::ostream &os) const {

  os << "+-----------------" << std::endl
     << "+   SOT     " << std::endl
     << "+-----------------" << std::endl;
  for (StackType::const_iterator it = this->stack.begin();
       this->stack.end() != it; ++it) {
    os << "| " << (*it)->getName() << std::endl;
  }
  os << "+-----------------" << std::endl;
}

std::ostream &operator<<(std::ostream &os, const Sot &sot) {
  sot.display(os);
  return os;
}

/* --------------------------------------------------------------------- */
/* --- COMMAND --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

std::ostream &Sot::writeGraph(std::ostream &os) const {
  StackType::const_iterator iter;
  for (iter = stack.begin(); iter != stack.end(); ++iter) {
    const TaskAbstract &task = **iter;
    StackType::const_iterator nextiter = iter;
    nextiter++;

    if (nextiter != stack.end()) {
      TaskAbstract &nexttask = **nextiter;
      os << "\t\t\t\"" << task.getName() << "\" -> \"" << nexttask.getName()
         << "\" [color=red]" << endl;
    }
  }

  os << "\t\tsubgraph cluster_Tasks {" << endl;
  os << "\t\t\tsubgraph \"cluster_" << getName() << "\" {" << std::endl;
  os << "\t\t\t\tcolor=lightsteelblue1; label=\"" << getName()
     << "\"; style=filled;" << std::endl;
  for (iter = stack.begin(); iter != stack.end(); ++iter) {
    const TaskAbstract &task = **iter;
    os << "\t\t\t\t\"" << task.getName() << "\" [ label = \"" << task.getName()
       << "\" ," << std::endl
       << "\t\t\t\t   fontcolor = black, color = black, fillcolor = magenta, "
          "style=filled, shape=box ]"
       << std::endl;
  }
  os << "\t\t\t}" << std::endl;
  os << "\t\t\t}" << endl;
  return os;
}
