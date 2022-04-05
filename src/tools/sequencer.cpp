/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>

#include <sot/core/debug.hh>
#include <sot/core/exception-tools.hh>
#include <sot/core/sequencer.hh>
#include <sot/core/sot.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Sequencer, "Sequencer");

Sequencer::Sequencer(const std::string &name)
    : Entity(name),
      timeInit(-1),
      playMode(false),
      outputStreamPtr(NULL),
      noOutput(false),
      triggerSOUT(boost::bind(&Sequencer::trigger, this, _1, _2), sotNOSIGNAL,
                  "Sequencer(" + name + ")::output(dummy)::trigger") {
  sotDEBUGIN(5);

  signalRegistration(triggerSOUT);
  triggerSOUT.setNeedUpdateFromAllChildren(true);

  sotDEBUGOUT(5);
}

Sequencer::~Sequencer(void) {
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SPECIFIC EVENT ------------------------------------------------------- */
/* --- SPECIFIC EVENT ------------------------------------------------------- */
/* --- SPECIFIC EVENT ------------------------------------------------------- */

class sotEventTaskBased : public Sequencer::sotEventAbstract {
 protected:
  TaskAbstract *taskPtr;
  const std::string defaultTaskName;

 public:
  sotEventTaskBased(const std::string name = "", TaskAbstract *task = NULL)
      : sotEventAbstract(name), taskPtr(task), defaultTaskName("NULL") {}

  void init(std::istringstream &cmdArgs) {
    cmdArgs >> std::ws;
    if (cmdArgs.good()) {
      std::string taskname;
      cmdArgs >> taskname;
      sotDEBUG(15) << "Add task " << taskname << std::endl;
      taskPtr = dynamic_cast<TaskAbstract *>(
          &dynamicgraph::PoolStorage::getInstance()->getEntity(taskname));
    }
  }
  virtual void display(std::ostream &os) const {
    if (taskPtr)
      os << taskPtr->getName();
    else
      os << "NULL";
  }
  virtual const std::string &getName() const {
    if (taskPtr)
      return taskPtr->getName();
    else
      return defaultTaskName;
  }
};

class sotEventAddATask : public sotEventTaskBased {
 public:
  sotEventAddATask(const std::string name = "", TaskAbstract *task = NULL)
      : sotEventTaskBased(name, task) {
    eventType = EVENT_ADD;
  }

  void operator()(Sot *sotptr) {
    sotDEBUGIN(15);
    sotDEBUG(45) << "Sot = " << sotptr << ". Task = " << taskPtr << "."
                 << std::endl;
    if ((NULL != sotptr) && (NULL != taskPtr)) sotptr->push(*taskPtr);
    sotDEBUGOUT(15);
  }

  virtual void display(std::ostream &os) const {
    os << "Add<";
    sotEventTaskBased::display(os);
    os << ">";
  }
};

class sotEventRemoveATask : public sotEventTaskBased {
 public:
  sotEventRemoveATask(const std::string name = "", TaskAbstract *task = NULL)
      : sotEventTaskBased(name, task) {
    eventType = EVENT_RM;
  }

  void operator()(Sot *sotptr) {
    sotDEBUGIN(15);
    sotDEBUG(45) << "Sot = " << sotptr << ". Task = " << taskPtr << "."
                 << std::endl;
    if ((NULL != sotptr) && (NULL != taskPtr)) sotptr->remove(*taskPtr);
    sotDEBUGOUT(15);
  }

  virtual void display(std::ostream &os) const {
    os << "Remove<";
    sotEventTaskBased::display(os);
    os << ">";
  }
};

class sotEventCmd : public Sequencer::sotEventAbstract {
 protected:
  std::string cmd;

 public:
  sotEventCmd(const std::string cmdLine = "")
      : sotEventAbstract(cmdLine + "<cmd>"), cmd(cmdLine) {
    eventType = EVENT_CMD;
    sotDEBUGINOUT(15);
  }

  void init(std::istringstream &args) {
    sotDEBUGIN(15);
    std::stringbuf *pbuf = args.rdbuf();
    const unsigned int size = (unsigned int)(pbuf->in_avail());
    char *buffer = new char[size + 1];
    pbuf->sgetn(buffer, size);

    buffer[size] = '\0';
    cmd = buffer;
    sotDEBUGOUT(15);
    delete[] buffer;
  }
  const std::string &getEventCmd() const { return cmd; }
  virtual void display(std::ostream &os) const { os << "Run: " << cmd; }
  virtual void operator()(Sot * /*sotPtr*/) {
    std::ostringstream onull;
    onull.clear(std::ios::failbit);
    std::istringstream iss(cmd);
    std::string cmdName;
    iss >> cmdName;
    // Florent: remove reference to g_shell
    // g_shell.cmd( cmdName,iss,onull );
  };
};

/* --- TASK MANIP ----------------------------------------------------------- */
/* --- TASK MANIP ----------------------------------------------------------- */
/* --- TASK MANIP ----------------------------------------------------------- */

void Sequencer::addTask(sotEventAbstract *task, const unsigned int timeSpec) {
  TaskMap::iterator listKey = taskMap.find(timeSpec);
  if (taskMap.end() == listKey) {
    sotDEBUG(15) << "New element at " << timeSpec << std::endl;
    taskMap[timeSpec].push_back(task);
  } else {
    TaskList &tl = listKey->second;
    tl.push_back(task);
  }
}

// rmTask
void Sequencer::rmTask(int eventType, const std::string &name,
                       const unsigned int time) {
  TaskMap::iterator listKey = taskMap.find(time);
  if (taskMap.end() != listKey)  // the time exist
  {
    TaskList &tl = listKey->second;
    for (TaskList::iterator itL = tl.begin(); itL != tl.end(); ++itL) {
      if ((*itL)->getEventType() == eventType && (*itL)->getName() == name) {
        tl.remove(*itL);
        break;
      }
    }

    // remove the list if empty
    if (tl.empty()) taskMap.erase(listKey);
  }
}

// clearAll
void Sequencer::clearAll() {
  TaskMap::iterator itM;
  for (itM = taskMap.begin(); itM != taskMap.end(); ++itM) {
    TaskList::iterator itL;
    TaskList &currentMap = itM->second;
    for (itL = currentMap.begin(); itL != currentMap.end(); ++itL)
      delete (*itL);
    itM->second.clear();
  }
  // remove all the lists
  taskMap.clear();
}
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */

int &Sequencer::trigger(int &dummy, const int &timeSpec) {
  sotDEBUGIN(15);

  if (!playMode) return dummy;
  if (-1 == timeInit) timeInit = timeSpec;

  sotDEBUG(15) << "Ref time: " << (timeSpec - timeInit) << std::endl;
  TaskMap::iterator listKey = taskMap.find(timeSpec - timeInit);
  if (taskMap.end() != listKey) {
    sotDEBUG(1) << "Time: " << (timeSpec - timeInit)
                << ": we've got a task to do!" << std::endl;
    TaskList &tl = listKey->second;
    for (TaskList::iterator iter = tl.begin(); iter != tl.end(); ++iter) {
      if (*iter) {
        (*iter)->operator()(sotPtr);
        if (NULL != outputStreamPtr) {
          (*outputStreamPtr) << "At time t=" << timeSpec << ": ";
          (*iter)->display(*outputStreamPtr);
          (*outputStreamPtr) << std::endl;
        }
      }
    }
  }

  sotDEBUGOUT(15);
  return dummy;
}

/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */

void Sequencer::display(std::ostream &os) const {
  if (noOutput) return;

  os << "Sequencer " << getName() << "(t0=" << timeInit
     << ",mode=" << ((playMode) ? "play" : "pause") << "): " << std::endl;
  for (TaskMap::const_iterator iterMap = taskMap.begin();
       iterMap != taskMap.end(); iterMap++) {
    os << " - t=" << (iterMap->first) << ":\t";
    const TaskList &tl = iterMap->second;
    for (TaskList::const_iterator iterList = tl.begin(); iterList != tl.end();
         iterList++) {
      (*iterList)->display(os);
      os << " ";
    }
    os << std::endl;
  }
}
