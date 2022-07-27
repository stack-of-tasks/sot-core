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
#include <list>
#ifdef WIN32
#include <time.h>
#endif /*WIN32*/

/* --- SOT --- */
#include <dynamic-graph/entity.h>

#include <sot/core/debug.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/pool.hh>
#include <sot/core/task-abstract.hh>

namespace dynamicgraph {
namespace sot {
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

PoolStorage::~PoolStorage(void) {
  sotDEBUGIN(15);

  sotDEBUGOUT(15);
  return;
}

/* --------------------------------------------------------------------- */
void PoolStorage::registerTask(const std::string &entname, TaskAbstract *ent) {
  Tasks::iterator entkey = task.find(entname);
  if (entkey != task.end())  // key does exist
  {
    throw ExceptionFactory(ExceptionFactory::OBJECT_CONFLICT,
                           "Another task already defined with the "
                           "same name. ",
                           "Task name is <%s>.", entname.c_str());
  } else {
    sotDEBUG(10) << "Register task <" << entname << "> in the pool."
                 << std::endl;
    task[entname] = ent;
  }
}

TaskAbstract &PoolStorage::getTask(const std::string &name) {
  Tasks::iterator entPtr = task.find(name);
  if (entPtr == task.end()) {
    SOT_THROW ExceptionFactory(ExceptionFactory::UNREFERED_OBJECT,
                               "Unknown task.", " (while calling <%s>)",
                               name.c_str());
  }
  return *entPtr->second;
}

/* --------------------------------------------------------------------- */
void PoolStorage::registerFeature(const std::string &entname,
                                  FeatureAbstract *ent) {
  Features::iterator entkey = feature.find(entname);
  if (entkey != feature.end())  // key does exist
  {
    throw ExceptionFactory(ExceptionFactory::OBJECT_CONFLICT,
                           "Another feature already defined with the"
                           " same name. ",
                           "Feature name is <%s>.", entname.c_str());
  } else {
    sotDEBUG(10) << "Register feature <" << entname << "> in the pool."
                 << std::endl;
    feature[entname] = ent;
  }
}

FeatureAbstract &PoolStorage::getFeature(const std::string &name) {
  Features::iterator entPtr = feature.find(name);
  if (entPtr == feature.end()) {
    SOT_THROW ExceptionFactory(ExceptionFactory::UNREFERED_OBJECT,
                               "Unknown feature.", " (while calling <%s>)",
                               name.c_str());
  }
  return *entPtr->second;
}

void PoolStorage::writeGraph(const std::string &aFileName) {
  size_t IdxPointFound = aFileName.rfind(".");
  std::string tmp1 = aFileName.substr(0, IdxPointFound);
  size_t IdxSeparatorFound = aFileName.rfind("/");
  std::string GenericName;
  if (IdxSeparatorFound != std::string::npos)
    GenericName = tmp1.substr(IdxSeparatorFound, tmp1.length());
  else
    GenericName = tmp1;

  /* Reading local time */
  time_t ltime;
  ltime = time(NULL);
  struct tm ltimeformatted;
#ifdef WIN32
  localtime_s(&ltimeformatted, &ltime);
#else
  localtime_r(&ltime, &ltimeformatted);
#endif /*WIN32*/

  /* Opening the file and writing the first comment. */
  std::ofstream GraphFile;
  GraphFile.open(aFileName.c_str(), std::ofstream::out);
  GraphFile << "/* This graph has been automatically generated. " << std::endl;
  GraphFile << "   " << 1900 + ltimeformatted.tm_year
            << " Month: " << 1 + ltimeformatted.tm_mon
            << " Day: " << ltimeformatted.tm_mday
            << " Time: " << ltimeformatted.tm_hour << ":"
            << ltimeformatted.tm_min;
  GraphFile << " */" << std::endl;
  GraphFile << "digraph " << GenericName << " { ";
  GraphFile << "\t graph [ label=\"" << GenericName
            << "\" bgcolor = white rankdir=LR ]" << std::endl
            << "\t node [ fontcolor = black, color = black, "
               "fillcolor = gold1, style=filled, shape=box ] ; "
            << std::endl;
  GraphFile << "\tsubgraph cluster_Tasks { " << std::endl;
  GraphFile << "\t\t color=blue; label=\"Tasks\";" << std::endl;

  for (Tasks::iterator iter = task.begin(); iter != task.end(); iter++) {
    TaskAbstract *ent = iter->second;
    GraphFile << "\t\t" << ent->getName() << " [ label = \"" << ent->getName()
              << "\" ," << std::endl
              << "\t\t   fontcolor = black, color = black, "
                 "fillcolor = magenta, style=filled, shape=box ]"
              << std::endl;
  }

  GraphFile << "}" << std::endl;

  GraphFile.close();
}

void PoolStorage::writeCompletionList(std::ostream & /*os*/) {}

PoolStorage *PoolStorage::getInstance() {
  if (instance_ == 0) {
    instance_ = new PoolStorage;
  }
  return instance_;
}
void PoolStorage::destroy() {
  delete instance_;
  instance_ = 0;
}

PoolStorage::PoolStorage() {}

PoolStorage *PoolStorage::instance_ = 0;
}  // namespace sot
}  // namespace dynamicgraph
