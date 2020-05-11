/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <iostream>
#include <sot/core/debug.hh>

#ifndef WIN32
#include <unistd.h>
#endif

using namespace std;

#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <sot/core/feature-abstract.hh>
#include <sot/core/mailbox-vector.hh>
#include <sstream>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

#include <boost/thread.hpp>

sot::MailboxVector *mailbox = NULL;

void f(void) {
  Vector vect(25);
  Vector vect2(25);
  for (int i = 0; i < 250; ++i) {
    std::cout << " iter  " << i << std::endl;
    for (int j = 0; j < 25; ++j)
      vect(j) = j + i * 10;
    mailbox->post(vect);
    Vector V = mailbox->getObject(vect2, 1);
    std::cout << vect2 << std::endl;
    std::cout << " getClassName   " << mailbox->getClassName() << std::endl;
    std::cout << " getName        " << mailbox->getName() << std::endl;
    std::cout << " hasBeenUpdated " << mailbox->hasBeenUpdated() << std::endl;
    std::cout << std::endl;
  }
}

int main(int, char **) {
  mailbox = new sot::MailboxVector("mail");

  boost::thread th(f);
  th.join();

  return 0;
}
