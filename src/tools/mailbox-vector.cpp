/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* --- SOT PLUGIN  --- */
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/debug.hh>
#include <sot/core/factory.hh>
#include <sot/core/mailbox-vector.hh>
#include <sot/core/mailbox.hxx>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

// Explicit template specialization
#ifdef WIN32
MailboxVector::MailboxVector(const std::string &name)
    : Mailbox<dynamicgraph::Vector>(name) {}
#else
MAILBOX_TEMPLATE_SPE(dynamicgraph::Vector)
#endif

template <>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MailboxVector, "Mailbox<Vector>");
