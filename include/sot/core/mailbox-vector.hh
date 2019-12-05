/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_MAILBOX_VECTOR_HH
#define __SOT_MAILBOX_VECTOR_HH

/* --- SOT PLUGIN  --- */
#include <sot/core/mailbox.hh>

#include <dynamic-graph/linear-algebra.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(mailbox_vector_EXPORTS)
#define MAILBOX_VECTOR_EXPORT __declspec(dllexport)
#else
#define MAILBOX_VECTOR_EXPORT __declspec(dllimport)
#endif
#else
#define MAILBOX_VECTOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {
#ifdef WIN32
class MAILBOX_VECTOR_EXPORT MailboxVector
    : public Mailbox<dynamicgraph::Vector> {
public:
  MailboxVector(const std::string &name);
};
#else
typedef Mailbox<dynamicgraph::Vector> MailboxVector;
#endif
} // namespace sot
} // namespace dynamicgraph

#endif // #ifndef  __SOT_MAILBOX_HH
