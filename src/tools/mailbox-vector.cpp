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

/* --- SOT PLUGIN  --- */
#include <jrl/mal/boost.hh>
#include <sot-core/debug.h>
#include <sot-core/factory.h>
#include <sot-core/mailbox.t.cpp>
#include <sot-core/mailbox-vector.h>

using namespace sot;
using namespace dynamicgraph;

// Explicit template specialization
#ifdef WIN32
MailboxVector::MailboxVector( const std::string& name): Mailbox<maal::boost::Vector> (name){}
#else
MAILBOX_TEMPLATE_SPE(maal::boost::Vector);
#endif

template<>DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MailboxVector,"Mailbox<Vector>");

