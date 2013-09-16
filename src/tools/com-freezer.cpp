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

#include <sot/core/com-freezer.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>


using namespace dynamicgraph;
using namespace dynamicgraph::sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CoMFreezer, "CoMFreezer");

CoMFreezer::CoMFreezer(const std::string & name)
  : Entity(name)
  , m_lastCoM(3)
  , m_previousPGInProcess(false)
  , m_lastStopTime(-1)

  , CoMRefSIN(NULL, "CoMFreezer("+name+")::input(vector)::CoMRef")
  , PGInProcessSIN(NULL, "CoMFreezer("+name+")::input(bool)::PGInProcess")
  , freezedCoMSOUT(boost::bind(&CoMFreezer::computeFreezedCoM, this, _1, _2),
      CoMRefSIN << PGInProcessSIN,
      "CoMFreezer("+name+")::output(vector)::freezedCoM")
{
  sotDEBUGIN(5);

  signalRegistration( CoMRefSIN << PGInProcessSIN << freezedCoMSOUT );

  sotDEBUGOUT(5);
}

CoMFreezer::~CoMFreezer(void)
{
  sotDEBUGIN(5);
  sotDEBUGOUT(5);
  return;
}

dynamicgraph::Vector & CoMFreezer::computeFreezedCoM(dynamicgraph::Vector & freezedCoM, const int & time)
{
  sotDEBUGIN(15);

  unsigned PGInProcess = PGInProcessSIN(time); 
  if(PGInProcess) /* CoM unfreezed */
  {
    m_lastCoM.noalias() = CoMRefSIN(time);
    m_previousPGInProcess = (PGInProcess == 0);
  }
  else
  {
    if(m_previousPGInProcess) /* pg.inprocess switch from 1 to 0 */
    {
      m_lastStopTime = time;
      m_lastCoM = CoMRefSIN(time);
      m_previousPGInProcess = (PGInProcess == 0);
    }
    else if(time < m_lastStopTime + 200) /* keep updating for 1s */
    {
      m_lastCoM = CoMRefSIN(time);
    }
  }

  freezedCoM.noalias() = m_lastCoM;

  sotDEBUGOUT(15);

  if(m_lastStopTime < 0)
  {
    m_lastCoM.noalias() = CoMRefSIN(time);
    m_lastStopTime = time;
    freezedCoM.noalias() = m_lastCoM;
    return freezedCoM;
  }


  return m_lastCoM;
}

void CoMFreezer::display(std::ostream & os) const
{
  os << "CoMFreezer " << getName() << "." << std::endl;
}
