/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef WIN32
  #include <sys/time.h>
#else
  #include <Windows.h>
#endif

#include <stdio.h>
#include <iostream>
#include <iomanip>      // std::setprecision
#include <sot/core/logger.hh>

namespace dynamicgraph
{
  namespace sot
  {
      using namespace std;

      Logger& getLogger()
      {
        static Logger l(0.001, 1.0);
        return l;
      }

      Logger::Logger(double timeSample, double streamPrintPeriod)
        : m_timeSample(timeSample),
          m_streamPrintPeriod(streamPrintPeriod),
          m_printCountdown(0.0)
      {
#ifdef LOGGER_VERBOSITY_ERROR
        m_lv = VERBOSITY_ERROR;
#endif
#ifdef LOGGER_VERBOSITY_WARNING_ERROR
        m_lv = VERBOSITY_WARNING_ERROR;
#endif
#ifdef LOGGER_VERBOSITY_INFO_WARNING_ERROR
        m_lv = VERBOSITY_INFO_WARNING_ERROR;
#endif
#ifdef LOGGER_VERBOSITY_ALL
        m_lv = VERBOSITY_ALL;
#endif
      }

      void Logger::countdown()
      {
        if(m_printCountdown<0.0)
          m_printCountdown = m_streamPrintPeriod;
        m_printCountdown -= m_timeSample;
      }

      void Logger::sendMsg(string msg, MsgType type, const char* file, int line)
      {
        if(m_lv==VERBOSITY_NONE ||
          (m_lv==VERBOSITY_ERROR && !isErrorMsg(type)) ||
          (m_lv==VERBOSITY_WARNING_ERROR && !(isWarningMsg(type) || isErrorMsg(type))) ||
          (m_lv==VERBOSITY_INFO_WARNING_ERROR && isDebugMsg(type)))
          return;

        // if print is allowed by current verbosity level
        if(isStreamMsg(type))
        {
          // check whether counter already exists
          string id = file+toString(line);
          map<string,double>::iterator it = m_stream_msg_counters.find(id);
          if(it == m_stream_msg_counters.end())
          {
            // if counter doesn't exist then add one
            m_stream_msg_counters.insert(make_pair(id, 0.0));
            it = m_stream_msg_counters.find(id);
          }

          // if counter is greater than 0 then decrement it and do not print
          if(it->second>0.0)
          {
            it->second -= m_timeSample;
            return;
          }
          else  // otherwise reset counter and print
            it->second = m_streamPrintPeriod;
        }
        printf("%s\n", msg.c_str());
        fflush(stdout); // Prints to screen or whatever your standard out is
      }

      bool Logger::setTimeSample(double t)
      {
        if(t<=0.0)
          return false;
        m_timeSample = t;
        return true;
      }

      bool Logger::setStreamPrintPeriod(double s)
      {
        if(s<=0.0)
          return false;
        m_streamPrintPeriod = s;
        return true;
      }
  } // namespace sot
} // namespace dynamicgraph

