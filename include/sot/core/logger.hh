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

#ifndef __sot_torque_control_logger_H__
#define __sot_torque_control_logger_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (logger_EXPORTS)
#    define LOGGER_EXPORT __declspec(dllexport)
#  else
#    define LOGGER_EXPORT __declspec(dllimport)
#  endif
#else
#  define LOGGER_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/core/signal-helper.hh>
#include <sot/core/matrix-geometry.hh>
#include <map>
#include <iomanip> // std::setprecision
#include "boost/assign.hpp"


namespace dynamicgraph {
  namespace sot {

//#define LOGGER_VERBOSITY_INFO_WARNING_ERROR
#define LOGGER_VERBOSITY_ALL

#define SEND_MSG(msg,type)         sendMsg(msg,type,__FILE__,__LINE__)

#ifdef LOGGER_VERBOSITY_ERROR
  #define SEND_DEBUG_STREAM_MSG(msg)
  #define SEND_INFO_STREAM_MSG(msg)
  #define SEND_WARNING_STREAM_MSG(msg)
  #define SEND_ERROR_STREAM_MSG(msg)    SEND_MSG(msg,MSG_TYPE_ERROR_STREAM)
#endif

#ifdef LOGGER_VERBOSITY_WARNING_ERROR
  #define SEND_DEBUG_STREAM_MSG(msg)
  #define SEND_INFO_STREAM_MSG(msg)\
  #define SEND_WARNING_STREAM_MSG(msg)  SEND_MSG(msg,MSG_TYPE_WARNING_STREAM)
  #define SEND_ERROR_STREAM_MSG(msg)    SEND_MSG(msg,MSG_TYPE_ERROR_STREAM)
#endif

#ifdef LOGGER_VERBOSITY_INFO_WARNING_ERROR
  #define SEND_DEBUG_STREAM_MSG(msg)
  #define SEND_INFO_STREAM_MSG(msg)     SEND_MSG(msg,MSG_TYPE_INFO_STREAM)
  #define SEND_WARNING_STREAM_MSG(msg)  SEND_MSG(msg,MSG_TYPE_WARNING_STREAM)
  #define SEND_ERROR_STREAM_MSG(msg)    SEND_MSG(msg,MSG_TYPE_ERROR_STREAM)
#endif

#ifdef LOGGER_VERBOSITY_ALL
  #define SEND_DEBUG_STREAM_MSG(msg) SEND_MSG(msg,MSG_TYPE_DEBUG_STREAM)
  #define SEND_INFO_STREAM_MSG(msg)   SEND_MSG(msg,MSG_TYPE_INFO_STREAM)
  #define SEND_WARNING_STREAM_MSG(msg)  SEND_MSG(msg,MSG_TYPE_WARNING_STREAM)
  #define SEND_ERROR_STREAM_MSG(msg)    SEND_MSG(msg,MSG_TYPE_ERROR_STREAM)
#endif

      /** Enum representing the different kind of messages.
       */
      enum MsgType
      {
        MSG_TYPE_DEBUG          =0,
        MSG_TYPE_INFO           =1,
        MSG_TYPE_WARNING        =2,
        MSG_TYPE_ERROR          =3,
        MSG_TYPE_DEBUG_STREAM   =4,
        MSG_TYPE_INFO_STREAM    =5,
        MSG_TYPE_WARNING_STREAM =6,
        MSG_TYPE_ERROR_STREAM   =7
      };

      template<typename T>
      std::string toString(const T& v, const int precision=3, const int width=-1)
      {
        std::stringstream ss;
        if(width>precision)
          ss<<std::fixed<<std::setw(width)<<std::setprecision(precision)<<v;
        else
          ss<<std::fixed<<std::setprecision(precision)<<v;
        return ss.str();
      }

      template<typename T>
      std::string toString(const std::vector<T>& v, const int precision=3, const int width=-1,
                           const std::string separator=", ")
      {
        std::stringstream ss;
        if(width>precision)
        {
          for(int i=0; i<v.size()-1; i++)
            ss<<std::fixed<<std::setw(width)<<std::setprecision(precision)<<v[i]<<separator;
          ss<<std::fixed<<std::setw(width)<<std::setprecision(precision)<<v[v.size()-1];
        }
        else
        {
          for(int i=0; i<v.size()-1; i++)
            ss<<std::fixed<<std::setprecision(precision)<<v[i]<<separator;
          ss<<std::fixed<<std::setprecision(precision)<<v[v.size()-1];
        }

        return ss.str();
      }

//      template<typename T, int N>
//      std::string toString(const Eigen::Matrix<T, N, 1, 0, N, 1>& v, const std::string separator=", ",
//                           const int precision=3, const int width=-1)
      template<typename T>
      std::string toString(const Eigen::MatrixBase<T>& v,
                           const int precision=3, const int width=-1, const std::string separator=", ")
      {
        std::stringstream ss;
        if(width>precision)
        {
          for(int i=0; i<v.size()-1; i++)
            ss<<std::fixed<<std::setw(width)<<std::setprecision(precision)<<v[i]<<separator;
          ss<<std::fixed<<std::setw(width)<<std::setprecision(precision)<<v[v.size()-1];
        }
        else
        {
          for(int i=0; i<v.size()-1; i++)
            ss<<std::fixed<<std::setprecision(precision)<<v[i]<<separator;
          ss<<std::setprecision(precision)<<v[v.size()-1];
        }

        return ss.str();
      }

      enum LoggerVerbosity
      {
        VERBOSITY_ALL,
        VERBOSITY_INFO_WARNING_ERROR,
        VERBOSITY_WARNING_ERROR,
        VERBOSITY_ERROR,
        VERBOSITY_NONE
      };

      /** A simple class for logging messages
      */
      class Logger
      {
      public:

        /** Constructor */
        Logger(double timeSample=0.001, double streamPrintPeriod=1.0);

        /** Destructor */
        ~Logger(){}

        /** Method to be called at every control iteration
           * to decrement the internal Logger's counter. */
        void countdown();

        /** Print the specified message on standard output if the verbosity level
         * allows it. The file name and the line number are used to identify
         * the point where sendMsg is called so that streaming messages are
         * printed only every streamPrintPeriod iterations.
         */
        void sendMsg(std::string msg, MsgType type, const char* file="", int line=0);

        /** Set the sampling time at which the method countdown()
           * is going to be called. */
        bool setTimeSample(double t);

        /** Set the time period for printing of streaming messages. */
        bool setStreamPrintPeriod(double s);

        /** Set the verbosity level of the logger. */
        void setVerbosity(LoggerVerbosity lv);

      protected:
        LoggerVerbosity m_lv;                /// verbosity of the logger
        double          m_timeSample;        /// specify the period of call of the countdown method
        double          m_streamPrintPeriod; /// specify the time period of the stream prints
        double          m_printCountdown;    /// every time this is < 0 (i.e. every _streamPrintPeriod sec) print stuff

        /** Pointer to the dynamic structure which holds the collection of streaming messages */
        std::map<std::string, double> m_stream_msg_counters;

        bool isStreamMsg(MsgType m)
        { return m==MSG_TYPE_ERROR_STREAM || m==MSG_TYPE_DEBUG_STREAM || m==MSG_TYPE_INFO_STREAM || m==MSG_TYPE_WARNING_STREAM; }

        bool isDebugMsg(MsgType m)
        { return m==MSG_TYPE_DEBUG_STREAM || m==MSG_TYPE_DEBUG; }

        bool isInfoMsg(MsgType m)
        { return m==MSG_TYPE_INFO_STREAM || m==MSG_TYPE_INFO; }

        bool isWarningMsg(MsgType m)
        { return m==MSG_TYPE_WARNING_STREAM || m==MSG_TYPE_WARNING; }

        bool isErrorMsg(MsgType m)
        { return m==MSG_TYPE_ERROR_STREAM || m==MSG_TYPE_ERROR; }
      };

      /** Method to get the logger (singleton). */
      Logger& getLogger();

  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_logger_H__
