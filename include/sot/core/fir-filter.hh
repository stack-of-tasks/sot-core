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

#ifndef __SOT_FIRFILTER_HH__
#define __SOT_FIRFILTER_HH__

#include <cassert>

#include <vector>
#include <algorithm>
#include <iterator>

//#include <boost/circular_buffer.hpp>

#include <jrl/mal/malv2.hh>
DECLARE_MAL_NAMESPACE(ml);

#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
namespace dg = dynamicgraph;


namespace detail
{
  /* GRMBL boost-sandox::circular_buffer smells... Why keep using 1.33?!
   * As a workaround, only the part of circular_buffer's interface used here is implemented.
   * Ugly, fatty piece of code.
   */
  template<class T>
  class circular_buffer
  {
  public:
    circular_buffer(): buf(1), start(0), numel(0) {}
    void push_front(const T& data) {
      if(start){ --start; } else { start = buf.size()-1; }
      buf[start] = data;
      if(numel < buf.size()){ ++numel; }
    }
    void reset_capacity(size_t n) {
      buf.resize(n);
      start = 0;
      numel = 0;
    }
    void reset_capacity(size_t n, const T& el) {
      buf.clear();
      buf.resize(n, el);
      start = 0;
      numel = 0;
    }
    T& operator[](size_t i) {
      assert((i<numel) && "Youre accessing an empty buffer");
      size_t index = (start+i) % buf.size();
      return buf[index];
    }
    size_t size() const { return numel; }
  private:
    std::vector<T> buf;
    size_t start;
    size_t numel;
  };
}

template<class sigT, class coefT>
class FIRFilter
  : public dg::Entity
{
public:
  virtual const std::string& getClassName() const { return dg::Entity::getClassName(); }
  static std::string getTypeName( void ) { return "Unknown"; }
  static const std::string CLASS_NAME;

public:
  FIRFilter( const std::string& name )
    : dg::Entity(name)
    , SIN(NULL,"sotFIRFilter("+name+")::input(T)::in")
    , SOUT(boost::bind(&FIRFilter::compute,this,_1,_2),
	   SIN,
	   "sotFIRFilter("+name+")::output(T)::out")
  {
    signalRegistration( SIN<<SOUT );
  }

  virtual ~FIRFilter() {}

  virtual sigT& compute( sigT& res,int time )
  {
    const sigT& in = SIN.access( time );
    reset_signal( res, in );
    data.push_front( in );

    size_t SIZE = std::min(data.size(), coefs.size());
    for(size_t i = 0; i < SIZE; ++i) {
      res += coefs[i] * data[i];
    }

    return res;
  }

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os )
  {
    if(cmdLine == "setCoefs") {
      coefT tmp;
      std::vector<coefT> ncoefs;
      while(cmdArgs>>tmp){ ncoefs.push_back(tmp); }
      if(!ncoefs.empty()){
	coefs.swap(ncoefs);
	data.reset_capacity(coefs.size());
      }
      else {
	std::copy( coefs.begin(), coefs.end(),
		  std::ostream_iterator<coefT>(os, " ") );
	os << std::endl;
      }
    }
    else if(cmdLine == "printBuffer") {
      for(size_t i = 0; i < data.size(); ++i){ os << data[i] << std::endl; }
    }
    else { dg::Entity::commandLine( cmdLine, cmdArgs, os); }
  }

  static void reset_signal(sigT& /*res*/, const sigT& /*sample*/ ) { }

public:
  dg::SignalPtr<sigT, int> SIN;
  dg::SignalTimeDependent<sigT, int> SOUT;

private:
  std::vector<coefT> coefs;
  detail::circular_buffer<sigT> data;
};

#endif
