/*
 * Copyright 2011,
 * Olivier Stasse, CNRS
 *
 * CNRS
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

#ifndef ABSTRACT_SOT_EXTERNAL_INTERFACE_HH
#define ABSTRACT_SOT_EXTERNAL_INTERFACE_HH

#include <vector>
#include <string>

namespace dynamicgraph {
  namespace sot {

    class SOT_CORE_EXPORT Sensors
    {

    private:
      std::string name_;
      std::vector<double> values_;

    public:
      Sensors() {};
      ~Sensors() {};

      const string & getName()
      { return name_;}

      void setName(const std::string & aname)
      { name_ = aname;}

      const vector<double> & getValues()
      { return values_;}

      void setValues(std::vector<double> values)
      { values_ = values;}

    };

    class SOT_CORE_EXPORT AbstractSotExternalInterface
    {
    public:

      AbstractSotExternalInterface(){};

      virtual ~AbstractSotExternalInterface(){};

      virtual void setupSetSensors(std::vector<Sensors> &sensorsIn)=0;
      
      virtual void nominalSetSensors(std::vector<Sensors> &sensorsIn)=0;

      virtual void cleanupSetSensors(std::vector<Sensors> &sensorsIn)=0;

      virtual void getControl(std::vector<double> &)=0;
    };
  }
}
typedef AbstractSoTExternalInterface * createSoTExternalInterface_t();
typedef void destroySotExternalInterface_t (AbstractSoTExternalInterface *);

#endif
