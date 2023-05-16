#ifndef CARMENCONFIGURATION_H
#define CARMENCONFIGURATION_H

#include "gmapping/log/configuration.h"
#include <gmapping/log/log_export.h>
#include <gmapping/sensor/sensor_base/sensor.h>
#include <istream>
#include <map>
#include <string>
#include <vector>

namespace GMapping {

class LOG_EXPORT CarmenConfiguration
    : public Configuration,
      public std::map<std::string, std::vector<std::string>> {
public:
  virtual std::istream &load(std::istream &is);
  virtual SensorMap computeSensorMap() const;
};

}; // namespace GMapping

#endif
