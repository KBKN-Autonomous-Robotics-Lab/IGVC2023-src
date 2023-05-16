#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <gmapping/log/log_export.h>
#include <gmapping/sensor/sensor_base/sensor.h>
#include <istream>

namespace GMapping {

class LOG_EXPORT Configuration {
public:
  virtual ~Configuration();
  virtual SensorMap computeSensorMap() const = 0;
};

}; // namespace GMapping
#endif
