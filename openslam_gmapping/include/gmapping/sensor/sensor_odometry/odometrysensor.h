#ifndef ODOMETRYSENSOR_H
#define ODOMETRYSENSOR_H

#include <gmapping/sensor/sensor_base/sensor.h>
#include <gmapping/sensor/sensor_odometry/sensor_odometry_export.h>
#include <string>

namespace GMapping {

class SENSOR_ODOMETRY_EXPORT OdometrySensor : public Sensor {
public:
  OdometrySensor(const std::string &name, bool ideal = false);
  inline bool isIdeal() const { return m_ideal; }

protected:
  bool m_ideal;
};

}; // namespace GMapping

#endif
