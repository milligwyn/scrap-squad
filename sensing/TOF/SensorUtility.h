#ifdef SENSOR_UTILITY_H
#define SENSOR_UTILITY_H

#include <Arduino.h>
#include <vl53l4cx_class.h>

void initSensors(VL53L4CX &s1, VL53L4CX &s2, int p1, int p2);
void processSensors(VL53L4CX &sensor, const char* sensorName);

#endif
