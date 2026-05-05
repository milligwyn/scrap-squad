#include "SensorUtility.h"

const int XSHUT_1 = 11;
const int XSHUT_2 = 12;

VL53L4CX sensor1(&Wire1, XSHUT_1);
VL53L4CX sensor2(&Wire1, XSHUT_2);

void setup()
{
  initializeSensors(sensor1, sensor2, XSHUT_1, XSHUT_2);

  sensor1.VL53L4CX_StartMeasurement();
  sensor2.VL53L4CX_StartMeasurement();
}

void loop()
{
  int dist1 = processSensors(sensor1);
  int dist2 = processSensors(sensor2);
}
