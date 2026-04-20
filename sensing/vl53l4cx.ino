#include "SensorUtility.h"

const int XSHUT_1 = 11;
const int XSHUT_2 = 12;
const int XSHUT_3 = 13;

VL53L4CX sensor1(&Wire, XSHUT_1);
VL53L4CX sensor2(&Wire, XSHUT_2);
VL53L$CX sensor3(&Wire, XSHUT_3);

void setup
{
  Serial.begin(115200);
  Wire.begin();
  While (!Serial)
  {
    delay(100);
  }

  initializeSensors(sensor1, sensor2, sensor3, XSHUT_1, XSHUT_2, XSHUT_3);

  sensor1.VL53L4CX_StartMeasurement();
  sensor2.VL53L4CX_StartMeasurement();
  sensor3.VL53L4CX_StartMeasurement();
}

void loop()
{
  processSensors(sensor1, "One");
  processSensors(sensor2, "Two");
  processSensors(sensor3, "Three");
}