#include "SensorUtility.h"

static VL53L4CX_MultiRangingData_t MultiRangingData;

void initSensors(VL53L4CX &s1, VL53L4CX &s2, VL53L4CX &s3, int p1, int p2, int p3)
{
  pinMode(p1, OUTPUT);
  pinMode(p2, OUTPUT);
  pinMode(p3, OUTPUT);

  digitalWrite(p1, LOW);
  digitalWrite(p2, LOW);
  digitalWrite(p3, LOW);
  delay(100);

  digitalWrite(p1, HIGH);
  delay(50);
  s1.begin();
  s1.InitSensor(0x52);
  s1.VL53L4CX_SetDeviceAddress(0x56);

  digitalWrite(p2, HIGH);
  delay(50);
  s2.begin();
  s2.InitSensor(0x52);
  s2.VL53L4CX_SetDeviceAddress(0x54);

  digitalWrite(p3, HIGH);
  delay(50);
  s3.begin();
  s3.InitSensor(0x52);
}

void processSensors(VL53L4CX &sensor, const *char sensorName)
{
  uint8_t ready = 0;
  if (sensor.VL53L4CX_GetMeasurementDataReady(&ready) == 0 && ready != 0)
  {
    if (sensor.VL53L4CX_GetMultiRangingData(&MultiRangingData) == 0)
    {
      for (int j = 0; j < MultiRangingData.NumberOfObjectsFound; j++)
      {
        if (MultiRangingData.RangeData[j].RangeStatus == 0)
        {
          Serial.print(sensorName);
          Serial.print(": ");
          Serial.print(MultiRangingData.RangeData[j].RangeMilliMeter);
          Serial.println("mm");
        }
      }
      sensor.VL53L4CX_ClearInterruptandStartMeasurement();
    }
  }
}