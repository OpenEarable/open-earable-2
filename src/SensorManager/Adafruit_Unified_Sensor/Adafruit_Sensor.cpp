#include "Adafruit_Sensor.h"

/**************************************************************************/
/*!
    @brief  Prints sensor information to serial console
*/
/**************************************************************************/
void Adafruit_Sensor::printSensorDetails(void) {
  sensor_t sensor;
  getSensor(&sensor);
  printk("------------------------------------\n");
  printk("Sensor:       ");
  printk(sensor.name);
  printk("\n");
  printk("Type:         ");
  switch ((sensors_type_t)sensor.type) {
  case SENSOR_TYPE_ACCELEROMETER:
    printk("Acceleration (m/s2)");
    break;
  case SENSOR_TYPE_MAGNETIC_FIELD:
    printk("Magnetic (uT)");
    break;
  case SENSOR_TYPE_ORIENTATION:
    printk("Orientation (degrees)");
    break;
  case SENSOR_TYPE_GYROSCOPE:
    printk("Gyroscopic (rad/s)");
    break;
  case SENSOR_TYPE_LIGHT:
    printk("Light (lux)");
    break;
  case SENSOR_TYPE_PRESSURE:
    printk("Pressure (hPa)");
    break;
  case SENSOR_TYPE_PROXIMITY:
    printk("Distance (cm)");
    break;
  case SENSOR_TYPE_GRAVITY:
    printk("Gravity (m/s2)");
    break;
  case SENSOR_TYPE_LINEAR_ACCELERATION:
    printk("Linear Acceleration (m/s2)");
    break;
  case SENSOR_TYPE_ROTATION_VECTOR:
    printk("Rotation vector");
    break;
  case SENSOR_TYPE_RELATIVE_HUMIDITY:
    printk("Relative Humidity (%)");
    break;
  case SENSOR_TYPE_AMBIENT_TEMPERATURE:
    printk("Ambient Temp (C)");
    break;
  case SENSOR_TYPE_OBJECT_TEMPERATURE:
    printk("Object Temp (C)");
    break;
  case SENSOR_TYPE_VOLTAGE:
    printk("Voltage (V)");
    break;
  case SENSOR_TYPE_CURRENT:
    printk("Current (mA)");
    break;
  case SENSOR_TYPE_COLOR:
    printk("Color (RGBA)");
    break;
  case SENSOR_TYPE_TVOC:
    printk("Total Volatile Organic Compounds (ppb)");
    break;
  case SENSOR_TYPE_VOC_INDEX:
    printk("Volatile Organic Compounds (Index)");
    break;
  case SENSOR_TYPE_NOX_INDEX:
    printk("Nitrogen Oxides (Index)");
    break;
  case SENSOR_TYPE_CO2:
    printk("Carbon Dioxide (ppm)");
    break;
  case SENSOR_TYPE_ECO2:
    printk("Equivalent/estimated CO2 (ppm)");
    break;
  case SENSOR_TYPE_PM10_STD:
    printk("Standard Particulate Matter 1.0 (ppm)");
    break;
  case SENSOR_TYPE_PM25_STD:
    printk("Standard Particulate Matter 2.5 (ppm)");
    break;
  case SENSOR_TYPE_PM100_STD:
    printk("Standard Particulate Matter 10.0 (ppm)");
    break;
  case SENSOR_TYPE_PM10_ENV:
    printk("Environmental Particulate Matter 1.0 (ppm)");
    break;
  case SENSOR_TYPE_PM25_ENV:
    printk("Environmental Particulate Matter 2.5 (ppm)");
    break;
  case SENSOR_TYPE_PM100_ENV:
    printk("Environmental Particulate Matter 10.0 (ppm)");
    break;
  case SENSOR_TYPE_GAS_RESISTANCE:
    printk("Gas Resistance (ohms)");
    break;
  case SENSOR_TYPE_UNITLESS_PERCENT:
    printk("Unitless Percent (%)");
    break;
  case SENSOR_TYPE_ALTITUDE:
    printk("Altitude (m)");
    break;
  }

  printk("\n");
  printk("Driver Ver:   ");
  printk("%i\n", sensor.version);
  printk("Unique ID:    ");
  printk("%i\n", sensor.sensor_id);
  printk("Min Value:    ");
  printk("%f\n", sensor.min_value);
  printk("Max Value:    ");
  printk("%f\n", sensor.max_value);
  printk("Resolution:   ");
  printk("%f\n", sensor.resolution);
  printk("------------------------------------\n\n");
}
