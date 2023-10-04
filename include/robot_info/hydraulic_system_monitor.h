#pragma once
#include <ros/ros.h>
#include <string>

class HydraulicSystemMonitor {

public:
  HydraulicSystemMonitor();
  std::string get_oil_temp();
  std::string get_oil_level();
  std::string get_oil_pressure();

protected:
  std::string oil_temp;
  std::string oil_level;
  std::string oil_pressure;
  std::string hydraulic_oil_temperature;
  std::string hydraulic_oil_tank_fill_level;
  std::string hydraulic_oil_pressure;
};