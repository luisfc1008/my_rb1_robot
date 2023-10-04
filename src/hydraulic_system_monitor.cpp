#include "robot_info/hydraulic_system_monitor.h"
#include "robot_info/agv_robot_info.h"
#include <ros/ros.h>

HydraulicSystemMonitor::HydraulicSystemMonitor() {
  this->hydraulic_oil_temperature = "hydraulic_oil_temperature: 45C";
  this->hydraulic_oil_tank_fill_level = "hydraulic_oil_tank_fill_level: 100%";
  this->hydraulic_oil_pressure = "hydraulic_oil_pressure: 250 bar";
}

std::string HydraulicSystemMonitor::get_oil_temp() { 
    oil_temp = hydraulic_oil_temperature;
    return oil_temp; 
}

std::string HydraulicSystemMonitor::get_oil_level() { 
    oil_level = hydraulic_oil_tank_fill_level;
    return oil_level; 
}

std::string HydraulicSystemMonitor::get_oil_pressure() { 
    oil_pressure = hydraulic_oil_pressure;
    return oil_pressure; 
}