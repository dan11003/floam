// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "lidar.h"


Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
  Eigen::AngleAxisd rollAngle(roll*M_PI/180.0, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch*M_PI/180.0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw*M_PI/180.0, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
  return q;
}


lidar::Lidar::Lidar(){
 
}

void lidar::Lidar::setLines(double num_lines_in){
    num_lines=num_lines_in;
}


void lidar::Lidar::setVerticalAngle(double vertical_angle_in){
    vertical_angle = vertical_angle_in;
}


void lidar::Lidar::setVerticalResolution(double vertical_angle_resolution_in){
    vertical_angle_resolution = vertical_angle_resolution_in;
}


void lidar::Lidar::setScanPeriod(double scan_period_in){
    scan_period = scan_period_in;
}


void lidar::Lidar::setMaxDistance(double max_distance_in){
	max_distance = max_distance_in;
}

void lidar::Lidar::setMinDistance(double min_distance_in){
	min_distance = min_distance_in;
}
