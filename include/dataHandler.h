#pragma once


#include "ros/ros.h"
#include <map>
#include <algorithm>
#include "sensor_msgs/Imu.h" 

#include "pcl_conversions/pcl_conversions.h"
#include "lidar.h"
//#include <dmapping/utility.h>
#include "math.h"
using std::cout;
using std::endl;

/* A steam of time stamped data  for lookup*/
#define SCAN_RATE 10.0
namespace dmapping {


Eigen::Quaterniond Imu2Orientation(const sensor_msgs::Imu& data);

Eigen::Vector3d Imu2AngularVelocity(const sensor_msgs::Imu& data);

Eigen::Vector3d Imu2LinearAcceleration(const sensor_msgs::Imu& data);

void CompensateVelocity(pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr input, const Eigen::Vector3d& velocity);




// Imu Handler
typedef std::pair<double, sensor_msgs::Imu> stampedImu;

bool compare (const stampedImu i, const stampedImu& j);

class ImuHandler{

public:

  ImuHandler(){}

  void AddMsg(sensor_msgs::Imu::ConstPtr msg);

  bool Get(const double& tStamp, sensor_msgs::Imu& data)const;

  sensor_msgs::Imu Get(const double& tStamp) const; // Currently no interpolation - nice but not reuqired

  bool TimeContained(const double)const;

  std::vector<stampedImu>::iterator end() {return data_.end();}

  std::vector<stampedImu>::iterator begin() {return data_.begin();}

  std::size_t size(){return data_.size();}

private:

  void Add(const sensor_msgs::Imu& msg);

  std::vector<stampedImu> data_;

  double first = 0;


};

bool Compensate(pcl::PointCloud<vel_point::PointXYZIRT>&input, pcl::PointCloud<vel_point::PointXYZIRT>& compensated, ImuHandler& handler, Eigen::Quaterniond& extrinsics);

void CompensateVelocity(pcl::PointCloud<vel_point::PointXYZIRT>::Ptr input, const Eigen::Vector3d& velocity);


}


