// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LIDAR_H_
#define _LIDAR_H_

//define lidar parameter
#include "stdio.h"
#include "pcl/point_types.h"
#include "eigen3/Eigen/Dense"
#include "pcl_ros/point_cloud.h"
#include"pcl_ros/publisher.h"

namespace vel_point{
struct PointXYZIRT
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float         intensity;            ///< laser intensity reading
  std::uint16_t ring;                 ///< laser ring number
  float         time;                 ///< laser time reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
}
EIGEN_ALIGN16;
}  // namespace velodyne_pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(vel_point::PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (float, time, time))

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);

template<class T>
void PublishCloud(const std::string& topic, pcl::PointCloud<T>& cloud, const std::string& frame_id, const ros::Time& t){

  pcl_conversions::toPCL(t,cloud.header.stamp);
  cloud.header.frame_id = frame_id;

  static std::map<std::string, ros::Publisher> pubs;
  std::map<std::string, ros::Publisher>::iterator it = pubs.find(topic);
  static ros::NodeHandle nh("~");
  if (it == pubs.end()){
    pubs[topic] =  nh.advertise<pcl::PointCloud<T>>(topic,100);
  }
  pubs[topic].publish(cloud);
}

namespace lidar{

class Lidar
{
public:
  Lidar();

  void setScanPeriod(double scan_period_in);
  void setLines(double num_lines_in);
  void setVerticalAngle(double vertical_angle_in);
  void setVerticalResolution(double vertical_angle_resolution_in);
  //by default is 100. pls do not change
  void setMaxDistance(double max_distance_in);
  void setMinDistance(double min_distance_in);

  double max_distance;
  double min_distance;
  int num_lines;
  double scan_period;
  int points_per_line;
  double horizontal_angle_resolution;
  double horizontal_angle;
  double vertical_angle_resolution;
  double vertical_angle;

  void GetParamFromRos(ros::NodeHandle& nh){

    nh.getParam("/scan_period", scan_period);
    nh.getParam("/vertical_angle", vertical_angle);
    nh.getParam("/max_dis", max_distance);
    nh.getParam("/min_dis", min_distance);
    nh.getParam("/scan_line", num_lines);

  }
};


}


#endif // _LIDAR_H_

