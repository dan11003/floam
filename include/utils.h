#ifndef UTILS_H
#define UTILS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "stdio.h"
#include "pcl/point_types.h"
#include "eigen3/Eigen/Dense"
#include "pcl_ros/point_cloud.h"
#include "string"
#include <boost/format.hpp>
#include <ros/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <boost/filesystem.hpp>
#include "lidar.h"
#include "sensor_msgs/Imu.h"


using boost::format;
using boost::io::group;


using std::cout; using std::cerr; using std::endl;
typedef struct
{
  std::string dump_directory;
  std::vector<Eigen::Affine3d> poses;
  std::vector<double> keyframe_stamps;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;

}Dump;

void SavePosegraph(
    const std::string& dump_directory,
    const std::vector<Eigen::Affine3d>& poses,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds);

void SaveOdom(
    const std::string& dump_directory,
    const std::vector<Eigen::Affine3d>& poses,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds);

sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in, const Eigen::Quaterniond& extQRPY);



//Runtime: 72 ms, faster than 99.56% of C++ online submissions for Find K Closest Elements.
//Memory Usage: 31 MB, less than 16.67% of C++ online submissions for Find K Closest Elements.
class NNSearchArray {
public:
    std::vector<int> findClosestElements(std::vector<double>& arr, int k, float max, float query);
};



#endif // UTILS_H
