// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _ODOM_ESTIMATIONDMAPPING_CLASS_H_
#define _ODOM_ESTIMATIONDMAPPING_CLASS_H_

//std lib
#include <string>
#include <math.h>
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include "pcl/common/transforms.h"

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include "lidar.h"
#include "lidarOptimization.h"
#include "dataHandler.h"
#include <ros/ros.h>
#include <sstream>
#include "string.h"
#include "utils.h"
#include <pcl/features/normal_3d.h>
#include <pcl/filters/random_sample.h>
#include "odomEstimationClass.h"
using std::cout;
using std::endl;
namespace dmapping {


typedef struct
{
  Eigen::Isometry3d pose; // odometry frame
  SurfElCloud cloud;
}keyframe;

typedef std::vector<keyframe> keyframes;

class OdomEstimationDmapping
{
public:

  OdomEstimationDmapping();

  void init(lidar::Lidar lidar_param, double map_resolution, const std::string& loss_function);

  void ProcessFrame(SurfElCloud& cloud, const Eigen::Quaterniond& qImu, Eigen::Isometry3d& odom_out);


  Eigen::Vector3d GetVelocity(){return (odom.translation() - last_odom.translation())/lidar_param_.scan_period;}

private:

  void initMapWithPoints(const SurfElCloud& cloud, const Eigen::Quaterniond& qImu);

  void RegisterAndUpdate(const SurfElCloud& cloud, const Eigen::Quaterniond& qImu, bool update_map);

  void getMap(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& laserCloudMap);

  bool KeyFrameUpdate(const SurfElCloud& cloud, const Eigen::Isometry3d& pose); // determines if the current pose is a new keyframe

  void addSurfCostFactor(const SurfElCloud& cloud,
                         const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& map_in,
                         ceres::Problem& problem,
                         ceres::LossFunction *loss_function);

  void addPointsToMap(const SurfElCloud& cloud);

  void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& surf_pc_out);

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudSurfMap;

  ceres::LossFunction* GetLoss(const std::string& lossFunction);

  // VARIABLES
  double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
  Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);
  Eigen::Isometry3d odom;
  Eigen::Isometry3d last_odom;
  Eigen::Quaterniond imu_prev;

  pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtreeSurfMap; //kd-tree
  pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilterSurf; //points downsampling before add to map
  pcl::CropBox<pcl::PointXYZINormal> cropBoxFilter; //local map


  std::string loss_function_; // Loss function in optimization
  lidar::Lidar lidar_param_;
  int optimization_count;

  const double keyframe_min_transl_ = 0.01; // 0.1 m
  const double keyframe_min_rot_ = 5*M_PI/180.0;    //or 5 deg
  size_t keyframe_history_ = 3;
  keyframes keyframes_;
  bool odom_initiated_ = false;
  bool first_map_update_ = true;


  //function
};

}
#endif // _ODOM_ESTIMATION_CLASS_H_

