// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _ODOM_ESTIMATION_CLASS_H_
#define _ODOM_ESTIMATION_CLASS_H_

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
using std::cout;
using std::endl;
pcl::PointCloud<pcl::PointXYZI>::Ptr VelToIntensityCopy(const pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr VelCloud);


class SurfelExtraction
{
public:
  SurfelExtraction(VelCurve::Ptr& surf_in, lidar::Lidar& lidar_par);

  void Extract(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& normals);

private:

  void LineNNSearch( const int ring, const double query, int &row, Eigen::MatrixXd& neighbours);

  bool GetNeighbours(const vel_point::PointXYZIRTC& pnt, Eigen::MatrixXd& neighbours);

  bool EstimateNormal(const vel_point::PointXYZIRTC& pnt, pcl::PointXYZINormal& output);

  void Initialize();

  lidar::Lidar lidar_par_;
  VelCurve::Ptr surf_in_;
  std::vector<VelCurve::Ptr> ringClouds_; //sorted in time, and segmented per ring
  std::vector<std::vector<double> > times_;

  pcl::PointXYZINormal defaultNormal;

};









typedef struct
{
  Eigen::Isometry3d pose; // odometry frame
  pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud; // sensor frame
  pcl::PointCloud<pcl::PointXYZI>::Ptr surf_cloud; // sensor frame
}keyframe;

typedef std::vector<keyframe> keyframes;

class OdomEstimationClass 
{
    public:

    typedef enum{VANILLA, INITIAL_ITERATION, REFINEMENT_AND_UPDATE} UpdateType;

    OdomEstimationClass();

    void init(lidar::Lidar lidar_param, double map_resolution, const std::string& loss_function);

    void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in);

    void UpdatePointsToMapSelector(pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr& edge_in, pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr& surf_in, bool deskew);

    /*!
         * \brief updatePointsToMap
         * \param edge_in
         * \param surf_in
         * \param initial_iteration if set to true, perform one step using constant
         */
    void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in, const UpdateType update_type = UpdateType::VANILLA); // ,

    void updatePointsToMap(const pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr& edge_in, const pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr& surf_in, const UpdateType update_type = UpdateType::VANILLA); // ,

    void getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap);

    Eigen::Vector3d GetVelocity(){return (odom.translation() - last_odom.translation())/lidar_param_.scan_period;}

    bool KeyFrameUpdate(pcl::PointCloud<pcl::PointXYZI>::Ptr surf_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud, const Eigen::Isometry3d& pose); // determines if the current pose is a new keyframe



    Eigen::Isometry3d odom;

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerMap;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfMap;
private:
    //optimization variable
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
    Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);
    Eigen::Isometry3d last_odom;

    //kd-tree
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeMap;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfMap;

    //points downsampling before add to map
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterEdge;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;

    //local map
    pcl::CropBox<pcl::PointXYZI> cropBoxFilter;

    //optimization count
    int optimization_count;

    // Loss function in optimization
    std::string loss_function_;

    lidar::Lidar lidar_param_;

    const double keyframe_min_transl_ = 0.07; // 0.1 m
    const double keyframe_min_rot_ = 2*M_PI/180.0;    //or 5 deg
    size_t keyframe_history_ = 3;
    keyframes keyframes_;


    //function
    void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
    void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
    void addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud);
    void pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po);
    void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out);
};

#endif // _ODOM_ESTIMATION_CLASS_H_

