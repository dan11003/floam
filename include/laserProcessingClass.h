// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LASER_PROCESSING_CLASS_H_
#define _LASER_PROCESSING_CLASS_H_
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include "lidar.h"
#include "pcl/filters/impl/filter.hpp"



//points covariance class
class Double2d{
public:
    int id;
    double value;
    Double2d(int id_in, double value_in);
};
//points info class
class PointsInfo{
public:
    int layer;
    double time;
    PointsInfo(int layer_in, double time_in);
};


class LaserProcessingClass 
{
public:
    LaserProcessingClass();

    void init(lidar::Lidar lidar_param_in);

    void featureExtraction(const pcl::PointCloud<vel_point::PointXYZIRT>::Ptr& pc_in, VelCurve::Ptr& pc_out_edge, VelCurve::Ptr& pc_out_surf, VelCurve::Ptr& pc_out_less_flat, VelCurve::Ptr& pc_out_less_edge);

    void featureExtractionFromSector(const pcl::PointCloud<vel_point::PointXYZIRT>::Ptr& pc_in, std::vector<Double2d>& cloudCurvature, VelCurve::Ptr& pc_out_edge, VelCurve::Ptr& pc_out_surf, VelCurve::Ptr& pc_out_less_flat, VelCurve::Ptr& pc_out_less_edge);

    void RingExtraction(const pcl::PointCloud<vel_point::PointXYZIRT>::Ptr& pc_in, std::vector<pcl::PointCloud<vel_point::PointXYZIRT>::Ptr> laserCloudScans);

    void RingExtractionVelodyne(const pcl::PointCloud<vel_point::PointXYZIRT>::Ptr& pc_in, std::vector<pcl::PointCloud<vel_point::PointXYZIRT>::Ptr>& laserCloudScans, std::vector<std::vector<double> >& range_image);

    void FloamFeatures(std::vector<pcl::PointCloud<vel_point::PointXYZIRT>::Ptr> laserCloudScans, VelCurve::Ptr& pc_out_edge, VelCurve::Ptr& pc_out_surf, VelCurve::Ptr& pc_out_less_flat, VelCurve::Ptr& pc_out_less_edge);


private:
    lidar::Lidar lidar_param;
};



#endif // _LASER_PROCESSING_CLASS_H_

