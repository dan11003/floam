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
#include "pcl_ros/publisher.h"
#include "lio_sam/generics.h"
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_kdl.h"

using std::endl;
using std::cout;



inline vel_point::PointXYZIRTC ToCurvature(const PointType& pnt, const double curvature){
    PointType tmp;
    tmp.x = pnt.x; tmp.y = pnt.y; tmp.z = pnt.z;
    tmp.ring = pnt.ring; tmp.time = pnt.time; tmp.intensity = pnt.intensity; tmp.curvature = curvature;
    return tmp;
}
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

template<class T>
void PublishCloud(const std::string& topic, pcl::PointCloud<T>& cloud, const std::string& frame_id, const ros::Time& t, ros::NodeHandle& nh){

    pcl_conversions::toPCL(t,cloud.header.stamp);
    cloud.header.frame_id = frame_id;

    static std::map<std::string, ros::Publisher> pubs;
    std::map<std::string, ros::Publisher>::iterator it = pubs.find(topic);

    if (it == pubs.end()){
        pubs[topic] =  nh.advertise<pcl::PointCloud<T>>(topic,100);
        pubs[topic].publish(cloud);
        usleep(100*1000);
    }
    pubs[topic].publish(cloud);
}


template<class T>
void PublishCloud(const std::string& topic, pcl::PointCloud<T>& cloud, const std::string& fix_frame_id, const std::string& child_frame_id, const Eigen::Affine3d& Teig, const ros::Time& t, ros::NodeHandle& nh){

    static tf::TransformBroadcaster Tbr;
    tf::Transform Tf;
    tf::transformEigenToTF(Teig, Tf);
    std::vector<tf::StampedTransform> trans_vek = {tf::StampedTransform(Tf, t, fix_frame_id, child_frame_id)};
    Tbr.sendTransform(trans_vek);
    pcl_conversions::toPCL(t, cloud.header.stamp);
    cloud.header.frame_id = child_frame_id;

    static std::map<std::string, ros::Publisher> pubs;
    std::map<std::string, ros::Publisher>::iterator it = pubs.find(topic);

    if (it == pubs.end()){
        pubs[topic] =  nh.advertise<pcl::PointCloud<T>>(topic,100);
        usleep(100*1000);
        pubs[topic].publish(cloud);
        usleep(100*1000);
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

