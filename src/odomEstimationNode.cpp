// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimationClass.h"
#include "fstream"
#include "iostream"
#include "stdio.h"
#include "string.h"

using std::endl;
using std::cout;
OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry;
void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

bool is_odom_inited = false;
double total_time =0;
int total_frame=0;
std::vector<Eigen::Affine3d> g_poses;
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> g_clouds;

void SavePosesHomogeneousBALM(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds, const std::vector<Eigen::Affine3d> poses, const std::string& directory){
    const std::string filename = directory + "alidarPose.csv";
    std::fstream stream(filename.c_str(), std::fstream::out);
    std::cout << "clouds size: " <<clouds.size() << std::endl;;
    std::cout << "poses size: " <<poses.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_transformed(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI> merged_downsamapled;


    for(int i = 0; i < poses.size() ; i++) {
        ros::Time tRos;
        pcl_conversions::fromPCL(clouds[i]->header.stamp, tRos);
        const double time = tRos.toSec();
        const Eigen::MatrixXd m = poses[i].matrix();

        stream <<std::fixed <<m(0,0) <<  "," << m(0,1) <<  "," << m(0,2) <<  "," << m(0,3) <<  "," << endl <<
                                m(1,0) <<  "," << m(1,1) <<  "," << m(1,2) <<  "," << m(1,3) <<  "," << endl <<
                                m(2,0) <<  "," << m(2,1) <<  "," << m(2,2) <<  "," << m(2,3) <<  "," << endl <<
                                m(3,0) <<  "," << m(3,1) <<  "," << m(3,2) <<  "," << time <<  "," << endl;
        const std::string pcdPath = directory + std::string("full") + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(pcdPath, *clouds[i]);
        pcl::PointCloud<pcl::PointXYZI> tmp_transformed;
        pcl::transformPointCloud(*clouds[i], tmp_transformed, poses[i]);
        *merged_transformed += tmp_transformed;
    }
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (merged_transformed);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (merged_downsamapled);
    pcl::io::savePCDFileBinary(directory + std::string("fmerged.pcd"), *merged_transformed);
    pcl::io::savePCDFileBinary(directory + std::string("fmerged_downsampled.pcd"), merged_downsamapled);



}
void Save(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const Eigen::Affine3d& T){

}
void odom_estimation(){
    while(1){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }
            //if time aligned 

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());


            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            *merged += *pointcloud_surf_in;
            *merged += *pointcloud_edge_in;
            pcl_conversions::toPCL(pointCloudEdgeBuf.front()->header.stamp, merged->header.stamp);
            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
            }else{
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                ROS_INFO("average odom estimation time %f ms \n \n", total_time/total_frame);
            }



            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            //q_current.normalize();
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, pointcloud_time, "map", "base_link"));

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "base_link";
            laserOdometry.header.stamp = pointcloud_time; // ros::Time::now();
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            pubLaserOdometry.publish(laserOdometry);
            g_clouds.push_back(merged);

            Eigen::Matrix4d Trans; // Your Transformation Matrix
            Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
            Trans.block<3,3>(0,0) = q_current.toRotationMatrix();;
            Trans.block<3,1>(0,3) = t_current;
            Eigen::Affine3d eigTransformNow(Trans);
            g_poses.push_back(eigTransformNow);



        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
std::string CreateFolder(const std::string& basePath){

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  //const std::string timeStr(std::put_time(&tm, "%Y-%m-%d_%H-%M"));

  std::time_t now = std::time(NULL);
  std::tm * ptm = std::localtime(&now);
  char buffer[32];
  // Format: Mo, 15.06.2009 20:20:00
  std::strftime(buffer, 32, "%a_%Y.%m.%d_%H:%M:%S", ptm);


  const std::string dir = basePath + "/" + std::string(buffer) + std::string("/");
  std::cout << dir << std::endl;
  if (boost::filesystem::create_directories(dir)){
      std::cout << "Created new directory" << "\n";
  }
  return dir;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    std::string directory;
    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);
    nh.getParam("/directory_output", directory);
    directory = CreateFolder(directory);



    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, map_resolution);
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    std::thread odom_estimation_process{odom_estimation};

    ros::spin();
    SavePosesHomogeneousBALM(g_clouds, g_poses, directory);

    std::cout << "output directory:\n" << std::endl;
    std::cout << directory << std::endl;

    return 0;
}

