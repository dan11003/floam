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
#include <sensor_msgs/Imu.h>
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
#include "laserProcessingClass.h"
#include "dataHandler.h"


LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::mutex mutex_imu;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
std::queue<sensor_msgs::ImuConstPtr> imuBuf;
dmapping::ImuHandler imuHandler;
lidar::Lidar lidar_param;


ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;
Eigen::Quaterniond exstrinsics;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  //std::cout << "callback lidar" << std::endl;
  mutex_lock.lock();
  pointCloudBuf.push(laserCloudMsg);
  //std::cout << "lidar queue: " << pointCloudBuf.size() << std::endl;
  mutex_lock.unlock();

}
void imuSubscriber(const sensor_msgs::Imu::ConstPtr& imuMsg){
  //std::cout << "callback imu" << std::endl;
  mutex_imu.lock();
  imuBuf.push(imuMsg);
  mutex_imu.unlock();
}

double total_time =0;
int total_frame=0;

void CenterTime(const pcl::PointCloud<vel_point::PointXYZIRT>::Ptr cloud){
  ros::Time t;
  pcl_conversions::fromPCL(cloud->header.stamp,t);
  const double tScan = t.toSec();
  const double tEnd = tScan + cloud->points.back().time;
  const double tBegin = tScan + cloud->points.front().time;
  const double tCenter = tBegin + (tEnd - tBegin)/2.0;
  const ros::Time tRosCenter(tCenter);
  pcl_conversions::toPCL(tRosCenter, cloud->header.stamp);
  for(auto && pnt : cloud->points ){
      pnt.time = pnt.time + tScan - tCenter;
  }
  //std::cout << "adjust: " << tScan - tCenter << std::endl;
}

void laser_processing(){

  while(1){
    //std::cout << "new itr: "<< std::endl;
    mutex_imu.try_lock();
    while (!imuBuf.empty()) {
      imuHandler.AddMsg(imuBuf.front());
      imuBuf.pop();
    }
    mutex_imu.unlock();
    //std::cout << "laser queue: " << pointCloudBuf.size() << std::endl;
    if(pointCloudBuf.size() > 1){ // ensure some time has elapsed - daniel
      //std::cout <<"new itr, lidar-imu: ´" << imuHandler.size() << std::endl;

      //read data
      mutex_lock.lock();
      pcl::PointCloud<vel_point::PointXYZIRT>::Ptr pointcloud_in(new pcl::PointCloud<vel_point::PointXYZIRT>());
      pcl::PointCloud<vel_point::PointXYZIRT>::Ptr compensated(new pcl::PointCloud<vel_point::PointXYZIRT>());
      pcl::PointCloud<vel_point::PointXYZIRT>::Ptr imu_aligned(new pcl::PointCloud<vel_point::PointXYZIRT>());
      pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
      CenterTime(pointcloud_in);
      ros::Time pointcloud_time = pcl_conversions::fromPCL(pointcloud_in->header.stamp); // pointCloudBuf.front())->header.stamp;
      pointCloudBuf.pop();
      mutex_lock.unlock();

      //std::cout << pointcloud_time << std::endl;


      bool can_compensate = dmapping::Compensate(*pointcloud_in, *compensated, imuHandler, exstrinsics);
      if(!can_compensate){
        std::cerr << "cannot compensate - no IMU data" << std::endl;
        continue;
      }
      Eigen::Quaterniond q(dmapping::Imu2Orientation(imuHandler.Get(pointcloud_time.toSec()))*exstrinsics);
      Eigen::Affine3d ImuNowT(q);

      pcl::transformPointCloud(*compensated, *imu_aligned, ImuNowT);
      const ros::Time t = ros::Time::now();
      PublishCloud("process1_uncompensated", *pointcloud_in, "sensor", t);
      PublishCloud("process2_uncompensated", *compensated, "sensor", t);
      PublishCloud("process3_prediction", *imu_aligned, "sensor", t);
      pointcloud_in = imu_aligned;


      pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr pointcloud_edge(new pcl::PointCloud<vel_point::PointXYZIRTC>());
      pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr pointcloud_surf(new pcl::PointCloud<vel_point::PointXYZIRTC>());

      std::chrono::time_point<std::chrono::system_clock> start, end;
      start = std::chrono::system_clock::now();
      laserProcessing.featureExtraction(pointcloud_in, pointcloud_edge, pointcloud_surf);
      end = std::chrono::system_clock::now();
      std::chrono::duration<float> elapsed_seconds = end - start;
      total_frame++;
      float time_temp = elapsed_seconds.count() * 1000;
      total_time+=time_temp;
      //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

      sensor_msgs::PointCloud2 laserCloudFilteredMsg;
      pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr pointcloud_filtered(new pcl::PointCloud<vel_point::PointXYZIRTC>());
      *pointcloud_filtered+=*pointcloud_edge;
      *pointcloud_filtered+=*pointcloud_surf;
      /*pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
      laserCloudFilteredMsg.header.stamp = pointcloud_time;
      laserCloudFilteredMsg.header.frame_id = "base_link";
      pubLaserCloudFiltered.publish(laserCloudFilteredMsg);*/
      PublishCloud("/velodyne_points_filtered", *pointcloud_filtered, "base_link", pointcloud_time);

      PublishCloud("/laser_cloud_edge", *pointcloud_edge, "base_link", pointcloud_time);
      /*sensor_msgs::PointCloud2 edgePointsMsg;
      pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
      edgePointsMsg.header.stamp = pointcloud_time;
      edgePointsMsg.header.frame_id = "base_link";
      pubEdgePoints.publish(edgePointsMsg);*/


      PublishCloud("/laser_cloud_surf", *pointcloud_surf, "base_link", pointcloud_time);
      /*sensor_msgs::PointCloud2 surfPointsMsg;
      pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
      surfPointsMsg.header.stamp = pointcloud_time;
      surfPointsMsg.header.frame_id = "base_link";
      pubSurfPoints.publish(surfPointsMsg);*/

    }
    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  } //whil

} //void

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;

  int scan_line = 64;
  double vertical_angle = 2.0;
  double scan_period= 0.1;
  double max_dis = 60.0;
  double min_dis = 2.0;
  std::string imu_topic = "/imu/data";
  std::string velodyne_topic = "/velodyne_points";

  nh.getParam("/scan_period", scan_period);
  nh.getParam("/vertical_angle", vertical_angle);
  nh.getParam("/max_dis", max_dis);
  nh.getParam("/min_dis", min_dis);
  nh.getParam("/scan_line", scan_line);
  nh.getParam("/imu_topic", imu_topic);

  lidar_param.setScanPeriod(scan_period);
  lidar_param.setVerticalAngle(vertical_angle);
  lidar_param.setLines(scan_line);
  lidar_param.setMaxDistance(max_dis);
  lidar_param.setMinDistance(min_dis);

  exstrinsics = euler2Quaternion(0,0,180);

  laserProcessing.init(lidar_param);

  std::cout << "subscribe to Lidar data - topic: " << std::quoted(velodyne_topic) << std::endl;
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(velodyne_topic, 100, velodyneHandler);

  std::cout << "subscribe to IMU data - topic: " << std::quoted(imu_topic) << std::endl;
  ros::Subscriber subIMU = nh.subscribe<sensor_msgs::Imu>(imu_topic, 100000, imuSubscriber);

  /*pubLaserCloudFiltered = nh.advertise<pcl::PointCloud<vel_point::PointXYZIRT>>("/velodyne_points_filtered", 100);

  pubEdgePoints = nh.advertise<pcl::PointCloud<vel_point::PointXYZIRT>>("/laser_cloud_edge", 100);

  pubSurfPoints = nh.advertise<pcl::PointCloud<vel_point::PointXYZIRT>>("/laser_cloud_surf", 100);*/

  std::thread laser_processing_process{laser_processing};

  ros::spin();

  return 0;
}

