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
#include "lio_sam/cloud_info.h"

using std::endl;
using std::cout;

bool deskew = false;
OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
ros::Publisher pubLaserCloudInfo;
lidar::Lidar lidar_param;
Dump dataStorage;
bool keep_running = true;

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



void SaveMerged(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds, const std::vector<Eigen::Affine3d> poses, const std::string& directory, double downsample_size){
  boost::filesystem::create_directories(directory);
  pcl::PointCloud<pcl::PointXYZI>::Ptr merged_transformed(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI> merged_downsamapled;
  std::cout << "Save merged point cloud to:\n" << directory << std::endl <<  std::endl;

  for(int i = 0; i < poses.size() ; i++) {
      pcl::PointCloud<pcl::PointXYZI> tmp_transformed;
      pcl::transformPointCloud(*clouds[i], tmp_transformed, poses[i]);
      *merged_transformed += tmp_transformed;
  }
  cout << "Downsample point cloud resolution " << downsample_size << endl;


  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (merged_transformed);
  sor.setLeafSize (downsample_size, downsample_size, downsample_size);
  sor.filter (merged_downsamapled);

  pcl::io::savePCDFileBinary(directory + std::string("floam_merged.pcd"), *merged_transformed);
  if(!merged_downsamapled.empty()){
    const std::string path_downsampled = directory + std::string("floam_merged_downsampled_leaf_") + std::to_string(downsample_size) + ".pcd";
    pcl::io::savePCDFileBinary(path_downsampled, merged_downsamapled);
  }else{
    std::cout << "No downsampled point cloud saved - increase \"output_downsample_size\"" << std::endl;
  }




}
void SavePosesHomogeneousBALM(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds, const std::vector<Eigen::Affine3d> poses, const std::string& directory, double downsample_size){
    boost::filesystem::create_directories(directory);
    const std::string filename = directory + "alidarPose.csv";
    std::fstream stream(filename.c_str(), std::fstream::out);
    std::cout << "Save BALM to:\n" << directory << std::endl << std::endl;
    /*std::cout << "Saving clouds size: " <<clouds.size() << std::endl;;
    std::cout << "Saving poses size: " <<poses.size() << std::endl;*/
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
    }
}
void Save(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const Eigen::Affine3d& T){

}
void PublishInfo(nav_msgs::Odometry startOdomMsg, pcl::PointCloud<vel_point::PointXYZIRT>::Ptr surf_in, pcl::PointCloud<vel_point::PointXYZIRT>::Ptr edge_in, pcl::PointCloud<vel_point::PointXYZIRT>::Ptr deskewed, const ros::Time& thisStamp){
  lio_sam::cloud_info cloudInfo;
  cloudInfo.header.stamp = thisStamp;
  cloudInfo.header.frame_id = "velodyne";

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

  double roll, pitch, yaw;
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  // Initial guess used in mapOptimization
  cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
  cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
  cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
  cloudInfo.initialGuessRoll  = roll;
  cloudInfo.initialGuessPitch = pitch;
  cloudInfo.initialGuessYaw   = yaw;
  cloudInfo.imuYawInit = yaw;
  cloudInfo.imuPitchInit = pitch; //can be replaced with measured imu orientation!!
  cloudInfo.imuRollInit = roll;
  cloudInfo.imuAvailable = true;

  cloudInfo.odomAvailable = true;


  pcl::toROSMsg(*surf_in, cloudInfo.cloud_surface);
  cloudInfo.cloud_surface.header.stamp = thisStamp;
  cloudInfo.cloud_surface.header.frame_id = "velodyne";

  pcl::toROSMsg(*surf_in, cloudInfo.cloud_corner);
  cloudInfo.cloud_corner.header.stamp = thisStamp;
  cloudInfo.cloud_corner.header.frame_id = "velodyne";


  pcl::toROSMsg(*deskewed, cloudInfo.cloud_deskewed);
  cloudInfo.cloud_deskewed.header.stamp = thisStamp;
  cloudInfo.cloud_deskewed.header.frame_id = "velodyne";

  pubLaserCloudInfo.publish(cloudInfo);

}
void odom_estimation(){
    ros::Time tPrev = ros::Time::now();
    while(ros::ok()){
        if( total_frame > 0 && (ros::Time::now() -  tPrev > ros::Duration(1.0))  ){// The mapper has been running (total_frame > 0), but no new data for over a second  - rosbag play was stopped.
            keep_running = false;
            std::cout << "No more data to process \"odomEstimationNode.cpp\"" << std::endl;
            break;
        }
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            //read data
            mutex_lock.lock();
            if(pointCloudSurfBuf.size() > 10){
            std::cout <<"Slow processing - in queue: " << pointCloudSurfBuf.size() << " scans."<< std::endl;
            }

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
            tPrev = ros::Time::now();
            //if time aligned 

            pcl::PointCloud<vel_point::PointXYZIRT>::Ptr pointcloud_edge_in(new pcl::PointCloud<vel_point::PointXYZIRT>());
            pcl::PointCloud<vel_point::PointXYZIRT>::Ptr pointcloud_surf_in(new pcl::PointCloud<vel_point::PointXYZIRT>());
            pcl::PointCloud<vel_point::PointXYZIRT>::Ptr merged(new pcl::PointCloud<vel_point::PointXYZIRT>());
            pcl::PointCloud<vel_point::PointXYZIRT>::Ptr merged_raw(new pcl::PointCloud<vel_point::PointXYZIRT>());


            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            pcl_conversions::toPCL(pointCloudEdgeBuf.front()->header.stamp, merged->header.stamp);
            pcl_conversions::toPCL(pointCloudEdgeBuf.front()->header.stamp, merged_raw->header.stamp);
            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            *merged_raw += *pointcloud_edge_in;
            *merged_raw += *pointcloud_surf_in;


            //cout << "itr - size: " << uncompensated_edge_in->size() << ", " << uncompensated_surf_in->size() << endl;
            if(is_odom_inited == false){
                pcl::PointCloud<pcl::PointXYZI>::Ptr uncompensated_edge_in = VelToIntensityCopy(pointcloud_edge_in);
                pcl::PointCloud<pcl::PointXYZI>::Ptr uncompensated_surf_in = VelToIntensityCopy(pointcloud_surf_in);
                odomEstimation.initMapWithPoints(uncompensated_edge_in, uncompensated_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
            }else{
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.UpdatePointsToMapSelector(pointcloud_edge_in, pointcloud_surf_in, deskew);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                ROS_INFO("Frame: %d. Average time / frame %lf [ms]. Speed %lf [m/s]\n",total_frame, total_time/total_frame, odomEstimation.GetVelocity().norm());

            }
            *merged += *pointcloud_surf_in;
            *merged += *pointcloud_edge_in;



            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            //q_current.normalize();
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            const ros::Time tNow = ros::Time::now();
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, pointcloud_time, "map", "base_link"));
            br.sendTransform(tf::StampedTransform(transform, tNow, "map", "sensor"));

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

            PublishCloud("/scan_registered", *merged, "sensor", tNow);

            PublishInfo(laserOdometry, pointcloud_surf_in, pointcloud_edge_in, merged, pointcloud_time);
            //void PublishInfo(nav_msgs::Odometry startOdomMsg, pcl::PointCloud<pcl::PointXYZI>::Ptr surf_in, pcl::PointCloud<pcl::PointXYZI>::Ptr edge_in, pcl::PointCloud<pcl::PointXYZI>::Ptr deskewed, const ros::Time& thisStamp){

            //PublishCloud("/scan_registered_uncompensated", *merged_raw, "sensor", tNow);

            Eigen::Matrix4d Trans; // Your Transformation Matrix
            Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
            Trans.block<3,3>(0,0) = q_current.toRotationMatrix();;
            Trans.block<3,1>(0,3) = t_current;
            Eigen::Affine3d eigTransformNow(Trans);

            pcl_conversions::toPCL(pointcloud_time,merged->header.stamp);

            dataStorage.poses.push_back(eigTransformNow);
            dataStorage.clouds.push_back(VelToIntensityCopy(merged));
            dataStorage.keyframe_stamps.push_back(pointcloud_time.toSec());
            //storeage.push_back(VelToIntensityCopy(merged));
            //g_poses.push_back(eigTransformNow);

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
    double output_downsample_size = 0.05;
    std::string loss_function = "Huber";
    bool save_Posegraph = false;
    bool save_BALM = true;
    bool save_odom = false;


    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);
    nh.getParam("/directory_output", directory);
    nh.getParam("/output_downsample_size", output_downsample_size);
    nh.getParam("/loss_function", loss_function);
    nh.getParam("/deskew", deskew);
    nh.getParam("/save_BALM", save_BALM);
    nh.getParam("/save_Posegraph", save_Posegraph);
    nh.getParam("/save_odom", save_odom);
    directory = CreateFolder(directory);



    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, map_resolution, loss_function);
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
    std::thread odom_estimation_process{odom_estimation};

    ros::Rate r(100); // 10 hz
    while (keep_running){
      ros::spinOnce();
      r.sleep();
    }

    std::cout << "output directory: " << directory << std::endl;
    std::cout << "Poses: " <<dataStorage.poses.size() << ", Scans: " <<dataStorage.clouds.size() << std::endl;
    SaveMerged(dataStorage.clouds, dataStorage.poses, directory, output_downsample_size);
    if(save_BALM){
      //cout << "Save BALM data " << endl;
      SavePosesHomogeneousBALM(dataStorage.clouds, dataStorage.poses, directory + "BALM/", output_downsample_size);
    }
    if(save_Posegraph){
      //cout << "Save Posegraph" << endl;
      SavePosegraph(directory + "posegraph", dataStorage.poses, dataStorage.keyframe_stamps, dataStorage.clouds );
    }
    if(save_odom){
      //cout << "Save Posegraph" << endl;
      SaveOdom(directory + "odom", dataStorage.poses, dataStorage.keyframe_stamps, dataStorage.clouds);
    }
    std::cout << "Program finished nicely" << std::endl << std::endl;

    return 0;
}

