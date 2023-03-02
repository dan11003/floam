

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
#include "pcl/features/normal_3d.h"
#include "lio_sam/cloud_info.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "dataHandler.h"




using namespace sensor_msgs;
using namespace message_filters;

bool deskew = false;
OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
typedef struct
{
 sensor_msgs::PointCloud2ConstPtr pointCloudEdge;
 sensor_msgs::PointCloud2ConstPtr pointCloudSurf;
 sensor_msgs::PointCloud2ConstPtr pointCloudLessEdge;
 sensor_msgs::ImuConstPtr imu;
}ProcessedData;
std::queue<ProcessedData> ProcessedDataBuf;

ros::Publisher pubLaserCloudInfo;
lidar::Lidar lidar_param;
Dump dataStorage;
bool keep_running = true;
std::string sensor_link = "base_link";
std::string sensor_link_now = "base_link_now";
std::string odom_link = "odom";

ros::Publisher pubLaserOdometry;
ros::Publisher pubLaserOdometryNow;
Eigen::Isometry3d poseEstimate;

void TrippleCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsgEdge,
                     const sensor_msgs::PointCloud2ConstPtr &laserCloudMsgSurf,
                     const sensor_msgs::PointCloud2ConstPtr &laserCloudMsgLessEdge,
                     const sensor_msgs::ImuConstPtr& imuMsg)
{
  ProcessedDataBuf.push(ProcessedData{laserCloudMsgEdge, laserCloudMsgSurf, laserCloudMsgLessEdge, imuMsg});
}


bool is_odom_inited = false;
double total_time =0;
int total_frame=0;



void SaveMerged(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds, const std::vector<Eigen::Affine3d> poses, const std::string& directory, double downsample_size){
  boost::filesystem::create_directories(directory);
  pcl::PointCloud<pcl::PointXYZI>::Ptr merged_transformed(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI> merged_downsamapled;
  std::cout << "\"FLOAM\"  - Save merged point cloud to:\n" << directory << std::endl <<  std::endl;

  for(int i = 0; i < poses.size() ; i++) {
    pcl::PointCloud<pcl::PointXYZI> tmp_transformed;
    pcl::transformPointCloud(*clouds[i], tmp_transformed, poses[i]);
    *merged_transformed += tmp_transformed;
  }
  cout << "\"FLOAM\"  - Downsample point cloud resolution " << downsample_size << endl;


  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (merged_transformed);
  sor.setLeafSize (downsample_size, downsample_size, downsample_size);
  sor.filter (merged_downsamapled);

  pcl::io::savePCDFileBinary(directory + std::string("floam_merged.pcd"), *merged_transformed);
  if(!merged_downsamapled.empty()){
    const std::string path_downsampled = directory + std::string("floam_merged_downsampled_leaf_") + std::to_string(downsample_size) + ".pcd";
    pcl::io::savePCDFileBinary(path_downsampled, merged_downsamapled);
  }else{
    std::cout << "\"FLOAM\"  - No downsampled point cloud saved - increase \"output_downsample_size\"" << std::endl;
  }
}

void SavePosesHomogeneousBALM(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds, const std::vector<Eigen::Affine3d> poses, const std::string& directory, double downsample_size){
    boost::filesystem::create_directories(directory);
    const std::string filename = directory + "alidarPose.csv";
    std::fstream stream(filename.c_str(), std::fstream::out);
    std::cout << "\"FLOAM\"  - Save BALM to:\n" << directory << std::endl << std::endl;
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

void PublishInfo(nav_msgs::Odometry startOdomMsg, VelCurve::Ptr surf_in, VelCurve::Ptr edge_in, VelCurve::Ptr less_edge_in, VelCurve::Ptr deskewed, const ros::Time& thisStamp){
  lio_sam::cloud_info cloudInfo;
  cloudInfo.header.stamp = thisStamp;
  cloudInfo.header.frame_id = sensor_link;

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
  cloudInfo.cloud_surface.header.frame_id = sensor_link;

  pcl::toROSMsg(*edge_in, cloudInfo.cloud_corner);
  cloudInfo.cloud_corner.header.stamp = thisStamp;
  cloudInfo.cloud_corner.header.frame_id = sensor_link;

  pcl::toROSMsg(*less_edge_in, cloudInfo.cloud_less_edge);
  cloudInfo.cloud_less_edge.header.stamp = thisStamp;
  cloudInfo.cloud_less_edge.header.frame_id = sensor_link;


  pcl::toROSMsg(*deskewed, cloudInfo.cloud_deskewed);
  cloudInfo.cloud_deskewed.header.stamp = thisStamp;
  cloudInfo.cloud_deskewed.header.frame_id = sensor_link;

  pubLaserCloudInfo.publish(cloudInfo);
}
void Publish(const Eigen::Isometry3d& poseEstimate, const ros::Time& ros_cloud_time, VelCurve::Ptr& merged, VelCurve::Ptr& surf, VelCurve::Ptr& edge, VelCurve::Ptr& less_edge){
  Eigen::Quaterniond q_current(poseEstimate.rotation());
  Eigen::Vector3d t_current = poseEstimate.translation();

  const ros::Time tNow = ros::Time::now();
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
  tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros_cloud_time, odom_link, sensor_link));
  br.sendTransform(tf::StampedTransform(transform, tNow, odom_link, sensor_link_now));

  // publish odometry
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = odom_link;
  laserOdometry.child_frame_id = sensor_link;
  laserOdometry.header.stamp = ros_cloud_time; // ros::Time::now();
  laserOdometry.pose.pose.orientation.x = q_current.x();
  laserOdometry.pose.pose.orientation.y = q_current.y();
  laserOdometry.pose.pose.orientation.z = q_current.z();
  laserOdometry.pose.pose.orientation.w = q_current.w();
  laserOdometry.pose.pose.position.x = t_current.x();
  laserOdometry.pose.pose.position.y = t_current.y();
  laserOdometry.pose.pose.position.z = t_current.z();

  nav_msgs::Odometry laserOdometryNow = laserOdometry;
  laserOdometryNow.header.stamp = ros::Time::now();
  laserOdometryNow.child_frame_id = sensor_link_now;

  pubLaserOdometry.publish(laserOdometry);
  pubLaserOdometryNow.publish(laserOdometryNow);

  PublishCloud("/scan_registered", *merged, sensor_link_now, tNow);

  PublishInfo(laserOdometry, surf, edge, less_edge, merged, ros_cloud_time);

  merged->header.stamp = pcl_conversions::toPCL(ros_cloud_time);
  dataStorage.poses.push_back(poseEstimate);
  dataStorage.clouds.push_back(VelToIntensityCopy(merged));
  dataStorage.keyframe_stamps.push_back(ros_cloud_time.toSec());
}
void odom_estimation(){
    ros::Time tPrev = ros::Time::now();
    while(ros::ok()){
        if( total_frame > 0 && (ros::Time::now() -  tPrev > ros::Duration(3.0))  ){// The mapper has been running (total_frame > 0), but no new data for over a second  - rosbag play was stopped.
            keep_running = false;
            std::cout << "\"FLOAM\"  - No more data to process" << std::endl;
            break;
        }
        if(!ProcessedDataBuf.empty()){
            mutex_lock.lock();
            if(ProcessedDataBuf.size() > 10){
              std::cout <<"\"FLOAM\" - Slow processing - in queue: " << ProcessedDataBuf.size() << " scans."<< std::endl;
            }
            tPrev = ros::Time::now();
            VelCurve::Ptr pointcloud_edge_in(new VelCurve()); VelCurve::Ptr pointcloud_less_edge_in(new VelCurve()); VelCurve::Ptr pointcloud_surf_in(new VelCurve());
            VelCurve::Ptr merged(new VelCurve());

            pcl::fromROSMsg(*ProcessedDataBuf.front().pointCloudEdge, *pointcloud_edge_in);
            pcl::fromROSMsg(*ProcessedDataBuf.front().pointCloudSurf, *pointcloud_surf_in);
            pcl::fromROSMsg(*ProcessedDataBuf.front().pointCloudLessEdge, *pointcloud_less_edge_in);

            const ros::Time ros_cloud_time = (ProcessedDataBuf.front().pointCloudSurf)->header.stamp;
            const pcl::uint64_t pcl_time = pcl_conversions::toPCL(ros_cloud_time);
            pointcloud_edge_in->header.stamp = pointcloud_less_edge_in->header.stamp = pointcloud_surf_in->header.stamp = pcl_time;

            const sensor_msgs::Imu ImuData = *ProcessedDataBuf.front().imu;
            const Eigen::Quaterniond qCurrent = dmapping::Imu2Orientation(ImuData);

            ProcessedDataBuf.pop();
            mutex_lock.unlock();

            //cout << "itr - size: " << uncompensated_edge_in->size() << ", " << uncompensated_surf_in->size() << endl;
            const ros::Time t0 = ros::Time::now();
            odomEstimation.ProcessFrame(pointcloud_edge_in, pointcloud_surf_in, pointcloud_less_edge_in, qCurrent, poseEstimate);
            const double velocity = odomEstimation.GetVelocity().norm();
            total_frame++;
            const float time_temp = (ros::Time::now()-t0).toSec();
            total_time+=time_temp;
            ROS_INFO("\"FLOAM\" - Frame: %d. Average time / frame %lf [ms]. Speed %lf [m/s]\n",total_frame, total_time/total_frame, velocity);
            *merged += *pointcloud_surf_in; *merged += *pointcloud_edge_in; *merged += *pointcloud_less_edge_in;
            Publish(poseEstimate, ros_cloud_time, merged, pointcloud_surf_in, pointcloud_edge_in, pointcloud_less_edge_in);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
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
    bool export_pcd = true;


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
    nh.param<bool>("/odom_save_balm", save_BALM, false);
    nh.param<bool>("/odom_save_posegraph", save_Posegraph, false);
    nh.param<bool>("/odom_save_odom", save_odom, false);
    nh.param<bool>("/export_odom_pcd", export_pcd, true);
    cout << "FLOAM save_BALM: " << save_BALM << ", save_Posegraph: " << save_Posegraph << ", save_odom: " << save_odom << ", export_slam_pcd: " << export_pcd << endl;

    directory = IO::CreateFolder(directory, "FLOAM", "FLOAM");


    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, map_resolution, loss_function);

    message_filters::Subscriber<PointCloud2> sub_edge(nh, "/laser_cloud_edge", 1);
    message_filters::Subscriber<PointCloud2> sub2_surf(nh, "/laser_cloud_surf", 1);
    message_filters::Subscriber<PointCloud2> sub3_less_edge(nh, "/laser_cloud_less_edge", 1);
    message_filters::Subscriber<Imu> sub_imu(nh, "/synced/imu/data", 1);

    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2, Imu> MySyncPolicy;
     // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_edge, sub2_surf, sub3_less_edge, sub_imu);
    sync.registerCallback(boost::bind(&TrippleCallback, _1, _2, _3, _4));

    pubLaserOdometryNow = nh.advertise<nav_msgs::Odometry>("/odom_now", 100);
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
    std::thread odom_estimation_process{odom_estimation};

    ros::Rate r(100); // 10 hz
    while (keep_running){
      ros::spinOnce();
      r.sleep();
    }


    if(export_pcd){
      std::cout << "\"FLOAM\" - output directory: " << directory << std::endl;
      std::cout << "\"FLOAM\" - Poses: " <<dataStorage.poses.size() << ", Scans: " <<dataStorage.clouds.size() << std::endl;
      SaveMerged(dataStorage.clouds, dataStorage.poses, directory, output_downsample_size);
    }
    else{
      std::cout << "\"FLOAM\"  - Saving disabled " << std::endl;
    }
    if(save_BALM && export_pcd){
      //cout << "Save BALM data " << endl;
      SavePosesHomogeneousBALM(dataStorage.clouds, dataStorage.poses, directory + "BALM/", output_downsample_size);
    }
    if(save_Posegraph && export_pcd){
      //cout << "Save Posegraph" << endl;
      SavePosegraph(directory + "posegraph", dataStorage.poses, dataStorage.keyframe_stamps, dataStorage.clouds );
    }
    if(save_odom && export_pcd){
      //cout << "Save Posegraph" << endl;
      SaveOdom(directory + "odom", dataStorage.poses, dataStorage.keyframe_stamps, dataStorage.clouds);
    }
    std::cout << "\"FLOAM\" - Program finished nicely" << std::endl << std::endl;

  return 0;
}

