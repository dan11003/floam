// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationDmapping.h"
namespace dmapping {

void OdomEstimationDmapping::init(lidar::Lidar lidar_param, double map_resolution, const std::string& loss_function){
  downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);
  kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZINormal>());
  optimization_count = 2;
  odom = Eigen::Isometry3d::Identity();
  last_odom = Eigen::Isometry3d::Identity();
  loss_function_ = boost::algorithm::to_lower_copy(loss_function);
  lidar_param_ = lidar_param;
}


void OdomEstimationDmapping::ProcessFrame(SurfElCloud& cloud, const Eigen::Quaterniond& qImu, Eigen::Isometry3d& odom_out){

  if(odom_initiated_ == false){
    odom = Eigen::Isometry3d(qImu);
    imu_prev = qImu;
    last_odom = odom;
    addPointsToMap(cloud);
    odom_initiated_ = true;
  }else{
    RegisterAndUpdate(cloud, qImu, false);
    //Eigen::Vector3d velocity = GetVelocity();
    //dmapping::CompensateVelocity(cloud, velocity);
    //dmapping::CompensateVelocity(cloud, velocity);
    RegisterAndUpdate(cloud, qImu, true);
  }
  odom_out = odom;
}

ceres::LossFunction* OdomEstimationDmapping::GetLoss(const std::string& lossFunction){
  if(lossFunction == "huber")
    return new ceres::HuberLoss(0.1);
 else
    return new ceres::CauchyLoss(0.2);
}
void OdomEstimationDmapping::RegisterAndUpdate(const SurfElCloud& cloud, const Eigen::Quaterniond& qImu, bool update_map){

  if(optimization_count>2)
    optimization_count--;

  const Eigen::Vector3d transl_prediction = odom.translation() + (odom.translation()  - last_odom.translation());
  const Eigen::Quaterniond qOdom(odom.linear());
  const Eigen::Quaterniond rot_prediction = qOdom*imu_prev.inverse()*qImu;
  const Eigen::Isometry3d odom_prediction = EigenCombine(rot_prediction, transl_prediction);
  imu_prev = qImu;

  q_w_curr = Eigen::Quaterniond(odom_prediction.rotation());
  t_w_curr = odom_prediction.translation();

  /*pcl::PointCloud<pcl::PointXYZINormal>::Ptr currentScan = cloud.GetPointCloud();
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZINormal>());

  downSamplingToMap(currentScan, downsampledSurfCloud);*/

  //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
  if(laserCloudSurfMap->points.size()>50){
    kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

    for (int iterCount = 0; iterCount < optimization_count; iterCount++){

      ceres::LossFunction *loss_function = GetLoss(loss_function_);

      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options);

      problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
      addSurfCostFactor(cloud, laserCloudSurfMap, problem, loss_function);

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 4;
      options.minimizer_progress_to_stdout = false;
      options.check_gradients = false;
      options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;

      ceres::Solve(options, &problem, &summary);

    }
  }else{
    printf("not enough points in map to associate, map error");
  }
  odom = Eigen::Isometry3d::Identity();
  odom.linear() = q_w_curr.toRotationMatrix();
  odom.translation() = t_w_curr;
  if(update_map){
    const bool UpdateMap = KeyFrameUpdate(cloud, odom);
    if(UpdateMap){
      addPointsToMap(cloud);
    }
  }
}

void OdomEstimationDmapping::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& surf_pc_out){
  downSizeFilterSurf.setInputCloud(surf_pc_in);
  downSizeFilterSurf.filter(*surf_pc_out);
}

void OdomEstimationDmapping::addSurfCostFactor(const SurfElCloud& cloud,
                                               const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& map_in,
                                               ceres::Problem& problem,
                                               ceres::LossFunction *loss_function){
  int surf_num=0;
  Eigen::Isometry3d poseCurrent = EigenCombine(q_w_curr, t_w_curr);
  SurfElCloud cloudTransformed = cloud.Transform(poseCurrent);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudNormal = cloud.GetPointCloud();

  const double max_radius = 0.5*0.5;

  for (int i = 0 ; i < cloudNormal->size() ; i++)
  {
    const Eigen::Vector3d src_mean(cloudNormal->points[i].x, cloudNormal->points[i].y, cloudNormal->points[i].z);
    const Eigen::Vector3d src_normal(cloudNormal->points[i].normal_x, cloudNormal->points[i].normal_y, cloudNormal->points[i].normal_z);

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeSurfMap->nearestKSearch(cloudNormal->points[i], 1, pointSearchInd, pointSearchSqDis);
    for(int j = 0 ; j < pointSearchInd.size() ; i++){
      if(pointSearchSqDis[j] < max_radius){
        const Eigen::Vector3d target_mean(map_in->points[pointSearchInd[j]].x, map_in->points[pointSearchInd[j]].y, map_in->points[pointSearchInd[j]].z );
        const Eigen::Vector3d target_normal(map_in->points[pointSearchInd[j]].normal_x, map_in->points[pointSearchInd[j]].normal_y, map_in->points[pointSearchInd[j]].normal_z );
      }

    }

  }
}

void OdomEstimationDmapping::addPointsToMap(const SurfElCloud& cloud){
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr surf_in = cloud.GetPointCloud();
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr surf_transf(new pcl::PointCloud<pcl::PointXYZINormal>());
  Eigen::Affine3d odomaffine(odom);
  pcl::transformPointCloudWithNormals(*surf_in, *surf_transf, odomaffine);
  if(laserCloudSurfMap->empty()){
    *laserCloudSurfMap = *surf_transf;
  }else{ // do something smarter
    *laserCloudSurfMap += *surf_transf;
  }

  double x_min = +odom.translation().x()-100;
  double y_min = +odom.translation().y()-100;
  double z_min = +odom.translation().z()-100;
  double x_max = +odom.translation().x()+100;
  double y_max = +odom.translation().y()+100;
  double z_max = +odom.translation().z()+100;

  //ROS_INFO("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
  cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
  cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
  cropBoxFilter.setNegative(false);

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZINormal>());
  cropBoxFilter.setInputCloud(laserCloudSurfMap);
  cropBoxFilter.filter(*tmpSurf);
  downSizeFilterSurf.setInputCloud(tmpSurf);
  *laserCloudSurfMap = * tmpSurf;
}

void OdomEstimationDmapping::getMap(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& laserCloudMap){
  *laserCloudMap += *laserCloudSurfMap;
}

OdomEstimationDmapping::OdomEstimationDmapping(){}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr VelToIntensityCopy(const pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr VelCloud){
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr converted(new pcl::PointCloud<pcl::PointXYZINormal>());
  converted->resize(VelCloud->size());
  converted->header = VelCloud->header;
  for(int i = 0; i < converted->size() ; i++){
    converted->points[i].x = VelCloud->points[i].x; converted->points[i].y = VelCloud->points[i].y; converted->points[i].z = VelCloud->points[i].z; converted->points[i].intensity = VelCloud->points[i].intensity;
  }
  return converted;
}

bool OdomEstimationDmapping::KeyFrameUpdate(const SurfElCloud& cloud, const Eigen::Isometry3d& pose){

  const keyframe currentFrame{pose, cloud};
  static bool first = true;
  if(first){
    first = false;
    keyframes_.push_back(std::move(currentFrame));
    return true;
  }else{
    const Eigen::Isometry3d delta = keyframes_.back().pose.inverse() * currentFrame.pose;
    const double delta_movement = delta.translation().norm();
    const double delta_rot = Eigen::AngleAxisd(delta.linear()).angle();

    if(delta_movement > keyframe_min_transl_ || delta_rot > keyframe_min_rot_) {
      keyframes_.push_back(currentFrame);
      if(keyframes_.size() > keyframe_history_){
        keyframes_.erase(keyframes_.begin());
      }
      return true;
    }else{
      return false;
    }
  }
}


}
