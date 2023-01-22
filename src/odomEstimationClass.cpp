// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"



SurfelExtraction::SurfelExtraction(VelCurve::Ptr& surf_in, lidar::Lidar& lidar_par) : lidar_par_(lidar_par){
  surf_in_ = surf_in;
  Initialize();
}

void SurfelExtraction::Initialize(){


  std::vector<VelCurve::Ptr> clouds(lidar_par_.num_lines);
  for(auto && cloud : clouds){ // initiaize
    cloud = VelCurve::Ptr(new VelCurve());
  }

  for(auto&& pnt : surf_in_->points){
    clouds[pnt.ring]->push_back(pnt);
  }

  for(auto && cloud : clouds){
    SortTime(cloud);
  }
}
void SurfelExtraction::Extract(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& normals){
  normals =  pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointXYZINormal p0;

  /*for(auto && pnt : surf_in_->points){ // only for test
    //p0.x = 0.1*i; p0.y = 0.1*i; p0.z = 0.1*i; p0.curvature = 2; p0.intensity = 50; p0.normal_x = 0.1; p0.normal_y = 0.1; p0.normal_z = 0.9;
    //p0.x = pnt.x; p0.y = pnt.y; p0.z = pnt.z; p0.normal_x = 1; p0.normal_y = 1; p0.normal_z = 1;
    //normals->push_back(p0);
  }*/
  pcl::PointXYZINormal pNormalEst;
  for(auto && pnt : surf_in_->points){
    if(EstimateNormal(pnt, pNormalEst)){
      normals->push_back(pNormalEst);
    }
  }

}
bool SurfelExtraction::GetNeighbours(const vel_point::PointXYZIRTC& pnt, Eigen::MatrixXd& neighbours){
  const int ring = pnt.ring;
  const double time = pnt.time;
  std::vector<int> search;
  // not last ring
  if(ring < lidar_par_.num_lines - 1){
  }
  // not first ring
  if(ring > 0 ){

  }


  return true;


}
bool SurfelExtraction::EstimateNormal(const vel_point::PointXYZIRTC& pnt,pcl::PointXYZINormal& pNormalEst){

  Eigen::MatrixXd X ; //neighbours
  const bool statusOK = GetNeighbours(pnt, X); // 3 x Nsamples
  if(!statusOK){
    return false;
  }
  const int Nsamples = X.rows();
  Eigen::Vector3d mean(0,0,0);  // 3 x 1

  for(Eigen::Index i=0 ; i<Nsamples ; i++)
    mean.block<3,1>(0,0) += X.block<1,3>(i,0).transpose(); // compute sum

  for(Eigen::Index i=0 ; i<Nsamples ; i++) // subtract mean
    X.block<1,3>(i,0) = X.block<1,3>(i,0) - mean.block<3,1>(0,0).transpose();

  const Eigen::Matrix3d cov = X.transpose()*X;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);

  const double l1 = es.eigenvalues()[0];  // l1 < l2 < l3
  const double l2 = es.eigenvalues()[1];
  const double l3 = es.eigenvalues()[2];
  const double planarity = 1 - (l1 + l2)/ (l1 + l2 + l3); // this should be it when l1 & l2 -> 0  planarity -> 1 if l3 >> l1+l2
  const Eigen::Vector3d normal = planarity*es.eigenvectors().col(0);
  pNormalEst.normal_x = normal(0); pNormalEst.normal_y = normal(1); pNormalEst.normal_z = normal(2); // assign normals
  pNormalEst.x = pnt.x; pNormalEst.y = pnt.y; pNormalEst.z = pnt.z; pNormalEst.intensity = pnt.intensity; //copy other fields
  return true;
}







void OdomEstimationClass::init(lidar::Lidar lidar_param, double map_resolution, const std::string& loss_function){
  //init local map
  laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  //downsampling size
  downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
  downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

  //kd-tree
  kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

  odom = Eigen::Isometry3d::Identity();
  last_odom = Eigen::Isometry3d::Identity();
  optimization_count=2;
  loss_function_ = boost::algorithm::to_lower_copy(loss_function);
  std::cout << "Use loss function: " << loss_function_ << std::endl;
  lidar_param_ = lidar_param;
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in){
  *laserCloudCornerMap += *edge_in;
  *laserCloudSurfMap += *surf_in;
  optimization_count=12;
}

void OdomEstimationClass::UpdatePointsToMapSelector(pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr& edge_in, pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr& surf_in, bool deskew){
  ros::Time t0 = ros::Time::now();
  if(!deskew){
    updatePointsToMap(edge_in, surf_in,  UpdateType::VANILLA);
    ros::Time t1 = ros::Time::now();
  }else{
    updatePointsToMap(edge_in, edge_in, UpdateType::INITIAL_ITERATION);
    Eigen::Vector3d velocity = GetVelocity();
    dmapping::CompensateVelocity(edge_in, velocity);
    dmapping::CompensateVelocity(surf_in, velocity);

    ros::Time t1 = ros::Time::now();
    updatePointsToMap(edge_in, surf_in, UpdateType::REFINEMENT_AND_UPDATE);
    ros::Time t2 = ros::Time::now();
    //cout << "Registration time: " << t2-t0<<", first iteration: " << t1-t0 <<", second iteration: " << t2-t1 <<endl;
  }
}

void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr& edge_in, const pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr& surf_in, const UpdateType update_type){
  pcl::PointCloud<pcl::PointXYZI>::Ptr edge_in_XYZI = VelToIntensityCopy(edge_in);
  pcl::PointCloud<pcl::PointXYZI>::Ptr surf_in_XYZI = VelToIntensityCopy(surf_in);
  updatePointsToMap(edge_in_XYZI, surf_in_XYZI, update_type);
}
void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in, const UpdateType update_type){

  if(optimization_count>2)
    optimization_count--;

  Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
  if( update_type == UpdateType::VANILLA || UpdateType::INITIAL_ITERATION ){
    last_odom = odom;
    odom = odom_prediction;
  }else if(update_type == UpdateType::REFINEMENT_AND_UPDATE){ // last_odom already correct, odom already set to registration before unwarp

  }

  q_w_curr = Eigen::Quaterniond(odom.rotation());
  t_w_curr = odom.translation();

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
  downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);
  //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
  if(laserCloudCornerMap->points.size()>10 && laserCloudSurfMap->points.size()>50){
    kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
    kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

    for (int iterCount = 0; iterCount < optimization_count; iterCount++){

      ceres::LossFunction *loss_function = nullptr;
      if(loss_function_ == "huber"){
        //  std::cout << "huber" << std::endl;
        loss_function = new ceres::HuberLoss(0.1);
      }
      else{
        //std::cout << "Cauchy" << std::endl;
        new ceres::CauchyLoss(0.2); // why 0.2? https://arxiv.org/pdf/2211.02445.pdf Fig.12 //daniel
      }
      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options);

      problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());

      addEdgeCostFactor(downsampledEdgeCloud,laserCloudCornerMap,problem,loss_function);
      addSurfCostFactor(downsampledSurfCloud,laserCloudSurfMap,problem,loss_function);

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
  if( update_type == UpdateType::VANILLA || UpdateType::REFINEMENT_AND_UPDATE){
    addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud);
  }

}

void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
  po->intensity = pi->intensity;
  //po->intensity = 1.0;
}

void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out){
  downSizeFilterEdge.setInputCloud(edge_pc_in);
  downSizeFilterEdge.filter(*edge_pc_out);
  downSizeFilterSurf.setInputCloud(surf_pc_in);
  downSizeFilterSurf.filter(*surf_pc_out);
}

void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
  int corner_num=0;
  for (int i = 0; i < (int)pc_in->points.size(); i++)
  {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&(pc_in->points[i]), &point_temp);

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
    if (pointSearchSqDis[4] < 1.0)
    {
      std::vector<Eigen::Vector3d> nearCorners;
      Eigen::Vector3d center(0, 0, 0);
      for (int j = 0; j < 5; j++)
      {
        Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
            map_in->points[pointSearchInd[j]].y,
            map_in->points[pointSearchInd[j]].z);
        center = center + tmp;
        nearCorners.push_back(tmp);
      }
      center = center / 5.0;

      Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
      for (int j = 0; j < 5; j++)
      {
        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
      Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
      {
        Eigen::Vector3d point_on_line = center;
        Eigen::Vector3d point_a, point_b;
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;

        ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
        problem.AddResidualBlock(cost_function, loss_function, parameters);
        corner_num++;
      }
    }
  }
  if(corner_num<20){
    printf("not enough correct points");
  }

}

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
  int surf_num=0;
  for (int i = 0; i < (int)pc_in->points.size(); i++)
  {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&(pc_in->points[i]), &point_temp);
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
    if (pointSearchSqDis[4] < 1.0)
    {

      for (int j = 0; j < 5; j++)
      {
        matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
        matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
        matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
      }
      // find the norm of plane
      Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
      double negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();

      bool planeValid = true;
      for (int j = 0; j < 5; j++)
      {
        // if OX * n > 0.2, then plane is not fit well
        if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                 norm(1) * map_in->points[pointSearchInd[j]].y +
                 norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
        {
          planeValid = false;
          break;
        }
      }
      Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
      if (planeValid)
      {
        ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
        problem.AddResidualBlock(cost_function, loss_function, parameters);

        surf_num++;
      }
    }

  }
  if(surf_num<20){
    printf("not enough correct points");
  }

}

void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud){

  for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
  {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);
    laserCloudCornerMap->push_back(point_temp);
  }

  for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
  {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);
    laserCloudSurfMap->push_back(point_temp);
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

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZI>());
  cropBoxFilter.setInputCloud(laserCloudSurfMap);
  cropBoxFilter.filter(*tmpSurf);
  cropBoxFilter.setInputCloud(laserCloudCornerMap);
  cropBoxFilter.filter(*tmpCorner);

  downSizeFilterSurf.setInputCloud(tmpSurf);
  downSizeFilterSurf.filter(*laserCloudSurfMap);
  downSizeFilterEdge.setInputCloud(tmpCorner);
  downSizeFilterEdge.filter(*laserCloudCornerMap);

}

void OdomEstimationClass::getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap){

  *laserCloudMap += *laserCloudSurfMap;
  *laserCloudMap += *laserCloudCornerMap;
}

OdomEstimationClass::OdomEstimationClass(){

}

pcl::PointCloud<pcl::PointXYZI>::Ptr VelToIntensityCopy(const pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr VelCloud){

  pcl::PointCloud<pcl::PointXYZI>::Ptr converted(new pcl::PointCloud<pcl::PointXYZI>());
  converted->resize(VelCloud->size());
  converted->header = VelCloud->header;
  for(int i = 0; i < converted->size() ; i++)
  {
    converted->points[i].x = VelCloud->points[i].x; converted->points[i].y = VelCloud->points[i].y; converted->points[i].z = VelCloud->points[i].z; converted->points[i].intensity = VelCloud->points[i].intensity;
  }
  return converted;
}
