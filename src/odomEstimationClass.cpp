// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"

NormalCloud::Ptr SurfElCloud::GetPointCloud(){
  NormalCloud::Ptr output(new NormalCloud());
  for(auto && surfEl : cloud){
    pcl::PointXYZINormal pnt;
    pnt.x = surfEl.centerPoint(0); pnt.y = surfEl.centerPoint(1); pnt.z = surfEl.centerPoint(2);
    pnt.normal_x = surfEl.normal(0); pnt.normal_y = surfEl.normal(1);  pnt.normal_z = surfEl.normal(2);
    pnt.intensity = surfEl.planarity;
    output->push_back(std::move(pnt));
  }
  return output;
}
SurfelExtraction::SurfelExtraction(VelCurve::Ptr& surf_in, lidar::Lidar& lidar_par) : lidar_par_(lidar_par){
  surf_in_ = surf_in;
  Initialize();
}

void SurfelExtraction::Initialize(){

  ringClouds_.resize(lidar_par_.num_lines);
  times_.resize(lidar_par_.num_lines);
  for(int i = 0 ; i < ringClouds_.size() ; i++){ // initiaize clouds
    ringClouds_[i] = VelCurve::Ptr(new VelCurve());
  }
  for(auto&& pnt : surf_in_->points){ // Fill up
    ringClouds_[pnt.ring]->push_back(pnt);
  }
  for(auto && cloud : ringClouds_){ // Sort
    SortTime(cloud);
  }
  for(auto&& cloud : ringClouds_){ // And create the same structure for doubles
    for(auto && pnt : cloud->points){
      times_[pnt.ring].push_back(pnt.time); // Please preallocate next time daniel!!!!
    }
  }

}
void SurfelExtraction::Extract(SurfElCloud& surfelCloud){

  /*for(auto && pnt : surf_in_->points){ // only for test
    //p0.x = 0.1*i; p0.y = 0.1*i; p0.z = 0.1*i; p0.curvature = 2; p0.intensity = 50; p0.normal_x = 0.1; p0.normal_y = 0.1; p0.normal_z = 0.9;
    //p0.x = pnt.x; p0.y = pnt.y; p0.z = pnt.z; p0.normal_x = 1; p0.normal_y = 1; p0.normal_z = 1;
    //normals->push_back(p0);
  }*/


 for(auto && pnt : surf_in_->points){
    SurfelPointInfo pntSurfEl;
    if(EstimateNormal(pnt, pntSurfEl)){
      surfelCloud.cloud.push_back(std::move(pntSurfEl));
    }
  }
 NormalCloud::Ptr normals = surfelCloud.GetPointCloud();

}
void SurfelExtraction::LineNNSearch( const int ring, const double query, int &row, Eigen::MatrixXd& neighbours){

  NNSearchArray NNSearch;
  const int scans_par_line = 1800;
  float hor_time_res = 6*lidar_par_.scan_period/scans_par_line; // max 6 points from center
  //cout << "max tdiff - not implemented yet: " << scans_par_line << std::endl;
  std::vector<int> indicies = NNSearch.findClosestElements(times_[ring], 5, hor_time_res, query);
  //cout << "nearby: " << indicies.size() << endl;
  if(indicies.size() > 0){
    //const int first_row = neighbours.rows();
    //cout << "first:" << first_row << endl;
    //neighbours.resize(first_row + indicies.size(), 3);
    //cout << "resized:" << endl;
    //neighbours = Eigen::MatrixXd(indicies.size(),3);
    for(int first_row = row; row< first_row + indicies.size() ; row++){ // i is row in matrix
      //cout << i - first_row << endl;
      const int idx = indicies[row - first_row]; // zero index
      //cout << "idx" << idx << endl;
      const Eigen::Vector3d  pntNear(ringClouds_[ring]->points[idx].x, ringClouds_[ring]->points[idx].y, ringClouds_[ring]->points[idx].z);
      neighbours.block<1,3>(row,0) = pntNear;
      //cout << "neigbour " << neighbours.block<1,3>(i,0).transpose() << endl;
    }
  }
}
bool SurfelExtraction::GetNeighbours(const vel_point::PointXYZIRTC& pnt, Eigen::MatrixXd& neighbours){
  const int ring = pnt.ring;
  const double time = pnt.time;
  std::vector<int> search;
  // not last ring
  neighbours = Eigen::MatrixXd(15,3);
  int first = 0;

  ///cout << "ring: "<< ring << endl;
  //cout << "dim: " << neighbours.rows() << " x " << neighbours.cols() << endl;
  LineNNSearch(ring, time,first, neighbours);
  //cout << "dim: " << neighbours.rows() << " x " << neighbours.cols() << endl;
  if(ring < lidar_par_.num_lines - 1){
    LineNNSearch(ring+1, time, first, neighbours);
  }
  //cout << "dim: " << neighbours.rows() << " x " << neighbours.cols() << endl;
  // not first ring
  if(ring > 0 ){
    LineNNSearch(ring-1, time, first, neighbours);
  }
  neighbours.conservativeResize(first,3);
  //cout << "dim: " << neighbours.rows() << " x " << neighbours.cols() << endl;

  return true;


}
bool SurfelExtraction::EstimateNormal(const vel_point::PointXYZIRTC& pnt, SurfelPointInfo& surfel){

  //cout << "EstimateNormal" << endl;
  Eigen::MatrixXd X; //neighbours
  const bool statusOK = GetNeighbours(pnt, X); // 3 x Nsamples
  if(!statusOK){
    return false;
  }
  /*pcl::PointCloud<pcl::PointXYZ> cloud, cloud_pnt;

  for(int i = 0 ; i <X.rows() ; i++){
    pcl::PointXYZ p(X(i,0), X(i,1), X(i,2));
    cloud.push_back(p);
  }
  cout << X << endl;
  cout <<"rows: " <<  X.rows() << endl;
  pcl::PointXYZ pnt_xyz(pnt.x, pnt.y, pnt.z);
  cloud_pnt.push_back(pnt_xyz);
  PublishCloud("surf", *surf_in_, "base_link", ros::Time::now() );
  PublishCloud("center", cloud_pnt, "base_link", ros::Time::now() );
  PublishCloud("neighbours", cloud, "base_link", ros::Time::now() );
  */

  //PublishCloud(const std::string& topic, Cloud& cloud, const std::string& frame_id, const ros::Time& t);
  const int Nsamples = X.rows();
  Eigen::Matrix<double,1,3> mean(0,0,0);  // 3 x 1

  for(Eigen::Index i=0 ; i<Nsamples ; i++)
    mean += X.block<1,3>(i,0); // compute sum
  mean/=Nsamples;

  for(Eigen::Index i=0 ; i<Nsamples ; i++) // subtract mean
    X.block<1,3>(i,0) = X.block<1,3>(i,0) - mean;

  const Eigen::Matrix3d cov = 1.0/(Nsamples - 1.0)*X.transpose()*X;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);


  const double l1 = std::sqrt(es.eigenvalues()[0]);  // l1 < l2 < l3
  const double l2 = std::sqrt(es.eigenvalues()[1]);
  const double l3 = std::sqrt(es.eigenvalues()[2]);
  const double planarity = 1 - (l1 + l2)/ (l1 + l2 + l3); // this should be it when l1 -> 0  & l2/l3 is high  planarity -> 1 if l3 >> l1+l2

  Eigen::Vector3d normal = es.eigenvectors().col(0);

  Eigen::Matrix <double, 3, 1> vp (-pnt.x, -pnt.y, -pnt.z);
  if(vp.dot(normal)> 0)
    normal *=-1;

  surfel.centerPoint = Eigen::Vector3d(pnt.x, pnt.y, pnt.z);
  surfel.mean = mean;
  surfel.l3 = l3;
  surfel.cov = cov;
  surfel.nSamples = Nsamples;
  surfel.planarity = 1 - (l1 + l2)/ (l1 + l2 + l3); // this should be it when l1 -> 0  & l2/l3 is high  planarity -> 1 if l3 >> l1+l2
  surfel.normal = normal;
  surfel.entropy = 0.5*log(1 + 2*M_PI*M_E*cov.determinant());
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

    //cout << "initial" << endl;

    updatePointsToMap(edge_in, edge_in, UpdateType::INITIAL_ITERATION);
    Eigen::Vector3d velocity = GetVelocity();
    dmapping::CompensateVelocity(edge_in, velocity);
    dmapping::CompensateVelocity(surf_in, velocity);

    ros::Time t1 = ros::Time::now();

    cout << "final" << endl;
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
  if( update_type == UpdateType::VANILLA || update_type == UpdateType::REFINEMENT_AND_UPDATE){
    const bool UpdateMap = KeyFrameUpdate(downsampledSurfCloud, downsampledEdgeCloud, odom);
    cout << "update?" << std::boolalpha << UpdateMap << endl;
    if(UpdateMap){
      addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud);
    }
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

bool OdomEstimationClass::KeyFrameUpdate(pcl::PointCloud<pcl::PointXYZI>::Ptr surf_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud, const Eigen::Isometry3d& pose){ // determines if the current pose is a new keyframe

  const keyframe currentFrame{pose, edge_cloud, surf_cloud};
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


