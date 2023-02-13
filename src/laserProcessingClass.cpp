// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "laserProcessingClass.h"

void LaserProcessingClass::init(lidar::Lidar lidar_param_in){

  lidar_param = lidar_param_in;

}
void LaserProcessingClass::RingExtractionVelodyne(const pcl::PointCloud<vel_point::PointXYZIRT>::Ptr& pc_in, std::vector<pcl::PointCloud<vel_point::PointXYZIRT>::Ptr>& laserCloudScans, std::vector<std::vector<double> >& range_image){
  const int N_SCANS = lidar_param.num_lines;
  range_image.resize(N_SCANS);
  for(int i=0;i<N_SCANS;i++){
    laserCloudScans.push_back(pcl::PointCloud<vel_point::PointXYZIRT>::Ptr(new pcl::PointCloud<vel_point::PointXYZIRT>()));
  }
  for (int i = 0; i < (int) pc_in->points.size(); i++){
    const int scanID = pc_in->points[i].ring;
    double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y + pc_in->points[i].z * pc_in->points[i].z );
    /*if(distance<lidar_param.min_distance || distance>lidar_param.max_distance)
      continue;*/
    //std::cout << scanID << ", ";
    vel_point::PointXYZIRT p_tmp;
    p_tmp.x = pc_in->points[i].x; p_tmp.y = pc_in->points[i].y; p_tmp.z = pc_in->points[i].z; p_tmp.intensity = pc_in->points[i].intensity; p_tmp.ring = pc_in->points[i].ring;  p_tmp.time = pc_in->points[i].time;
    laserCloudScans[scanID]->push_back(p_tmp);
    range_image[scanID].push_back(distance);
  }
}

void LaserProcessingClass::RingExtraction(const pcl::PointCloud<vel_point::PointXYZIRT>::Ptr& pc_in, std::vector<pcl::PointCloud<vel_point::PointXYZIRT>::Ptr> laserCloudScans){
  int N_SCANS = lidar_param.num_lines;
  for (int i = 0; i < (int) pc_in->points.size(); i++)
  {
    int scanID=0;
    double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y);
    if(distance<lidar_param.min_distance || distance>lidar_param.max_distance)
      continue;
    double angle = atan(pc_in->points[i].z / distance) * 180 / M_PI;

    if (N_SCANS == 16)
    {
      scanID = int((angle + 15) / 2 + 0.5);
      if (scanID > (N_SCANS - 1) || scanID < 0)
      {
        continue;
      }
    }
    else if (N_SCANS == 32)
    {
      scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
      if (scanID > (N_SCANS - 1) || scanID < 0)
      {
        continue;
      }
    }
    else if (N_SCANS == 64)
    {
      if (angle >= -8.83)
        scanID = int((2 - angle) * 3.0 + 0.5);
      else
        scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

      if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0)
      {
        continue;
      }
    }
    else
    {
      printf("wrong scan number\n");
    }
    vel_point::PointXYZIRT p_tmp;
    p_tmp.x = pc_in->points[i].x; p_tmp.y = pc_in->points[i].y; p_tmp.z = pc_in->points[i].z; p_tmp.intensity = pc_in->points[i].intensity;
    laserCloudScans[scanID]->push_back(p_tmp);
  }
}

void LaserProcessingClass::FloamFeatures(std::vector<pcl::PointCloud<vel_point::PointXYZIRT>::Ptr> laserCloudScans, VelCurve::Ptr& pc_out_edge, VelCurve::Ptr& pc_out_surf, VelCurve::Ptr& pc_out_less_flat, VelCurve::Ptr& pc_out_less_edge){

  VelCurve feature_cloud;
  const int N_SCANS = lidar_param.num_lines;
  for(int i = 0; i < N_SCANS; i++){
    if(laserCloudScans[i]->points.size()<131){
      continue;
    }

    std::vector<Double2d> cloudCurvature;
    int total_points = laserCloudScans[i]->points.size()-10;
    for(int j = 5; j < (int)laserCloudScans[i]->points.size() - 5; j++){
      double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x - 10 * laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;
      double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y - 10 * laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;
      double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z - 10 * laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;
      Double2d distance(j,diffX * diffX + diffY * diffY + diffZ * diffZ);
      cloudCurvature.push_back(distance);
      vel_point::PointXYZIRTC pnt_curve;
      pnt_curve.x =  laserCloudScans[i]->points[j].x; pnt_curve.y =  laserCloudScans[i]->points[j].y; pnt_curve.z =  laserCloudScans[i]->points[j].z;
      pnt_curve.ring =  laserCloudScans[i]->points[j].ring; pnt_curve.curvature =  distance.value; pnt_curve.intensity = laserCloudScans[i]->points[j].intensity;
      feature_cloud.push_back(std::move(pnt_curve));
    }
    for(int j=0;j<6;j++){ // this is not a good idea for accuracy!
      int sector_length = (int)(total_points/6);
      int sector_start = sector_length *j;
      int sector_end = sector_length *(j+1)-1;
      if (j==5){
        sector_end = total_points - 1;
      }
      std::vector<Double2d> subCloudCurvature(cloudCurvature.begin()+sector_start,cloudCurvature.begin()+sector_end);
      featureExtractionFromSector(laserCloudScans[i],subCloudCurvature, pc_out_edge, pc_out_surf, pc_out_less_flat, pc_out_less_edge);
    }

  }// Per ring
  PublishCloud("feature_extract", feature_cloud, "base_link", ros::Time::now());
}
void LaserProcessingClass::featureExtraction(const pcl::PointCloud<vel_point::PointXYZIRT>::Ptr& pc_in, VelCurve::Ptr& pc_out_edge, VelCurve::Ptr& pc_out_surf, VelCurve::Ptr& pc_out_less_flat, VelCurve::Ptr& pc_out_less_edge){

  cout << pc_in->size() << endl;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pc_in, indices);

  std::vector<pcl::PointCloud<vel_point::PointXYZIRT>::Ptr> laserCloudScans;
  std::vector<std::vector<double> > range_image;
  RingExtractionVelodyne(pc_in, laserCloudScans, range_image);
  FloamFeatures(laserCloudScans, pc_out_edge, pc_out_surf, pc_out_less_flat, pc_out_less_edge);
}


void LaserProcessingClass::featureExtractionFromSector(const pcl::PointCloud<vel_point::PointXYZIRT>::Ptr& pc_in,
                                                       std::vector<Double2d>& cloudCurvature,
                                                       VelCurve::Ptr& pc_out_edge,
                                                       VelCurve::Ptr& pc_out_surf,
                                                       VelCurve::Ptr& pc_out_less_flat,
                                                       VelCurve::Ptr& pc_out_less_edge){

  std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double2d & a, const Double2d & b)
  {
    return a.value < b.value;
  });


  int largestPickedNum = 0;
  std::vector<int> picked_points;
  int point_info_count =0;
  for (int i = cloudCurvature.size()-1; i >= 0; i--)
  {
    const int ind = cloudCurvature[i].id;
    const int curv = cloudCurvature[i].value;
    if(std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
      if(cloudCurvature[i].value <= 0.1){
        break;
      }

      largestPickedNum++;
      picked_points.push_back(ind);

      if (curv > 50 && largestPickedNum <= 20){
        pc_out_edge->push_back(ToCurvature(pc_in->points[ind], curv));
        //pc_out_edge->push_back(pc_in->points[ind]);
        point_info_count++;
      }else{
        break;
      }

      for(int k=1;k<=5;k++){
        double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
        double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
        double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
          break;
        }
        picked_points.push_back(ind+k);
      }
      for(int k=-1;k>=-5;k--){
        double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
        double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
        double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
          break;
        }
        picked_points.push_back(ind+k);
      }

    }
  }

  // split remaining points
  for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
  {
    const int ind = cloudCurvature[i].id;
    const float curv = cloudCurvature[i].value;
    if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end())
    {
      if(curv < 50 ){
        pc_out_surf->push_back(ToCurvature(pc_in->points[ind], curv));
      }else{
        pc_out_less_edge->push_back(ToCurvature(pc_in->points[ind], curv));
      }
    }
  }



}
LaserProcessingClass::LaserProcessingClass(){

}

Double2d::Double2d(int id_in, double value_in){
  id = id_in;
  value = value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in){
  layer = layer_in;
  time = time_in;
};
