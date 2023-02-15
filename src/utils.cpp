#include "utils.h"

void SavePosegraph(
    const std::string& dump_directory,
    const std::vector<Eigen::Affine3d>& poses,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds){

  std::cout << "\"FLOAM\" - Save posegraph to:\n" << dump_directory << std::endl << std::endl;

  boost::filesystem::create_directories(dump_directory);
  std::ofstream graph_ofs(dump_directory + "/graph.g2o");
  unsigned int count = 0;
  for(const auto& pose_affine : poses) {
    const Eigen::Matrix4d pose = pose_affine.matrix();
    const Eigen::Vector3d t = pose.block<3, 1>(0, 3);
    const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    //std::cout << "VERTEX_SE3:QUAT " << count << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    graph_ofs << "VERTEX_SE3:QUAT " << count++ << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  graph_ofs << "FIX 0" << "\n";

  //std::cout << "factors:" << poses.size() -1 << std::endl;
  const int last = poses.size() - 1;
  if(poses.size () <= 1){
    std::cerr << "cannot save a pose graph with only 1 vertex" << std::endl;
  }

  for(int i = 0 ; i < poses.size() - 1 ; i++) {
    const int key1 = i;
    const int key2 = i + 1;
    const Eigen::Affine3d poseCurrent = poses[key1];
    const Eigen::Affine3d poseNext    = poses[key2];
    const Eigen::Affine3d between_factor = poseCurrent.inverse()*poseNext;


    Eigen::Matrix4d relative = between_factor.matrix();
    Eigen::Vector3d t = relative.block<3, 1>(0, 3);
    Eigen::Quaterniond q(relative.block<3, 3>(0, 0));

    graph_ofs << "EDGE_SE3:QUAT " << key1 << " " << key2;
    graph_ofs << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
    //std::cout << "EDGE_SE3:QUAT " << key1 << " " << key2;
    //std::cout << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();

    Eigen::Matrix<double,6,1> variances;
    variances << 0.01, 0.01, 0.01, 0.001, 0.001, 0.001;
    Eigen::MatrixXd inf = variances.asDiagonal();

    for(int i = 0; i < inf.rows(); i++) {
      for(int j=i; j<inf.cols(); j++) {
        graph_ofs << " " << inf(i, j);
        //std::cout << " " << inf(i, j);
      }
    }
    graph_ofs << "\n";
  }

  graph_ofs.close();

  //std::cout << "Save clouds: " << clouds.size() << std::endl;
  for(int i = 0; i < clouds.size(); i++) {
    std::string keyframe_directory = (boost::format("%s/%06d") % dump_directory % i).str();
    //std::cout <<std::endl << "i: " << i << keyframe_directory  << std::endl;
    boost::filesystem::create_directories(keyframe_directory);
    // cloud = transformPointCloud(cloud, keyframe_poses[i]);
    pcl::io::savePCDFileBinary(keyframe_directory + "/cloud.pcd", *clouds[i]);

    ros::Time stamp(keyframe_stamps[i]);

    std::ofstream data_ofs(keyframe_directory + "/data");
    data_ofs << "stamp " << stamp.sec << " " << stamp.nsec << "\n";
    data_ofs << "estimate\n" << poses[i].matrix() << "\n";
    data_ofs << "odom\n" << poses[i].matrix() << "\n";
    data_ofs << "accum_distance -1" << "\n";
    data_ofs << "id " << i << "\n";
  }
}


void SaveOdom(
    const std::string& dump_directory,
    const std::vector<Eigen::Affine3d>& poses,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds){

  boost::filesystem::create_directories(dump_directory);

  std::cout << "\"FLOAM\" - Save odom to:\n" << dump_directory << std::endl << std::endl;
  //std::cout << "Save clouds: " << clouds.size() << std::endl;
  for(int i = 0; i < clouds.size(); i++) {

    ros::Time t(keyframe_stamps[i]);
    std::string filename = dump_directory + str( boost::format("/%lf_%lf") % t.sec % t.nsec);

    pcl::io::savePCDFileBinary( filename + ".pcd", *clouds[i]);

    std::ofstream data_ofs(filename + ".odom");
    Eigen::Matrix<double,4,4> mat = poses[i].matrix();
    data_ofs << mat(0,0) << " " << mat(0,1) << " " << mat(0,2) << " " << mat(0,3) << std::endl;
    data_ofs << mat(1,0) << " " << mat(1,1) << " " << mat(1,2) << " " << mat(1,3) << std::endl;
    data_ofs << mat(2,0) << " " << mat(2,1) << " " << mat(2,2) << " " << mat(2,3) << std::endl;
    data_ofs << mat(3,0) << " " << mat(3,1) << " " << mat(3,2) << " " << mat(3,3) << std::endl;
    data_ofs.close();
  }
}
sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in, const Eigen::Quaterniond& extQRPY){
  sensor_msgs::Imu imu_out = imu_in;
  Eigen::Matrix3d extRot(extQRPY.toRotationMatrix());
  // rotate acceleration
  Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
  acc = extRot * acc;
  imu_out.linear_acceleration.x = acc.x();
  imu_out.linear_acceleration.y = acc.y();
  imu_out.linear_acceleration.z = acc.z();
  // rotate gyroscope
  Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
  gyr = extRot * gyr;
  imu_out.angular_velocity.x = gyr.x();
  imu_out.angular_velocity.y = gyr.y();
  imu_out.angular_velocity.z = gyr.z();
  // rotate roll pitch yaw
  Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
  Eigen::Quaterniond q_final = q_from * extQRPY;
  imu_out.orientation.x = q_final.x();
  imu_out.orientation.y = q_final.y();
  imu_out.orientation.z = q_final.z();
  imu_out.orientation.w = q_final.w();

  if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
  {
    ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
    ros::shutdown();
  }

  return imu_out;
}



std::vector<int> NNSearchArray::findClosestElements(std::vector<double>& arr, int k, float max, float query) {

  auto itr_begin = arr.begin();
  auto itr_end = arr.end();

  std::vector<int> indicies;
  auto it = std::lower_bound(itr_begin, itr_end, query);

  if(it == itr_begin){
    for(auto itr = itr_begin ; itr != itr_begin+k ; itr++){
      indicies.push_back(std::distance(itr_begin,itr));
    }
    return indicies; // return vector<int>(itr_begin(), itr_begin()+k);
  }else if(it == itr_end){
    for(auto itr = itr_end - k ; itr != itr_end ; itr++){
      indicies.push_back(std::distance(itr_begin,itr));
    }
    return indicies; //return vector<int>(itr_end() - k, itr_end());

  }

  const int n = arr.size();
  int idx = it - itr_begin;
  int left, right;
  left = idx - k/2;
  right = left + k;

  // cout << left << ", " << right << " -> ";

  if(left < 0){
    left = 0;
    right = left +k;
  }

  if(right > n){
    right = n;
    left = right - k;
  }

  // cout << left << ", " << right << " -> ";

  /*
    since arr[idx] is just a surrogate of x,
    we need to examine the diff of elements with "x" again
    */

  /*
    while the diff of the element to the leftmost element "<=" that of the rightmost element,
    move the subarray left

    here the "<=" means we prefer smaller element
    */
  while(left > 0 && query - arr[left-1] <= arr[right-1] - query){
    right--;
    left--;
  }

  /*
    while the diff of rightmost element "<" that of the element to the leftmost element,
    move the subarray right
    */
  while(right < n && arr[(right-1)+1] - query  < query - arr[left]){
    right++;
    left++;
  }

  // cout << left << ", " << right << endl;
  //std::vector<int> indicies(itr_begin()+left , itr_begin()+right);
  //std::cout << "query: " << query << std::endl;
  for(auto itr = itr_begin + left  ; itr != itr_begin + right ; itr++){
    //cout << std::fabs(query - *itr) << ", ";
    indicies.push_back(std::distance(itr_begin,itr));
  }
  //std::cout << std::endl << std::endl;
  return indicies;
}

Eigen::Isometry3d vectorToAffine3d(double x, double y, double z, double ex, double ey, double ez) {
  return Eigen::Translation<double, 3>(x, y, z) *
      Eigen::AngleAxis<double>(ex, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(ey, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(ez, Eigen::Vector3d::UnitZ());
}


