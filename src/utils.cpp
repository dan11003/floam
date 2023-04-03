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


/*std::vector<int> NNSearchArray::findClosestElements(std::vector<double>& arr, int k, float max, float query) {

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

  while(left > 0 && query - arr[left-1] <= arr[right-1] - query){
    right--;
    left--;
  }

  while(right < n && arr[(right-1)+1] - query  < query - arr[left]){
    right++;
    left++;
  }

  for(auto itr = itr_begin + left  ; itr != itr_begin + right ; itr++){
    indicies.push_back(std::distance(itr_begin,itr));
  }
  //std::cout << std::endl << std::endl;
  return indicies;
}*/



/*Eigen::Isometry3d EigenCombine(const Eigen::Quaterniond& q, const Eigen::Vector3d& transl){
  Eigen::Matrix4d prediction = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d pred_mat;
  pred_mat.setIdentity();
  pred_mat.block<3,3>(0,0) = q.toRotationMatrix();
  pred_mat.block<3,1>(0,3) = transl;
  return Eigen::Isometry3d(pred_mat);
}*/


