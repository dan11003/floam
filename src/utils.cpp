#include "utils.h"

void SavePosegraph(
    const std::string& dump_directory,
    const std::vector<Eigen::Affine3d>& poses,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds){

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

  std::cout << "factors:" << poses.size() -1 << std::endl;
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
        std::cout << " " << inf(i, j);
      }
    }
    graph_ofs << "\n";
  }

  graph_ofs.close();

  std::cout << "Save clouds: " << clouds.size() << std::endl;
  for(int i = 0; i < clouds.size(); i++) {
    std::string keyframe_directory = (boost::format("%s/%06d") % dump_directory % i).str();
    std::cout <<std::endl << "i: " << i << keyframe_directory  << std::endl;
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
