#include "dataHandler.h"

/* A steam of time stamped data  for lookup*/
namespace dmapping {


Eigen::Quaterniond Imu2Orientation(const sensor_msgs::Imu& data){
  return Eigen::Quaterniond(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z);
}

Eigen::Vector3d Imu2AngularVelocity(const sensor_msgs::Imu& data){
  return Eigen::Vector3d(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z);
}

Eigen::Vector3d Imu2LinearAcceleration(const sensor_msgs::Imu& data){
  return Eigen::Vector3d(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z);
}

bool compare (const stampedImu i, const stampedImu& j)
{
  return (i.first < j.first);
}

void ImuHandler::AddMsg(sensor_msgs::Imu::ConstPtr msg){
  sensor_msgs::Imu imuMsg = *msg;
  if ( data_.empty()){
    Add(imuMsg);
    return;
  }
  const double tdiff = msg->header.stamp.toSec() - data_.back().first;
  if( tdiff > 0.00001 ){
    Add(imuMsg);
  }
  else{
    //count_invalid++;
    //ROS_INFO_STREAM_THROTTLE(1, "dublicated time stamp, " << count_invalid);
  }
  //cout << "dublicated time stamp, " << count_invalid << endl;;
  //cout << "valid      time stamp, " << count_valid << endl;;
}


void ImuHandler::Add(const sensor_msgs::Imu& data){
  const double tStamp = data.header.stamp.toSec();;
  //tf::quaternionMsgToEigen(data.orientation, orient);
  data_.push_back(std::make_pair(tStamp, data));
}
sensor_msgs::Imu Interpolate(const double tSlerp, const sensor_msgs::Imu& data1, const sensor_msgs::Imu& data2){
  return data1;
}
bool ImuHandler::Get(const double& tStamp, sensor_msgs::Imu& data)const {
  auto first = data_.begin();
  auto last = data_.end();
  stampedImu search = std::make_pair(tStamp,sensor_msgs::Imu());
  auto itr_after = std::lower_bound(first, last, search , compare);
  auto itr_before = std::prev(itr_after, 1);
  if(itr_after != last && itr_after != first && itr_before != first){
    //data = first->second;
    //cout << "elements: " << data_.size() << endl;
    //cout << "search: " << GetRelTime(tStamp) << ", before: "<< GetRelTime(itr_before->first) <<", t diff" <<  itr_before->first - tStamp<<", idx: " << std::distance(data_.begin(), itr_before) << endl;
    //cout << "search: " << tStamp << ", next  : "<< itr_after->first <<", t diff" << itr_after->first - tStamp <<", idx: " << std::distance(data_.begin(), itr_after) << endl;
    const double tSlerp = (tStamp - itr_before->first )/(itr_after->first - itr_before->first);
    data = Interpolate(tSlerp, itr_before->second, itr_after->second);
    //cout << tSlerp << endl;
    //data = itr_before->second.slerp(tSlerp, itr_after->second);
    return true;
  }
  else
    return false;
}
sensor_msgs::Imu ImuHandler::Get(const double& tStamp) const{
  sensor_msgs::Imu data;
  Get(tStamp, data);
  return data;
}
bool ImuHandler::TimeContained(const double t)const{
  if(!data_.empty() && t >= data_.front().first && t <= data_.back().first)
    return true;
  else
    return false;
}

void CompensateVelocity(pcl::PointCloud<PointType>::Ptr input, const Eigen::Vector3d& velocity){

  for(auto && pnt : input->points){
    const double tPoint = pnt.time;
    const Eigen::Vector3d pntPosition(pnt.x, pnt.y, pnt.z);
    const Eigen::Vector3d pntError = velocity*tPoint; // Compute the error from movement and scan rate
    const Eigen::Vector3d pntCompensated = pntPosition + pntError; // remove the error
    pnt.x = pntCompensated(0); pnt.y = pntCompensated(1); pnt.z = pntCompensated(2);
  }

}

bool Compensate(pcl::PointCloud<PointType>&input, pcl::PointCloud<PointType>& compensated, ImuHandler& handler, Eigen::Quaterniond& extrinsics){

  compensated.resize(input.size());
  const double tScan = pcl_conversions::fromPCL(input.header.stamp).toSec();
  const double t0 = input.points.front().time + tScan;
  const double t1 = input.points.back().time + tScan;
  if(!handler.TimeContained(t0) || !handler.TimeContained(t1) ){
    std::cout << "no imu data" << std::endl;
    return false;
  }

  //std::cout << "begin: " <<  t0 << ", end: " << t1 << ", scan:" << tScan << std::endl;
  const Eigen::Quaterniond qInit(Imu2Orientation(handler.Get(tScan))*extrinsics);
  const Eigen::Quaterniond qInitInv = qInit.inverse();

  //std::cout << "qInit" << qInit.matrix() << std::endl;
  ros::Time tr0 = ros::Time::now();
  for(int i = 0 ; i <input.points.size() ; i++){
    const double timeCurrent = tScan + input.points[i].time;
    const Eigen::Quaterniond qNow = Imu2Orientation(handler.Get(timeCurrent))*extrinsics;
    const Eigen::Quaterniond qDiff = qInitInv*qNow;
    const Eigen::Vector3d pTransformed = qDiff*Eigen::Vector3d(input.points[i].x, input.points[i].y, input.points[i].z);
    compensated.points[i].x = pTransformed(0); compensated.points[i].y = pTransformed(1); compensated.points[i].z = pTransformed(2);
    compensated.points[i].ring = input.points[i].ring; compensated.points[i].time = input.points[i].time; compensated.points[i].intensity = input.points[i].intensity;
  }
  ros::Time tr1 = ros::Time::now();
  //std::cout << "time elapse: " << tr1-tr0 << std::endl;
  return true;

}




}

