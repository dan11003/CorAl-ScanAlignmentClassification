#include "alignment_checker/DataHandler.h"
namespace CorAlignment {


FiledataHandler::FiledataHandler(const std::string& directory, const std::string& poses, const std::string& data_prefix, int index_first_scan) {

}

std::shared_ptr<PoseScan> FiledataHandler::Next(){

}


RadarRosbagHandler::RadarRosbagHandler(const std::string& rosbag_path, const PoseScan::Parameters& scanPars, const std::string& radar_topic, const std::string& gt_topic): scanPars_(scanPars){

  bag_.open(rosbag_path, rosbag::bagmode::Read);
  view_= std::make_unique<rosbag::View>(bag_, rosbag::TopicQuery({radar_topic,gt_topic}));
  pub_image = nh_.advertise<sensor_msgs::Image>("/Navtech/Polar", 1000);
  pub_odom = nh_.advertise<nav_msgs::Odometry>("/gt", 1000);

  m_ = view_->begin();

}
void RadarRosbagHandler::UnpackImage(sensor_msgs::ImageConstPtr& image_msg){

  cv_bridge::CvImagePtr cv_polar_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_8UC1);
  assert(cv_polar_image != NULL);
  cv_polar_image->header.stamp =  image_msg->header.stamp;
  radar_stream_.push_back(cv_polar_image);
  cout<<"image timestamp: "<<cv_polar_image->header.stamp.toSec()<<", size: "<<cv_polar_image->image.cols<<", "<<cv_polar_image->image.rows<<endl;
  pub_image.publish(*image_msg);
}
void RadarRosbagHandler::UnpackPose(nav_msgs::Odometry::ConstPtr& odom_msg){
  nav_msgs::Odometry msg_odom = *odom_msg;
  poseStamped stamped_gt_pose = std::make_pair(Eigen::Affine3d::Identity(), odom_msg->header.stamp);
  tf::poseMsgToEigen(msg_odom.pose.pose, stamped_gt_pose.first);
  pose_stream_.push_back(stamped_gt_pose);

  msg_odom.header.stamp = ros::Time::now();
  msg_odom.header.frame_id = "world";
  pub_odom.publish(msg_odom);

}
std::shared_ptr<PoseScan> RadarRosbagHandler::Next(){


  while(m_!=view_->end() && ros::ok()){

    sensor_msgs::ImageConstPtr image_msg  = m_->instantiate<sensor_msgs::Image>();
    nav_msgs::Odometry::ConstPtr odom_msg = m_->instantiate<nav_msgs::Odometry>();
    if(image_msg  != NULL)
      UnpackImage(image_msg);
    else if(odom_msg != NULL)
      UnpackPose(odom_msg);

    if(radar_stream_.size()>3)
      radar_stream_.erase(radar_stream_.begin());
    if(pose_stream_.size()>3)
      pose_stream_.erase(pose_stream_.begin());

    cout<<"poses: "<<pose_stream_.size()<<endl;
    cout<<"images: "<<radar_stream_.size()<<endl;
    cout<<std::distance(view_->begin(), m_)<<"/"<<view_->size()<<endl;
    // Need to make sure image and pose are synced in time here. Actually they are except perhaps for the begining or the end, just discard until they are synced.
    if(radar_stream_.size()<3 || pose_stream_.size()<3){
      m_++;
      continue;
    }
    else{
      m_++;
      break;
    }
  }
  //At this point, the time stamps of
  //"pose_stream_[i].second" and
  //"radar_stream_[i]->header.stamp" must be the exact same < 0.00001.

  const Eigen::Affine3d Tmotion = pose_stream_.front().first.inverse()*pose_stream_.back().first; // this must be scaled down by 1/2

  if(scanPars_.scan_type == rawradar)
    return PoseScan_S(new RawRadar(radar_stream_[1], pose_stream_[1].first, Tmotion));
  else if(scanPars_.scan_type == kstrong)
    return PoseScan_S(new kstrongRadar(radar_stream_[1], pose_stream_[1].first, Tmotion, scanPars_.kstrong, scanPars_.z_min, scanPars_.range_res, scanPars_.range_min));
  else return nullptr;

}

MockupHandler::MockupHandler(){
  for(int i=1;i<=9;i+=step_resolution){
    pcl::PointXYZI p;
    p.x = i; p.y = i; p.z = i; p.intensity = 100;
    cloud.push_back(p);
  }

}
std::shared_ptr<PoseScan> MockupHandler::Next(){
  cout<<"Mockup::Next()"<<endl;
  if(step==100)
    return nullptr;
  else{
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation()<<(step++)*step_resolution, 0, 0;
    pcl::transformPointCloud(cloud, *transformed, T.inverse()); //transform cloud into frame of T
    return PoseScan_S(new RawLidar(transformed, T, Eigen::Affine3d::Identity()));
  }
}

}
