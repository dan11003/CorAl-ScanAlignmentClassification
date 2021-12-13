#include "alignment_checker/DataHandler.h"
namespace CorAlignment {


FiledataHandler::FiledataHandler(const std::string& directory, const std::string& poses, const std::string& data_prefix, int index_first_scan) {

}

std::shared_ptr<PoseScan> FiledataHandler::Next(){

}


RadarRosbagHandler::RadarRosbagHandler(const std::string& rosbag_path, const std::string& radar_topic, const std::string& gt_topic){

  bag_.open(rosbag_path, rosbag::bagmode::Read);
  //std::vector<std::string> topics = ;
  view_= std::make_unique<rosbag::View>(bag_, rosbag::TopicQuery({radar_topic,gt_topic}));
  m_ = view_->begin();
  pub_odom = nh_.advertise<nav_msgs::Odometry>("/gt", 1000);
  pub_image = nh_.advertise<sensor_msgs::Image>("/Navtech/Polar", 1000);

}
void RadarRosbagHandler::UnpackImage(sensor_msgs::ImageConstPtr& image_msg){
  sensor_msgs::Image img = *image_msg;

  cv_bridge::CvImagePtr cv_polar_image;
  cv_polar_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_8UC1);
  cv_polar_image->header.stamp =  image_msg->header.stamp;
  radar_stream_.push_back(cv_polar_image->image);

  cout<<"image timestamp: "<<cv_polar_image->header.stamp.toSec()<<", size: "<<cv_polar_image->image.cols<<", "<<cv_polar_image->image.rows<<endl;
  pub_image.publish(img);
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

    m_++;
  }
  return nullptr;

}

MockupHandler::MockupHandler(){
  for(int i=1;i<=9;i+=step_resolution){
    cloud.push_back(pcl::PointXYZ(i,i,i));
  }

}
std::shared_ptr<PoseScan> MockupHandler::Next(){
  cout<<"Mockup::Next()"<<endl;
  if(step==5)
    return nullptr;
  else{
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation()<<(step++)*step_resolution, 0, 0;
    pcl::transformPointCloud(cloud, *transformed, T.inverse()); //transform cloud into frame of T
    return PoseScan_S(new lidarscan(transformed, T, Eigen::Affine3d::Identity()));
 }
}

}
