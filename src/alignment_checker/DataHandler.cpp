#include "alignment_checker/DataHandler.h"
namespace CorAlignment {


FiledataHandler::FiledataHandler(const std::string& directory, const std::string& poses, const std::string& data_prefix, int index_first_scan) {

}

std::shared_ptr<PoseScan> FiledataHandler::Next(){

}


RadarRosbagHandler::RadarRosbagHandler(const std::string& rosbag_path, const std::string& radar_topic, const std::string& gt_topic){

  bag_.open(rosbag_path, rosbag::bagmode::Read);
  //std::vector<std::string> topics = ;
  view_pose_ = std::make_unique<rosbag::View>(bag_, rosbag::TopicQuery({gt_topic}));
  view_image_ = std::make_unique<rosbag::View>(bag_, rosbag::TopicQuery({radar_topic}));


}

std::shared_ptr<PoseScan>RadarRosbagHandler::Next(){

  static rosbag::View::iterator m_img = view_image_->begin();
  //while(m!=view_->end() && ros::ok()){




}

}
