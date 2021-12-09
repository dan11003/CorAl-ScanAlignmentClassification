#pragma once

#include "alignment_checker/ScanType.h"
#include "boost/shared_ptr.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "alignment_checker/Utils.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"

namespace CorAlignment {

typedef std::pair<Eigen::Affine3d,ros::Time> poseStamped;

class dataHandler
{
public:
  dataHandler() {}

  virtual std::shared_ptr<PoseScan> Next() = 0;
};

class FiledataHandler: public dataHandler
{
public:
  FiledataHandler(const std::string& directory, const std::string& poses, const std::string& data_prefix="", int index_first_scan = 0);

  std::shared_ptr<PoseScan> Next();
};

class RadarRosbagHandler: public dataHandler
{
public:
  RadarRosbagHandler(const std::string& rosbag_path, const std::string& radar_topic = "/Navtech/Polar", const std::string& gt_topic = "/gt");

  std::shared_ptr<PoseScan> Next();

protected:
  rosbag::Bag bag_;
  std::unique_ptr<rosbag::View> view_pose_,view_image_;


  std::vector<sensor_msgs::Image> radar_stream_;
  std::vector<poseStamped> pose_stream_;
};

typedef std::unique_ptr<dataHandler> dataHandler_U;


}
