#ifndef VIEWER_H
#define VIEWER_H
#include "stdio.h"
#include "iostream"
#include "Eigen/Eigen"
#include "thread"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "alignment_checker/scancomparsion.h"
#include "aliases.h"
#include "pcl/common/transforms.h"
#include "alignment_checker/utils.h"
namespace alignment_checker {


class viewer
{
public:
  viewer(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds,std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> &poses);

private:

  void KeyboardInputThread();

  void SetScans();

  bool new_input_ = false;
  double step_ = 0.01;
  bool downsample_=true;
  int target_idx_ = 0;
  Eigen::Matrix<double,6,1> offset_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> poses_;
  boostScan comp_;
  VisComparsion vis_;

  std::thread *input_th_;

};



}
#endif // VIEWER_H
