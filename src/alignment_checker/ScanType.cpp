#include "alignment_checker/ScanType.h"
namespace CorAlignment {

int PoseScan::pose_count = 1;
lidarscan::lidarscan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot ) : PoseScan(T,Tmot), cloud_(cloud){

}
radarscan::radarscan(const cv::Mat& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot ): PoseScan(T,Tmot), polar_(polar){

}

pcl::PointCloud<pcl::PointXYZ>::Ptr lidarscan::GetCloudCopy(const Eigen::Affine3d& T){
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed( new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud_, *transformed, T);
  return transformed;

}
}
