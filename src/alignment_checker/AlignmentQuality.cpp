#include "alignment_checker/AlignmentQuality.h"
namespace CorAlignment {


lidarscan::lidarscan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot ) : PoseScan(T,Tmot), cloud_(cloud){

}
radarscan::radarscan(const cv::Mat& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot ): PoseScan(T,Tmot), polar_(polar){

}

std::vector<double> p2dQuality::GetQualityMeasure(){
  return {0,0,0};
}
std::vector<double> CorAl::GetQualityMeasure(){
  return {0,0,0};
}


}
