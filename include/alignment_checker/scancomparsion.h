#ifndef SCANCOMPARISON_H
#define SCANCOMPARISON_H
#include "alignment_checker/scan.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "iostream"
namespace alignment_checker {

using std::endl;
using std::cout;
using std::cerr;

class ScanComparsion
{

public:

  ScanComparsion(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, double radius =0.5, bool downsample = true);

  void SetNext(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src);

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetSrc(){return src_;}

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetTar(){return target_;}

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetSrcOverlap(){if(st_src_overlap!=NULL) return st_src_overlap->GetScanWithInformation(); else return NULL;}

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetTarOverlap(){if(st_tar_overlap!=NULL) return st_tar_overlap->GetScanWithInformation(); else return NULL;}

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetMergedOverlap(){if(st_merged!=NULL) return st_merged->GetScanWithInformation(); else return NULL;}

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetMergedDifferential();

  void SetInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, bool downsample);

  void StoreComparsionData(const std::string &dir, int suffix);


protected:

  void CalculateOverlap();

  bool CheckAlignment(double tolerance=0.3);


  pcl::PointCloud<pcl::PointXYZ>::Ptr src_, target_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_overlap_, src_overlap_;
  boost::shared_ptr<ScanType> ScanSrc_, ScanTarget_;

  boost::shared_ptr<ScanType> st_src_overlap = NULL;
  boost::shared_ptr<ScanType> st_tar_overlap = NULL;
  boost::shared_ptr<ScanType> st_merged = NULL;
  double radius_;
  bool aligned_;

};

typedef boost::shared_ptr<ScanComparsion> boostScan;

class VisComparsion
{
public:

  VisComparsion(const std::string frameid="/world");

  void PlotClouds(boost::shared_ptr<ScanComparsion> &comp);


private:


  pcl::PointCloud<pcl::PointXYZ>::Ptr Stamp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ros::Time &tcloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr Stamp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const ros::Time &tcloud);

  ros::NodeHandle nh_;
  ros::Publisher pub_src, pub_tar, pub_overlap_src, pub_overlap_tar,pub_overlap_merged, pub_diff_merged;
  std::string frameid_;
};

}
#endif // SCANCOMPARISON_H
