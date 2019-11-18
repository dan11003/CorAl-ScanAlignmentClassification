#include "alignment_checker/scancomparsion.h"


namespace alignment_checker{

ScanComparsion::ScanComparsion( const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target) :
  src_overlap_(new pcl::PointCloud<pcl::PointXYZ>),
  tar_overlap_(new pcl::PointCloud<pcl::PointXYZ>)
{
  src_ = src;
  target_ = target;
  ScanSrc_ = boost::shared_ptr<ScanType>(new ScanType(src_));
  ScanTarget_ = boost::shared_ptr<ScanType>(new ScanType(target_));
  cout<<"calculate overlap"<<endl;
  CalculateOverlap();
  cout<<"done calculate overlap"<<endl;
}


bool ScanComparsion::CheckAlignment(double tolerance){

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_overlap (new pcl::PointCloud<pcl::PointXYZ>);

  (*merged_overlap)+=(*src_overlap_);
  (*merged_overlap)+=(*tar_overlap_);
  ScanType st_src(src_overlap_);
  ScanType st_tar(tar_overlap_);
  ScanType st_merged(merged_overlap);
  double inf_src = st_src.GetInformation(radius_);
  cout<<"src: "<<inf_src<<endl;
  double inf_tar = st_tar.GetInformation(radius_);
  cout<<"tar: "<<inf_tar<<endl;
  double separate = (inf_src + inf_tar)/2.0;
  double inf_merged = st_merged.GetInformation(radius_);


  cout<<"separate: "<<separate<<endl;
  cout<<"merged: "<<inf_merged<<endl;
  cout<<"ratio: "<<inf_merged/separate<<endl;


}
void ScanComparsion::CalculateOverlap(){
  src_overlap_->clear();
  tar_overlap_->clear();
  ScanTarget_->GetOverlap(src_, src_overlap_, tar_overlap_, radius_);
}
void ScanComparsion::SetNext( const pcl::PointCloud<pcl::PointXYZ>::Ptr &src){
  target_ = src_;
  ScanTarget_ = ScanSrc_;
  src_ = src;
  ScanSrc_ = boost::shared_ptr<ScanType>(new ScanType(src_));
  CalculateOverlap();
}


VisComparsion::VisComparsion(const std::string frameid): nh_("~")
{
  pub_src = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/src", 10);
  pub_tar = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/target", 10);
  pub_overlap_src = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/overlap_src", 10);
  pub_overlap_tar = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/overlap_target", 10);
  frameid_ = frameid;
}

void VisComparsion::PlotClouds(boost::shared_ptr<ScanComparsion> &comp){
  ros::Time tnow = ros::Time::now();
  cout<<"pub src"<<endl;
  pub_src.publish(Stamp(comp->GetSrc(),tnow));
  cout<<"pub tar"<<endl;
  pub_tar.publish(Stamp(comp->GetTar(),tnow));
  cout<<"pub overlap_src"<<endl;
  pub_overlap_src.publish(Stamp(comp->GetSrcOverlap(),tnow));
  cout<<"pub overlap_tar"<<endl;
  pub_overlap_tar.publish( Stamp(comp->GetTarOverlap(),tnow));
  cout<<"published all"<<endl;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr VisComparsion::Stamp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ros::Time &tcloud){
pcl_conversions::toPCL(tcloud, cloud->header.stamp);
cloud->header.frame_id = frameid_;
return cloud;
}



}

