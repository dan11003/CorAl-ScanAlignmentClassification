#include "alignment_checker/scancomparsion.h"


namespace alignment_checker{

ScanComparsion::ScanComparsion(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, double radius, bool downsample) :
  src_overlap_(new pcl::PointCloud<pcl::PointXYZ>),
  tar_overlap_(new pcl::PointCloud<pcl::PointXYZ>),
  src_(new pcl::PointCloud<pcl::PointXYZ>),
  target_(new pcl::PointCloud<pcl::PointXYZ>)
{
  radius_ = radius;
  SetInput(src, target, downsample);
  CalculateOverlap();
  aligned_ = CheckAlignment();

}
void ScanComparsion::SetInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, bool downsample){

  if(downsample){
    pcl::VoxelGrid<pcl::PointXYZ>  sors;
    sors.setInputCloud(src);
    sors.setLeafSize(0.08f,0.08f,0.08f);
    sors.filter(*src_);

    pcl::VoxelGrid<pcl::PointXYZ>  sort;
    sort.setInputCloud(target);
    sort.setLeafSize(0.08f,0.08f,0.08f);
    sort.filter(*target_);
  }
  else{
    *src_ = *src;
    *target_ = *target;
  }

  ScanSrc_ = boost::shared_ptr<ScanType>(new ScanType(src_));
  ScanTarget_ = boost::shared_ptr<ScanType>(new ScanType(target_));
  CalculateOverlap();
  aligned_ = CheckAlignment();
}


bool ScanComparsion::CheckAlignment(double tolerance){

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_overlap (new pcl::PointCloud<pcl::PointXYZ>);

  (*merged_overlap)+=(*src_overlap_);
  (*merged_overlap)+=(*tar_overlap_);

  st_src_overlap = boost::shared_ptr<ScanType>(new ScanType(src_overlap_,true));
  st_tar_overlap = boost::shared_ptr<ScanType>(new ScanType(tar_overlap_,true));
  st_merged = boost::shared_ptr<ScanType>(new ScanType(merged_overlap,false));
  double inf_src = st_src_overlap->CalculateInformation(radius_);
  //cout<<"src: "<<inf_src<<endl;
  double inf_tar = st_tar_overlap->CalculateInformation(radius_);
  //cout<<"tar: "<<inf_tar<<endl;
  double separate = (inf_src+inf_tar)/2.0;//inf_src > inf_tar? inf_src : inf_tar;
  double inf_merged = st_merged->CalculateInformation(radius_);
  //cout<<"infmerged: "<<inf_merged<<endl;

  //cout<<"src_: "<<src_overlap_->size()<<", tar: "<<tar_overlap_->size()<<", mer: "<<merged_overlap->size()<<endl;
  cout<<"separate: "<<separate<<endl;
  cout<<"merged: "<<inf_merged<<endl;
  cout<<"diff: "<<inf_merged-separate<<endl;
  //cout<<"percent: "<<(inf_merged-separate)<<endl;


}
void ScanComparsion::CalculateOverlap(){
  src_overlap_->clear();
  tar_overlap_->clear();
  ScanTarget_->GetOverlap(src_, src_overlap_, tar_overlap_, radius_);

}

void ScanComparsion::StoreComparsionData(const std::string &dir, int suffix){
  std::ofstream tars, srcs, mergs;
   tars.open  (dir+"/"+"comp_tar_"+std::to_string(suffix));
   srcs.open  (dir+"/"+"comp_src_"+std::to_string(suffix));
   mergs.open (dir+"/"+"comp_merged_"+std::to_string(suffix));

   cout<<"save: "<<dir<<endl;

   for (const auto &e : st_src_overlap->GetEntropy()) srcs<< e <<endl;

   for (const auto &e : st_tar_overlap->GetEntropy()) tars<< e <<endl;

   for (const auto &e : st_merged->GetEntropy()) mergs<< e <<endl;

   srcs.close();
   tars.close();
   mergs.close();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ScanComparsion::GetMergedDifferential(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr diff(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged = st_merged->GetScan();
  if(merged==NULL){
    cerr<<"differential information failed"<<endl;
    return NULL;
  }
  diff->resize(merged->size());

  std::vector<double> separate_ent = st_src_overlap->GetEntropy();
  std::vector<double> merged_ent = st_merged->GetEntropy();
  separate_ent.insert( separate_ent.end(), st_src_overlap->GetEntropy().begin(), st_src_overlap->GetEntropy().end() );

if(separate_ent.size()!=merged_ent.size()){
  cerr<<"Wrong size"<<endl;
  return NULL;
}

  for(int i=0;i<merged->size();i++){
    pcl::PointXYZI p;
    p.x = merged->points[i].x;
    p.y = merged->points[i].y;
    p.z = merged->points[i].z;
    double e_diff = 256.0/2+merged_ent[i]-separate_ent[i];
    if(e_diff > 256)
      e_diff = 256;
    else if(e_diff < 0)
      e_diff = 0;

    p.intensity = e_diff;
    diff->points[i] = p;
  }

  return diff;
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
  pub_overlap_src = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("/overlap_src", 10);
  pub_overlap_tar = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("/overlap_target", 10);
  pub_overlap_merged = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("/overlap_merged", 10);
  pub_diff_merged = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("/differential_merged", 10);

  frameid_ = frameid;
}

void VisComparsion::PlotClouds(boost::shared_ptr<ScanComparsion> &comp){
  ros::Time tnow = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_info;

  cloud = Stamp(comp->GetSrc(),tnow);
  if(cloud !=NULL)
    pub_src.publish(cloud);

  cloud = Stamp(comp->GetTar(),tnow);
  if(cloud !=NULL)
    pub_tar.publish(cloud);

  cloud_info = Stamp(comp->GetSrcOverlap(),tnow);
  if(cloud_info !=NULL)
    pub_overlap_src.publish(cloud_info);

  cloud_info = Stamp(comp->GetTarOverlap(),tnow);
  pub_overlap_tar.publish(cloud_info);

  cloud_info = Stamp(comp->GetMergedOverlap(),tnow);
  pub_overlap_merged.publish(cloud_info);


  //cloud_info = Stamp(comp->GetMergedDifferential(),tnow);

  //pub_diff_merged.publish(cloud_info);


}

pcl::PointCloud<pcl::PointXYZ>::Ptr VisComparsion::Stamp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ros::Time &tcloud){
  pcl_conversions::toPCL(tcloud, cloud->header.stamp);
  cloud->header.frame_id = frameid_;
  return cloud;

}
pcl::PointCloud<pcl::PointXYZI>::Ptr VisComparsion::Stamp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const ros::Time &tcloud){
  pcl_conversions::toPCL(tcloud, cloud->header.stamp);
  cloud->header.frame_id = frameid_;
  return cloud;

}



}

