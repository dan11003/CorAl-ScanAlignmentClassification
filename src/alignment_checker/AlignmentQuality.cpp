#include "alignment_checker/AlignmentQuality.h"
namespace CorAlignment {

std::map<std::string, ros::Publisher> AlignmentQualityPlot::pubs = std::map<std::string, ros::Publisher>();

std::vector<double> p2dQuality::GetQualityMeasure(){
  return {0,0,0};
}
std::vector<double> p2pQuality::GetQualityMeasure(){

  //Calculate the mean of all the residuals
  int residuals_size = residuals_.size();
  if(residuals_size  == 0)
    return {0, 0, 0};

  double means = 0;
  for(int i=0; i<residuals_size; i++){
    means += residuals_[i];
  }
  means = means/double(residuals_size);
  return {means, 0, 0};
}

std::vector<double> CorAl::GetQualityMeasure(){
  return {0,0,0};
}




p2pQuality::p2pQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset) : AlignmentQuality(src, ref, par, Toffset){


  //pcl::transform

  auto ref_pcd =  std::dynamic_pointer_cast<RawLidar>(ref);
  auto src_pcd = std::dynamic_pointer_cast<RawLidar>(src);


  //Transform into "world" frame and than into frame of "ref"
  const Eigen::Affine3d Tsrc = src_pcd->GetAffine();
  const Eigen::Affine3d Tref = ref_pcd->GetAffine();
  const Eigen::Affine3d Tchange = Tref.inverse()*Tsrc*Toffset;
  //cout<<"change\n"<<Tchange.matrix()<<endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr src_cld = src_pcd->GetCloudCopy(Tchange);//get cloud and also change reference frame to ref
  pcl::PointCloud<pcl::PointXYZI>::Ptr ref_cld = ref_pcd->GetCloudNoCopy();
  //cout<<"src: "<<src_cld->size()<<", ref: "<<ref_cld->size()<<endl;
  /*for(auto && p : src_cld->points)
    cout<<"src: "<<p.getArray3fMap().transpose()<<endl;
  for(auto && p : ref_cld->points)
    cout<<"ref: "<<p.getArray3fMap().transpose()<<endl;*/

  //Build kd tree for fast neighbor search
  kdtree_.setInputCloud(ref_cld); // ref cloud

  //Calculate the sqaured distances for all 'p' in the point cloud

  for(auto && p : src_cld->points){
    std::vector<float> pointRadiusSquaredDistance;
    std::vector<int> pointIdxRadiusSearch;
    const double radius = par_.radius;
    kdtree_.setSortedResults(true);

    if ( kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
      residuals_.push_back(pointRadiusSquaredDistance[0]);
    }
  }
  //cout<<"residuals: "<<residuals_.size()<<endl;


}
void AlignmentQualityPlot::PublishCloud(const std::string& topic, pcl::PointCloud<pcl::PointXYZI>::Ptr& cld_plot, const Eigen::Affine3d& T, const std::string& frame_id, const int value){

  std::map<std::string, ros::Publisher>::iterator it = AlignmentQualityPlot::pubs.find(topic);
  if(it == pubs.end()){
    ros::NodeHandle nh("~");
    pubs[topic] = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(topic,1000);
    it = AlignmentQualityPlot::pubs.find(topic);
  }
  ros::Time t = ros::Time::now();
  pcl_conversions::toPCL(t, cld_plot->header.stamp);
  cld_plot->header.frame_id = frame_id;
  pubs[topic].publish(*cld_plot);


  // Publish the corresponding reference frame
  static tf::TransformBroadcaster Tbr;
  tf::Transform Tf;
  std::vector<tf::StampedTransform> trans_vek;
  tf::transformEigenToTF(T, Tf);
  trans_vek.push_back(tf::StampedTransform(Tf, t, "/world", frame_id));
  Tbr.sendTransform(trans_vek);
}

void AlignmentQualityPlot::PublishPoseScan(const std::string& topic, std::shared_ptr<PoseScan>& scan_plot, const Eigen::Affine3d& T, const std::string& frame_id, const int value){

  if(scan_plot == NULL)
    return;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cld_plot = scan_plot->GetCloudNoCopy(); //Extract point cloud from PoseScan
  PublishCloud(topic,cld_plot,T,frame_id,value);
}

CFEARQuality::CFEARQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset)  : AlignmentQuality(src, ref, par, Toffset){
  auto CFEAR_src = std::dynamic_pointer_cast<CFEARFeatures>(src);
  auto CFEAR_ref = std::dynamic_pointer_cast<CFEARFeatures>(ref);
  assert(CFEAR_src!=NULL && CFEAR_ref!=NULL);
  radar_mapping::costmetric pnt_cost = radar_mapping::Str2Cost(par.method);
  if(pnt_cost==radar_mapping::costmetric::P2L)
    cout<<"P2L cost";
  else if(pnt_cost==radar_mapping::costmetric::P2D)
    cout<<"P2D cost";
  else
    cout<<"P2P cost";

  radar_mapping::n_scan_normal_reg reg(pnt_cost, radar_mapping::losstype::None, 0);
  std::vector<radar_mapping::MapNormalPtr> feature_vek = {CFEAR_ref->CFEARFeatures_, CFEAR_src->CFEARFeatures_};
  std::vector<Eigen::Affine3d> Tvek = {CFEAR_ref->GetAffine(),CFEAR_src->GetAffine()*Toffset};
  radar_mapping::MapPointNormal::PublishMap("scan1", feature_vek[0], Tvek[0], "world", 1 );
  radar_mapping::MapPointNormal::PublishMap("scan2", feature_vek[1], Tvek[1], "world", -1);
  reg.GetCost(feature_vek, Tvek, quality_.front(), residuals_);
  cout<<"CFEAR Feature cost: "<<quality_.front()<<", residuals: "<<residuals_.size()<<", normalized"<<quality_.front()/(residuals_.size()+0.0001)<<endl;
}

void CorAlRadarQuality::GetNearby(const pcl::PointXY& query, Eigen::MatrixXd& nearby_src, Eigen::MatrixXd& nearby_ref, Eigen::MatrixXd& merged){
  std::vector<int> pointIdxRadiusSearch_src, pointIdxRadiusSearch_ref;
  std::vector<float> pointRadiusSquaredDistance_src, pointRadiusSquaredDistance_ref;
  int nr_nearby_src = kd_src.radiusSearch(query, par_.radius, pointIdxRadiusSearch_src, pointRadiusSquaredDistance_src);
  int nr_nearby_ref = kd_ref.radiusSearch(query, par_.radius, pointIdxRadiusSearch_ref, pointRadiusSquaredDistance_ref);
  nearby_src.resize(nr_nearby_src,2);
  nearby_ref.resize(nr_nearby_ref,2);
  merged.resize(nr_nearby_ref + nr_nearby_src,2);
  int tot = 0;
  for(std::size_t i = 0; i < pointIdxRadiusSearch_src.size (); i++){
    nearby_src(i,0) = src_pcd->points[pointIdxRadiusSearch_src[i]].x;
    nearby_src(i,1) = src_pcd->points[pointIdxRadiusSearch_src[i]].y;
    merged.block<1,2>(tot++,0) = nearby_src.block<1,2>(i,0);
  }
  for(std::size_t i = 0; i < pointIdxRadiusSearch_ref.size (); i++){
    nearby_ref(i,0) = ref_pcd->points[pointIdxRadiusSearch_ref[i]].x;
    nearby_ref(i,1) = ref_pcd->points[pointIdxRadiusSearch_ref[i]].y;
    merged.block<1,2>(tot++,0) = nearby_ref.block<1,2>(i,0);
  }


}
bool CorAlRadarQuality::Covariance(Eigen::MatrixXd& x, Eigen::Matrix2d& cov){ //mean already subtracted from x
  //Compute and subtract mean
  //cout<<"before subtract: "<< x<<endl;
  //cout<<"mean: "<<x.rowwise().mean().transpose()<<endl;
  Eigen::MatrixXd mean = x.colwise().mean();

  for(int i=0;i<x.rows();i++) // subtract mean
    x.block<1,2>(i,0) = x.block<1,2>(i,0) - mean;

  Eigen::Matrix2d covSum = x.transpose()*x;
  float n = x.rows();
  cov = covSum*1.0/(n-1.0);
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov.block<2,2>(0,0));
  double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
  //cout<<"cond "<<cond<<endl;

  return cond < 100000;
}
bool CorAlRadarQuality::ComputeEntropy(const Eigen::Matrix2d& cov_sep, const Eigen::Matrix2d& cov_joint, int index){



  double det_j = cov_joint.determinant();
  double det_s = cov_sep.determinant();
  if(isnanl(det_s) || isnanl(det_j))
    return false;

  const double sep_entropy =  1.0/2.0*log(2.0*M_PI*exp(1.0)*det_s+0.00000001);
  const double joint_entropy = 1.0/2.0*log(2.0*M_PI*exp(1.0)*det_j+0.00000001);

  if( isnan(sep_entropy) || isnan(joint_entropy) )
    return false;
  else{
    sep_res_[index] = sep_entropy;
    joint_res_[index] = joint_entropy;
    return true;
  }
}
CorAlRadarQuality::CorAlRadarQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset)  : AlignmentQuality(src, ref, par, Toffset)
{
  cout<<"CorAl Quality"<<endl;
  ref_pcd_entropy = ref->GetCloudCopy(ref->GetAffine());
  src_pcd_entropy = src->GetCloudCopy(src->GetAffine()*Toffset);
  assert(ref_pcd_entropy != NULL && src_pcd_entropy !=NULL);


  src_pcd = ac::pcl3dto2d(src_pcd_entropy, src_i);
  ref_pcd = ac::pcl3dto2d(ref_pcd_entropy, ref_i);
  kd_src.setInputCloud(src_pcd);
  kd_ref.setInputCloud(ref_pcd);
  const int merged_size = src_pcd->size() + ref_pcd->size();
  assert(src_pcd->size() > 0 && ref_pcd->size());

  sep_res_.resize(merged_size, 100.0);
  sep_valid.resize(merged_size, false);
  joint_res_.resize(merged_size, 100.0);
  diff_res_.resize(merged_size, 100.0);


  int index = 0;
  for (auto && searchPoint : src_pcd->points){
    Eigen::MatrixXd msrc, mref, mjoint;
    //cout<<"get nearby"<<endl;
    GetNearby(searchPoint, msrc, mref, mjoint);
    //cout<<"got nearby"<<endl;
    if(mref.cols() < overlap_req_){
      index++;
      continue;
    }

    Eigen::Matrix2d sep_cov, joint_cov;
    if( Covariance(msrc,sep_cov) && Covariance(mjoint,joint_cov) ){
      if( ComputeEntropy(sep_cov,joint_cov,index) ){

        sep_valid[index] = true;
        diff_res_[index] = joint_res_[index] - sep_res_[index];
        joint_ += joint_res_[index]; sep_ += sep_res_[index];
        count_valid++;
      }
    }
    index++;
  }
  for (auto && searchPoint : ref_pcd->points){
    Eigen::MatrixXd msrc, mref, mjoint;
    GetNearby(searchPoint, msrc, mref, mjoint);
    if(msrc.cols() < overlap_req_)
      continue;
    Eigen::Matrix2d sep_cov, joint_cov;
    if( Covariance(mref,sep_cov) && Covariance(mjoint,joint_cov) ){
      if( ComputeEntropy(sep_cov,joint_cov,index) ){
        sep_valid[index] = true;
        diff_res_[index] = joint_res_[index] - sep_res_[index];
        joint_ += joint_res_[index]; sep_ += sep_res_[index];
        count_valid++;
      }
    }
    index++;
  }
  if(count_valid > 0){
    sep_ /=count_valid;
    joint_ /=count_valid;
  }
  diff_ = joint_ - sep_;
  cout<<"joint: "<<joint_<<", sep: "<<sep_<<", diff_"<<diff_<<", N: "<<count_valid<<endl;

  // Create merged point cloud
  merged_entropy = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  *merged_entropy += *src_pcd_entropy;
  *merged_entropy += *ref_pcd_entropy;

  //Assign entrpy values
  index = 0;
  for (auto && p : src_pcd_entropy->points)  // asign entropy
    p.intensity = sep_res_[index]; // set intensity
  for (auto && p : ref_pcd_entropy->points)  // asign entropy
    p.intensity = sep_res_[index++]; // set intensity

  for (auto && p : merged_entropy->points)  // asign entropy
    p.intensity = joint_res_[index]; // set intensity

  AlignmentQualityPlot::PublishCloud("/coral_src",    src_pcd_entropy, Eigen::Affine3d::Identity(), "coral_world");
  AlignmentQualityPlot::PublishCloud("/coral_ref",    ref_pcd_entropy, Eigen::Affine3d::Identity(), "coral_world");
  AlignmentQualityPlot::PublishCloud("/coral_merged", merged_entropy,  Eigen::Affine3d::Identity(), "coral_world");
}

}

