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


void AlignmentQualityPlot::PublishPoseScan(const std::string& topic, std::shared_ptr<PoseScan>& p_scan, const Eigen::Affine3d& T, const std::string& frame_id, const int value){

  if(p_scan == NULL)
    return;

  //Extract point cloud from PoseScan
  std::shared_ptr<RawLidar> l_scan = std::dynamic_pointer_cast<RawLidar>(p_scan);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_scan = l_scan->GetCloudNoCopy();

  //Publish point cloud
  std::map<std::string, ros::Publisher>::iterator it = AlignmentQualityPlot::pubs.find(topic);
  if(it == pubs.end()){
    ros::NodeHandle nh("~");
    pubs[topic] = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(topic,1000);
    it = AlignmentQualityPlot::pubs.find(topic);
  }
  ros::Time t = ros::Time::now();
  pcl_conversions::toPCL(t, cloud_scan->header.stamp);
  cloud_scan->header.frame_id = frame_id;
  //cout << cloud_scan->size() << endl;
  pubs[topic].publish(*cloud_scan);

  /*ros::NodeHandle nh;
  ros::Publisher cld_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(topic, 1000);
  ros::Time t = ros::Time::now();
  cld_pub.publish(*cloud_scan);
  */

  // Publish the corresponding reference frame
  static tf::TransformBroadcaster Tbr;
  tf::Transform Tf;
  std::vector<tf::StampedTransform> trans_vek;
  tf::transformEigenToTF(T, Tf);
  trans_vek.push_back(tf::StampedTransform(Tf, t, "/world", frame_id));
  Tbr.sendTransform(trans_vek);



}

/*
void MapPointNormal::PublishMap(const std::string& topic, MapNormalPtr map, Eigen::Affine3d& T, const std::string& frame_id, const int value){
  if(map==NULL)
    return;

  std::map<std::string, ros::Publisher>::iterator it = MapPointNormal::pubs.find(topic);
  if (it == pubs.end()){
    ros::NodeHandle nh("~");
    pubs[topic] =  nh.advertise<visualization_msgs::MarkerArray>(topic,100);
    it = MapPointNormal::pubs.find(topic);
  }
  //cout<<"publish to "<<topic<<endl;
  visualization_msgs::Marker m = DefaultMarker(ros::Time::now(), frame_id);
  m.action = visualization_msgs::Marker::DELETEALL;
  visualization_msgs::MarkerArray marr_delete;
  marr_delete.markers.push_back(m);
  it->second.publish(marr_delete);
  std::vector<cell> cells = map->TransformCells(T);
  visualization_msgs::MarkerArray marr = Cells2Markers(cells, ros::Time::now(), frame_id, value);
  it->second.publish(marr);
}*/

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
  nearby_src.resize(2,nr_nearby_src);
  nearby_ref.resize(2,nr_nearby_ref);
  merged.resize(2,nr_nearby_ref + nr_nearby_src);
  int tot = 0;
  for(std::size_t i = 0; i < pointIdxRadiusSearch_src.size (); i++){
    nearby_src(0,i) = src_pcd->points[pointIdxRadiusSearch_src[i]].x;
    nearby_src(1,i) = src_pcd->points[pointIdxRadiusSearch_src[i]].y;
    merged.block<2,1>(0,tot++) = nearby_src.block<2,1>(0,i);
  }
  for(std::size_t i = 0; i < pointIdxRadiusSearch_ref.size (); i++){
    nearby_ref(0,i) = src_pcd->points[pointIdxRadiusSearch_ref[i]].x;
    nearby_ref(1,i) = ref_pcd->points[pointIdxRadiusSearch_ref[i]].y;
    merged.block<2,1>(0,tot++) = nearby_ref.block<2,1>(0,i);
  }


}
bool CorAlRadarQuality::Covariance(Eigen::MatrixXd& x, Eigen::Matrix2d& cov){ //mean already subtracted from x
  Eigen::Matrix2d covSum = x.transpose()*x;
  float n = x.rows();
  cov = covSum*1.0/(n-1.0);
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov.block<2,2>(0,0));
  double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
  //cout<<"cond "<<cond<<endl;
  return cond < 100000;
}
CorAlRadarQuality::CorAlRadarQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset)  : AlignmentQuality(src, ref, par, Toffset)
{
  ref_pcd = ac::pcl3dto2d(ref->GetCloudCopy(ref->GetAffine()), ref_i);
  src_pcd = ac::pcl3dto2d(src->GetCloudCopy(src->GetAffine()*Toffset), src_i);
  kd_src.setInputCloud(src_pcd);
  kd_ref.setInputCloud(ref_pcd);

  sep_res_.resize(src_pcd->size() + ref_pcd->size(), 0);
  sep_valid.resize(src_pcd->size() +ref_pcd->size(), false);
  joint_res_.resize(src_pcd->size() + ref_pcd->size(), 0);


  int pnt = 0;
  for (auto && searchPoint : src_pcd->points){
    Eigen::MatrixXd msrc, mref, mmerged;
    GetNearby(searchPoint, msrc, mref, mmerged);
    cout<<"before subtract: "<< msrc<<endl;
    cout<<"mean: "<<msrc.rowwise().mean()<<endl;
    msrc -= msrc.rowwise().mean(); // I think this sould subtract mean
    mmerged -= mmerged.rowwise().mean();
    cout<<"after subtract: "<< msrc<<endl;
    Eigen::Matrix2d Csrc, Cmerged;
    bool success_src = Covariance(msrc,Csrc);
    bool success_merged = Covariance(msrc,Cmerged);
    if(success_src && success_merged){


    }

  }



}

}

