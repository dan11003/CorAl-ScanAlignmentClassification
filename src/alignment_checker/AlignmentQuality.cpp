#include "alignment_checker/AlignmentQuality.h"
namespace CorAlignment {



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
  std::shared_ptr<lidarscan> ref_pcd = std::shared_ptr<lidarscan>(std::dynamic_pointer_cast<lidarscan>(ref));
  std::shared_ptr<lidarscan> src_pcd = std::shared_ptr<lidarscan>(std::dynamic_pointer_cast<lidarscan>(src));


  //Transform into "world" frame and than into frame of "ref"
  const Eigen::Affine3d Tsrc = src_pcd->GetAffine();
  const Eigen::Affine3d Tref = ref_pcd->GetAffine();
  const Eigen::Affine3d Tchange = Tref.inverse()*Tsrc*Toffset;
  //cout<<"change\n"<<Tchange.matrix()<<endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cld = src_pcd->GetCloudCopy(Tchange);//get cloud and also change reference frame to ref
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cld = ref_pcd->GetCloudNoCopy();
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

}

