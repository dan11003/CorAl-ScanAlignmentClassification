#include "alignment_checker/AlignmentQuality.h"
namespace CorAlignment {



std::vector<double> p2dQuality::GetQualityMeasure(){
  return {0,0,0};
}
std::vector<double> p2pQuality::GetQualityMeasure(){
	
	//Calculate the mean of all the residuals
	int residuals_size_ = residuals_.size();
	double means_ = 0;
	for(int i=0; i<residuals_size_; i++){
		means_ += residuals_[i]; 
	}
	means_ = means_/double(residuals_size_);
  return {means_};	
}
std::vector<double> CorAl::GetQualityMeasure(){
  return {0,0,0};
}



p2pQuality::p2pQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset) : AlignmentQuality(src, ref, par, Toffset){
  cout<<"create p2p quality"<<endl;

  //pcl::transform
  std::shared_ptr<lidarscan> ref_pcd = std::shared_ptr<lidarscan>(std::dynamic_pointer_cast<lidarscan>(ref));
  std::shared_ptr<lidarscan> src_pcd = std::shared_ptr<lidarscan>(std::dynamic_pointer_cast<lidarscan>(src));


  //Transform into "world" frame and than into frame of "ref"
  const Eigen::Affine3d Tsrc = src_pcd->GetAffine();
  const Eigen::Affine3d Ttar = src_pcd->GetAffine();
  const Eigen::Affine3d Tchange = Ttar.inverse()*Tsrc*Toffset;
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cld = src_pcd->GetCloudCopy(Tchange);//get cloud and also change reference frame to ref
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cld = ref_pcd->GetCloudNoCopy();

  //Build kd tree for fast neighbor search
  kdtree_.setInputCloud(ref_cld); // ref cloud

  //Calculate the sqaured distances for all 'p' in the point cloud
  
  for(auto && p : src_cld->points){
    std::vector<float> pointRadiusSquaredDistance;
    std::vector<int> pointIdxRadiusSearch;
    const double radius = 1;
    kdtree_.setSortedResults(true);

    if ( kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
		residuals_.push_back(pointRadiusSquaredDistance[0]);
		
      //for (int i=0;i<pointIdxRadiusSearch.size();i++) {
        //pcl::PointXYZ p_ref = ref_cld->points[pointIdxRadiusSearch[0]];

        

      //}

    }
  }


}


}

