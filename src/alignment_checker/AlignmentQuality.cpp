#include "alignment_checker/AlignmentQuality.h"
namespace CorAlignment {



std::vector<double> p2dQuality::GetQualityMeasure(){
  return {0,0,0};
}
std::vector<double> CorAl::GetQualityMeasure(){
  return {0,0,0};
}


p2pQuality::p2pQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset) : AlignmentQuality(src, ref, par, Toffset){

  //pcl::transform
  std::shared_ptr<lidarscan> ref_pcd = std::dynamic_pointer_cast<lidarscan>(ref);
  std::shared_ptr<lidarscan> src_pcd = std::dynamic_pointer_cast<lidarscan>(src);

  //Transform into "world" frame and than into frame of "ref"
  const Eigen::Affine3d Tsrc = src_pcd->GetAffine();
  const Eigen::Affine3d Ttar = src_pcd->GetAffine();
  const Eigen::Affine3d Tchange = Ttar.inverse()*Tsrc*Toffset;
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cld = src_pcd->GetCloudCopy(Tchange);//get cloud and also change reference frame to ref
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cld = ref_pcd->GetCloudNoCopy();

  //Build kd tree for fast neighbor search
  //kdtree_.setInputCloud(ref_cld); // ref cloud

  //For all p in
  for(auto && p : src_cld->points){
    std::vector<float> pointRadiusSquaredDistance;
    std::vector<int> pointIdxRadiusSearch;
    const double radius = 1;
    if ( kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
      for (int i=0;i<pointIdxRadiusSearch.size();i++) {
        pcl::PointXYZ p = ref_cld->points[pointIdxRadiusSearch[i]];
        // Do whatever needs to be done :)

      }

    }
  }

}


}

