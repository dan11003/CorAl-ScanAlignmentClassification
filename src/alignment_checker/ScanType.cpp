#include "alignment_checker/ScanType.h"
namespace CorAlignment {

int PoseScan::pose_count = 1;

std::string Scan2str(const scan_type& val){
    switch (val){
    case rawlidar: return "rawlidar";
    case rawradar: return "rawradar";
    case kstrong: return "kstrong";
    case cen2018: return "cen2018";
    case cfear: return "cfear";
    case kstrongCart: return "kstrongCart";
    default: return "none";
    }
}
scan_type Str2Scan(const std::string& val){
    if (val=="rawlidar")
        return scan_type::rawlidar;
    else if (val=="rawradar")
        return scan_type::rawradar;
    else if (val=="kstrong")
        return scan_type::kstrong;
    else if (val=="cen2018")
        return scan_type::cen2018;
    else if (val=="kstrongCart")
        return scan_type::kstrongCart;
    else if (val=="none")
        return scan_type::none;
    else if (val=="cfear")
        return scan_type::cfear;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr PoseScan::GetCloudCopy(const Eigen::Affine3d& T){
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed( new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud_, *transformed, T);
    return transformed;
}



kstrongRadar::kstrongRadar(cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot, int kstrong, double z_min, double range_res, double min_distance) : RawRadar(polar, T, Tmot){
    assert(polar !=NULL);
    radar_mapping::k_strongest_filter(polar, cloud_, kstrong, z_min, range_res, min_distance);
    assert(cloud_ != nullptr);
    cout<<"k strongest: "<<cloud_->size()<<endl;
}

CFEARFeatures::CFEARFeatures(cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot , int kstrong, double z_min, double range_res, double min_distance, double resolution ) : kstrongRadar(polar, T, Tmot, kstrong, z_min, range_res, min_distance){
    CFEARFeatures_ = radar_mapping::MapNormalPtr(new radar_mapping::MapPointNormal(cloud_, resolution));
}
CartesianRadar::CartesianRadar(cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot, int kstrong, double z_min, double range_res, double min_distance ): RawRadar(polar,T,Tmot){
    polar_ = polar;
    cart_ = boost::make_shared<cv_bridge::CvImage>();
    cart_->encoding = polar_->encoding;
    cart_->header.stamp = polar_->header.stamp;
    cout<<"CartesianRadar::CartesianRadar"<<endl;
    radar_mapping::KstrongestPolar filter(z_min,kstrong,min_distance);
    filter.getFilteredImage(polar_,polar_filtered_);
    std::vector<double> azimuths;
    for (int bearing = 0; bearing < polar->image.rows; bearing++)
      azimuths.push_back( ((double)(bearing+1) / polar_->image.rows) * 2 * M_PI);
    //alignment_checker::radar_polar_to_cartesian(polar_->image,azimuths,cart_->image);
    cout<<"end CartesianRadar::CartesianRadar"<<endl;
}

}
