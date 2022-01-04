#include "alignment_checker/ScanType.h"
namespace CorAlignment {

int PoseScan::pose_count = 1;

std::string Scan2str(const scan_type& val){
    switch (val){
    case rawlidar: return "rawlidar";
    case rawradar: return "rawradar";
    case kstrong: return "kstrong";
    case kstrongStructured: return "kstrongStructured";
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
    else if (val=="kstrongStructured")
        return scan_type::kstrongStructured;
    else if (val=="cen2018")
        return scan_type::cen2018;
    else if (val=="kstrongCart")
        return scan_type::kstrongCart;
    else if (val=="none")
        return scan_type::none;
    else if (val=="cfear")
        return scan_type::cfear;
}
PoseScan::PoseScan(const PoseScan::Parameters pars, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion)
    : Test_(T), Tmotion_(Tmotion), cloud_(new pcl::PointCloud<pcl::PointXYZI>()), pose_id(pose_count++),pars_(pars)
{

}

RawRadar::RawRadar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion)
    : PoseScan(pars,T,Tmotion), range_res_(pars.range_res)
{
    polar_ = polar;
}

kstrongRadar::kstrongRadar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion)
    : RawRadar(pars, polar, T, Tmotion)
{
    assert(polar !=NULL);

    radar_mapping::k_strongest_filter(polar, cloud_, pars.kstrong, pars.z_min, pars.range_res, pars.sensor_min_distance);
    assert(cloud_ != nullptr);
    if(pars.compensate){
        //cout<<"compensate"<<Tmotion.translation().transpose()<<endl;
        //const Eigen::Affine3d Tmotion_next = T.inverse()*Tnext;
        //const Eigen::Affine3d Tmotion = Tprev.inverse()*Tnext;
        //Eigen::Vector3d par;
        //radar_mapping::Affine3dToEigVectorXYeZ(Tmotion,par);
        //Eigen::Affine3d Tmotion_adjusted = radar_mapping::vectorToAffine3d(par(0)/2.0, par(1)/2.0, 0, 0, 0, par(2)/2.0);

        radar_mapping::Compensate(cloud_, Tmotion_, false); //cout<<"k strongest: "<<cloud_->size()<<endl;
    }
}
kstrongStructuredRadar::kstrongStructuredRadar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion)
    : RawRadar(pars, polar, T, Tmotion)
{
    assert(polar !=NULL);
    radar_mapping::StructuredKStrongest kstrong(polar_, pars.z_min, pars.kstrong, pars.sensor_min_distance, pars.range_res);
    kstrong.getPeaksFilteredPointCloud(cloud_,true); // get peaks
    kstrong_peaks_ = cloud_;

    kstrong.getPeaksFilteredPointCloud(kstrong_filtered_,false); // all k-strongest points

    if(pars.compensate){
        radar_mapping::Compensate(kstrong_peaks_, Tmotion_, false); //cout<<"k strongest: "<<cloud_->size()<<endl;
        radar_mapping::Compensate(kstrong_filtered_, Tmotion_, false); //cout<<"k strongest: "<<cloud_->size()<<endl;
    }
}

CFEARFeatures::CFEARFeatures(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion )
    : kstrongRadar(pars, polar, T, Tmotion)
{

    CFEARFeatures_ = radar_mapping::MapNormalPtr(new radar_mapping::MapPointNormal(cloud_, pars.resolution));
    //cout<<"frame: "<<pose_id<<"time: "<<cloud_->header.stamp<<", "<<cloud_->size()<<", "<<CFEARFeatures_->GetSize()<<endl;
}


CartesianRadar::CartesianRadar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion )
    : RawRadar(pars,polar,T,Tmotion), sensor_min_distance(pars.sensor_min_distance), cart_resolution_(pars.cart_resolution), cart_pixel_width_(pars.cart_pixel_width){
    polar_ = polar;
    cart_ = boost::make_shared<cv_bridge::CvImage>();
    cart_->encoding = polar_->encoding;
    cart_->header.stamp = polar_->header.stamp;
    //cout<<"CartesianRadar::CartesianRadar"<<endl;
    radar_mapping::KstrongestPolar filter(pars.z_min, pars.kstrong, pars.sensor_min_distance);
    //filter.getFilteredImage(polar_,polar_filtered_);
    polar_->image.convertTo(polar_->image, CV_32F, 1/255.0);


    std::vector<double> azimuths;
    for (int bearing = 0; bearing < polar->image.rows; bearing++)
        azimuths.push_back( ((double)(bearing+1) / polar_->image.rows) * 2 * M_PI);

    alignment_checker::radar_polar_to_cartesian(polar_->image, azimuths, cart_->image);

}

pcl::PointCloud<pcl::PointXYZI>::Ptr PoseScan::GetCloudCopy(const Eigen::Affine3d& T){
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed( new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud_, *transformed, T);
    return transformed;
}

PoseScan_S RadarPoseScanFactory(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr radar_msg, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion){
    if(pars.scan_type == rawradar)
        return PoseScan_S(new RawRadar(pars, radar_msg, T, Tmotion));
    else if(pars.scan_type == kstrong)
        return PoseScan_S(new kstrongRadar(pars, radar_msg, T, Tmotion));
    else if(pars.scan_type == ScanType::kstrongStructured)
        return PoseScan_S(new kstrongStructuredRadar(pars, radar_msg, T, Tmotion));
    else if(pars.scan_type == cfear)
        return PoseScan_S(new CFEARFeatures(pars, radar_msg, T, Tmotion));
    else if(pars.scan_type == kstrongCart)
        return PoseScan_S(new CartesianRadar(pars, radar_msg, T, Tmotion));

    else return nullptr;
}



}
