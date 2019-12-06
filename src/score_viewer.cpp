

#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>


#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>


#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <time.h>
#include <fstream>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/program_options.hpp>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/publisher.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/filter.h"
//#include "pcl/visualization/cloud_viewer.h"
#include "pcl/point_cloud.h"
#include "alignment_checker/scancomparsion.h"
#include "pcl/common/transforms.h"
#include "alignment_checker/alignmenttester.h"
#include "alignment_checker/utils.h"
#include "alignment_checker/viewer.h"
namespace po = boost::program_options;
namespace ac = alignment_checker;
using std::cout;
using std::endl;
using std::string;





int main(int argc, char **argv)
{


  string filepath, directory_clouds, output_dir;
  double ground_height, radius;
  bool segment_ground, visualize, identity;

  ros::init(argc, argv, "score_viewer");
  ros::NodeHandle param("~");
  ros::Publisher cloud_pub;
  cloud_pub =param.advertise<pcl::PointCloud<pcl::PointXYZ>>("/points2", 1);
  po::options_description desc("Allowed options");
  std::string cloud_prefix;
  desc.add_options()
      ("help", "produce help message")
      ("pose-file-path",po::value<std::string>(&filepath)->default_value(std::string("/home/daniel/.ros/maps/offarla-2012_gt=1_submap=0_sizexy=250_Z=15_intrchR=8_compR=0_res=0.4_maxSensd=130_keyF=1_d=0.3_deg=3_alpha=0_dl=0_xyzir=0_mpsu=0_mnpfg=6kmnp0_sensorpose_est.txt")),"file of path containing pose files")
      ("cloud-dir", po::value<std::string>(&directory_clouds)->default_value(std::string("/home/daniel/ros/mapping_ws/src/graph_map/graph_map/scripts")),"directory of clouds")
      ("cloud-prefix",po::value<std::string>(&cloud_prefix)->default_value(std::string("cloud_")),"prefix of cloud names")
      ("output-dir",po::value<std::string>(&output_dir)->default_value(std::string("")),"output directory")
      ("ground-max-height",po::value<double>(&ground_height)->default_value(0.25),"ground height to used for filter gorund")
      ("radius", po::value<double>(&radius)->default_value(0.5),"radius of a voxel")
      ("visualize","visualize pointclouds")
      ("identity","identity affinity")
      ("segment-ground","segment gorund at a certain hight");

  //Boolean parameres are read through notifiers
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << "\n";
    return false;
  }
  if(filepath.size()==0){
    std::cerr<<"File path empty"<<std::endl;
    exit(0);
  }
  segment_ground = vm.count("segment-ground");
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > poses;
  ac::ReadPosesFromFile(filepath, poses);

  if(poses.size()==0){
    std::cerr<<"pose vector empty"<<std::endl;
    exit(0);
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds, filtered_clouds;

  ac::ReadCloudsFromFile(directory_clouds, cloud_prefix, clouds);
  if(poses.size()!=clouds.size()){
    std::cerr<<"Input data is of wrong size: clouds="<<clouds.size()<<", poses="<<poses.size()<<endl;
    exit(0);
  }
  else{
    cout<<"Found "<<poses.size()<<" poses and point clouds"<<endl;
  }

  cout<<"first outside: "<<clouds[0]->size()<<endl;
  filtered_clouds = clouds;

  ac::viewer(clouds, poses);





  return 0;
}



