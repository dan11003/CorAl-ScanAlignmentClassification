

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
#include "pcl/common/transforms.h"

#include "alignment_checker/Utils.h"
#include "tf_conversions/tf_eigen.h"
//#include "yaml-cpp/yaml.h"
#include "alignment_checker/DataHandler.h"
#include "alignment_checker/ScanEvaluator.h"
#include "alignment_checker/AlignmentQuality.h"
#include "memory"

namespace po = boost::program_options;
namespace ac=alignment_checker;
using std::cout;
using std::endl;
using std::string;

using namespace CorAlignment;



int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate");

  // Input output
  std::string filepath, directory_clouds, cloud_prefix;
  int index_first_scan;
  std::string output_dir, output_file_name;

  //Method
  std::string method, dataset;

  scanEvaluator::parameters evalPars;
  AlignmentQuality::parameters qualityPars;


  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("input-file-path",po::value<std::string>(&filepath)->default_value(std::string("/home/daniel/.ros/maps/offarla-2012_gt=1_submap=0_sizexy=250_Z=15_intrchR=8_compR=0_res=0.4_maxSensd=130_keyF=1_d=0.3_deg=3_alpha=0_dl=0_xyzir=0_mpsu=0_mnpfg=6kmnp0_sensorpose_est.txt")),"file of path containing pose files")
      ("cloud-dir", po::value<std::string>(&directory_clouds)->default_value(std::string("/home/daniel/ros/mapping_ws/src/graph_map/graph_map/scripts")),"directory of clouds")
      ("cloud-prefix",po::value<std::string>(&cloud_prefix)->default_value(std::string("cloud_")),"prefix of cloud names")
      ("output-dir",po::value<std::string>(&evalPars.output_directory)->default_value(std::string("cloud_")),"output directory")
      ("output-eval-metafile",po::value<std::string>(&evalPars.output_meta_file)->default_value(std::string("params.txt")),"output meta file name")
      ("output-eval-file",po::value<std::string>(&evalPars.output_eval_file)->default_value(std::string("eval.txt")),"output file name")
      ("method",po::value<std::string>(&evalPars.method)->default_value(std::string("entropy")),"evaluation method")
      //("eval-yaml",po::value<std::string>(&eval_yaml)->default_value(std::string("")),"yaml file")
      ("eval-name",po::value<std::string>(&evalPars.eval_name)->default_value(std::string("eval")),"evaluation method")
      ("data-set",po::value<std::string>(&evalPars.dataset)->default_value(std::string("dataset")),"filename")
      ("sequence",po::value<std::string>(&evalPars.sequence)->default_value(std::string("")),"filename")
      ("index-first-scan",po::value<int>(&index_first_scan)->default_value(0),"index of first scan");
  //("radius", po::value<double>(&radius)->default_value(0.5),"radius of a voxel");


  //Boolean parameres are read through notifiers
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << "\n";
    return 0;
  }
  if(filepath.size()==0){
    std::cerr<<"File path empty"<<std::endl;
    exit(0);
  }


  cout<<"Evaluation"<<endl;
  cout<<evalPars.ToString()<<endl;
  cout<<qualityPars.ToString()<<endl;

  dataHandler_U fileHandler = std::make_unique<RadarRosbagHandler>(filepath);
  scanEvaluator eval(fileHandler, evalPars, qualityPars);
  cout<<"finished"<<endl;


  return 0;
}



