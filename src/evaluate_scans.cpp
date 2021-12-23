

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
#include "alignment_checker/ScanType.h"
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
  std::string scantype;

  //Method
  std::string method, dataset;

  scanEvaluator::parameters evalPars;
  AlignmentQuality::parameters qualityPars;
  PoseScan::Parameters scanPars;
  bool test;


  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("input-file-path",po::value<std::string>(&filepath)->default_value(std::string("/home/daniel/.ros/maps/offarla-2012_gt=1_submap=0_sizexy=250_Z=15_intrchR=8_compR=0_res=0.4_maxSensd=130_keyF=1_d=0.3_deg=3_alpha=0_dl=0_xyzir=0_mpsu=0_mnpfg=6kmnp0_sensorpose_est.txt")),"file of path containing pose files")
      ("cloud-dir", po::value<std::string>(&directory_clouds)->default_value(std::string("/home/daniel/ros/mapping_ws/src/graph_map/graph_map/scripts")),"directory of clouds")
      ("cloud-prefix",po::value<std::string>(&cloud_prefix)->default_value(std::string("cloud_")),"prefix of cloud names")
      ("output-dir",po::value<std::string>(&evalPars.output_directory)->default_value(std::string("cloud_")),"output directory")
      ("output-eval-metafile",po::value<std::string>(&evalPars.output_meta_file)->default_value(std::string("params.txt")),"output meta file name")
      ("output-eval-file",po::value<std::string>(&evalPars.output_eval_file)->default_value(std::string("eval.txt")),"output file name")
      ("method",po::value<std::string>(&qualityPars.method)->default_value(std::string("")),"evaluation method")
      ("scan-type",po::value<std::string>(&scantype)->default_value(std::string("kstrong")),"evaluation method")
      ("kstrong",po::value<int>(&scanPars.kstrong)->default_value(12),"")
      ("zmin",po::value<double>(&scanPars.z_min)->default_value(60),"")
      //("eval-yaml",po::value<std::string>(&eval_yaml)->default_value(std::string("")),"yaml file")
      ("eval-name",po::value<std::string>(&evalPars.eval_name)->default_value(std::string("eval")),"evaluation method")
      ("data-set",po::value<std::string>(&evalPars.dataset)->default_value(std::string("dataset")),"filename")
      ("sequence",po::value<std::string>(&evalPars.sequence)->default_value(std::string("")),"filename")
      ("run-test", "dont open real data, use mockup instead")
      ("radius-association",po::value<double>(&qualityPars.radius)->default_value(0.2),"radius used when searching nearest point (in kdtree radius search)")
      ("range-error",po::value<double>(&evalPars.range_error)->default_value(0.1),"Misalignment error - position")
      ("theta-range",po::value<double>(&evalPars.theta_range)->default_value(2*M_PI),"Misalignment error - position")
      ("offset-rotation-steps",po::value<int>(&evalPars.offset_rotation_steps)->default_value(4),"Misalignment error - position")
      ("theta-error",po::value<double>(&evalPars.theta_error)->default_value(0.0),"Misalignment error - angular")
      ("index-first-scan",po::value<int>(&index_first_scan)->default_value(0),"index of first scan")
      ("visualization",po::value<bool>(&evalPars.visualize)->default_value(true),"flag indicating whether the visualization should be enabled");


  //Boolean parameres are read through notifiers
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << "\n";
    return 0;
  }
  scanPars.scan_type = Str2Scan(scantype);

  cout<<"----------------\nEvaluation\n----------------"<<endl;
  cout<<evalPars.ToString()<<endl;
  cout<<qualityPars.ToString()<<endl;
  cout<<scanPars.ToString()<<endl;
  dataHandler_U fileHandler = nullptr;

  if(vm.count("run-test"))
  {
    cout<<"Using mockup data"<<endl;
    fileHandler = std::make_unique<MockupHandler>();
  }
  else
    fileHandler = std::make_unique<RadarRosbagHandler>(filepath, scanPars);


  scanEvaluator eval(fileHandler, evalPars, qualityPars);
  cout<<"----------------\nFinished\n----------------"<<endl;


  return 0;
}



