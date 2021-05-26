
//#include "graph_map/graph_map_fuser.h"
#include <ros/ros.h>

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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include <time.h>
#include <fstream>
#include <cstdio>

#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "robust_mapping_custom_msgs/registration.h"
#include "robust_mapping_custom_msgs/n_registration.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include "alignment_checker/utils.h"
#include "alignment_checker/scan.h"
#include "alignment_checker/scancomparsion.h"
#include "opencv2/core.hpp"
#include <opencv2/ml.hpp>
#include <opencv2/highgui.hpp>
#include "map"
#include "mlpack/methods/logistic_regression/logistic_regression.hpp"
#include "mlpack/methods/linear_regression/linear_regression.hpp"
#include "mlpack/core.hpp"
#include <armadillo>
#include<iostream>
#include<mlpack/core.hpp>
#include "ros/timer.h"
#include "alignment_checker/scan.h"
#include "string"
#include <iostream>
#include <iomanip>
#include <sstream>


/** \brief A ROS node which implements scan matching based on ndt
 * \author Daniel adolfsson
 *
 */
using namespace cv;
using namespace cv::ml;

using std::string;
using std::cout;
using std::cerr;
using std::endl;
using namespace mlpack;
using namespace  arma;
using namespace mlpack::regression;

namespace ac=alignment_checker;

class CorAlServer{
public:
  CorAlServer():nh_("~"){

    double swell, max_swell_dist;
    std::string path;
    nh_.param<std::string>("alignment_quality_service", alignment_quality_service, "/alignment_quality_service");
    nh_.param<std::string>("world_frame", world_id_, "map");
    nh_.param<std::string>("training_data_path", path, "");
    nh_.param<double>("resolution", radius_, 0.4);
    nh_.param<double>("max_swell_distance", max_swell_dist ,50);
    nh_.param<double>("max_swell", swell, 2.5);
    nh_.param<bool>("downsample", downsample_, true);
    nh_.param<double>("rejection", rejection_, 0.1);
    nh_.param<double>("decision_th", decision_th, 0.5);
    nh_.param<double>("Tplot", Tplot, 2);


    nh_.param<bool>("visualize", visualize, true);
    ac::ScanType::max_swell = swell;
    ac::ScanType::max_swell_dist = max_swell_dist;
    cout<<"Loading training data from: "<<path<<endl;
    LoadCsv(path);
    int idx_train_end = Table[1].size()-1;

    // cout<<"table: "<<Table[1].size()<<", "<<Table[2].size()<<", "<<Table[9].size()<<endl;
    arma::Mat<double> predictors( GetPredictors(0, idx_train_end) );
    arma::Row<size_t> labels(GetLabels(0, idx_train_end) );
    cout<<"Training logistic classifier on :"<<idx_train_end+1<<" samples"<<endl;
    l = new mlpack::regression::LogisticRegression<arma::Mat<double>>(predictors, labels);
    cout<<"Training accuracy: "<<l->ComputeAccuracy(predictors, labels, decision_th)<<endl;

    service = nh_.advertiseService(alignment_quality_service, &CorAlServer::CorAlService, this);
    timer_plot = nh_.createTimer(ros::Duration(Tplot), &CorAlServer::MissalignedCloudsVisualizer, this);


  }
  void Publish(const std::string& topic, pcl::PointCloud<pcl::PointXYZ> &cloud, std::string frame_id, const ros::Time &t){
    cloud.header.frame_id = frame_id;
    pcl_conversions::toPCL(t, cloud.header.stamp);
    alignment_checker::PublishCloud(topic, cloud);
  }
  void Publish(const std::string& topic, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string frame_id, const ros::Time &t){
    cloud->header.frame_id = frame_id;
    pcl_conversions::toPCL(t, cloud->header.stamp);
    alignment_checker::PublishCloud(topic, *cloud);
  }

  bool LoadCsv(const std::string& path){
    string line;
    int index =0;
    std::ifstream myfile (path);
    int nr_line = 0;

    std::vector< std::vector< std::string > > content;
    if (myfile.is_open()){
      while ( getline (myfile,line) ){
        std::vector<double> pose_compoments;
        std::vector<std::string> line_splited;
        boost::split( line_splited, line, boost::is_any_of(",") );
        if(nr_line == 0)
          header = line_splited;
        else
          content.push_back(line_splited);
        for (std::vector<std::string>::iterator i = line_splited.begin(); i != line_splited.end(); ++i)
          Table[std::distance(line_splited.begin(), i)].push_back(*i);

      }
      myfile.close();
      if(Table.size()>0)
        return true;
    }
    else{
      std::cout<<"couldn't open file"<<endl;
    }
    return false;
  }
  arma::Mat<double> GetPredictors(int start_idx, int end_idx){

    int count = end_idx-start_idx+1;
    arma::Mat<double> predictors(2, count);
    for (int i=1 ; i<=2 ; i++) {
      for (int j=start_idx ; j <= end_idx ; j++){
        predictors(i-1, j) = std::atof( Table[i][j].c_str());
        //cout<<predictors(i, j)<<endl;
      }
    }
    return predictors;
  }
  arma::Row<size_t> GetLabels(int start_idx, int end_idx){

    int count = end_idx-start_idx+1;
    arma::Row<size_t> labels(count);
    if(start_idx<0 || end_idx > Table[9].size()-1){
      cerr<<"Error reading matrix"<<endl;
    }

    for (int j=start_idx ; j <= end_idx ; j++){
      labels(j) = std::atoi( Table[9][j].c_str());
      //cout<<"j: "<<j<<", label: "<<labels(j)<<endl;
    }
    return labels;
  }

  void Load(){

    int idx_train_end = Table[1].size()*0.5;
    int idx_test_start =  idx_train_end +1;
    int idx_test_end =  Table[1].size()-1;
    int count_eval = idx_test_end - idx_test_start + 1;
    cout<<"idx_train_end="<<idx_train_end<<", idx_test_start="<<idx_test_start<<", idx_test_end="<<idx_test_end<<endl;
    // cout<<"table: "<<Table[1].size()<<", "<<Table[2].size()<<", "<<Table[9].size()<<endl;
    arma::Mat<double> predictors( GetPredictors(0, idx_train_end));
    arma::Row<size_t> labels(GetLabels(0, idx_train_end));
    //cout<<"get: "<<idx_test_start<<", "<<idx_test_end<<", out of "<<Table[1].size()<<endl;
    arma::Mat<double> eval_predictors( GetPredictors(0, idx_test_end));
    arma::Row<size_t> eval_labels(GetLabels(0, idx_test_end));



    /*cout<<"predictors: "<<predictors<<endl;
    cout<<"labels: "<<labels<<endl;
    cout<<"eval_predictors: "<<eval_predictors<<endl;
    cout<<"eval_labels: "<<eval_labels<<endl;*/

    //mlpack::regression::LogisticRegression<arma::Mat<double>> l(predictors, labels);
    //double d = l.ComputeAccuracy(eval_predictors, eval_labels);

    double sum = 0;
    double N = 0;
    for( int j=0 ; j<count_eval  ; j++){
      arma::Mat<double> online_pred(2,1);
      arma::Mat<double> p_response(1,1);
      online_pred(0,0) = eval_predictors(0,j);
      online_pred(1,0) = eval_predictors(1,j);
      arma::Row<size_t> online_label({eval_labels(j)});
      arma::Row<double> pred({eval_predictors(0,j), eval_predictors(1,j)});
      //size_t y_pred = l->Classify(pred, decision_th);
      l->Classify(online_pred, p_response);
      cout<<"Response: "<<p_response(0,0)<<" ---  LABEL: "<<eval_labels(j)<<endl;

      //l->Classify(pred, decision_th);
      //cout<<"ypred: "<<y_pred<<", actual: "<<eval_labels(j)<<endl;

      //    cout<<sum<<" Accuracy: "<<sum/N<<", N: "<<N<<endl;
      //arma::Mat<double> data; // The dataset itself.
      //arma::Row<size_t> responses; // T
    }
    //mlpack::regression::LogisticRegression<arma::Mat<double>> reg(data, responses);
    //std::string filename="/home/iliad/workspace/iliad_ws/src/graph_map/graph_map/data/test_data/lr_mode.bin";
    //data::Load(filename, "lr_model", reg);

  }
  bool CorAlService(robust_mapping_custom_msgs::registration::Request& request, robust_mapping_custom_msgs::registration::Response& response){

    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(request.cloudsrc, *src);
    pcl::fromROSMsg(request.cloudtarget, *target);
    //cout<<"src: "<<src->points.size()<<endl;
    //cout<<"target: "<<target->points.size()<<endl;
    Eigen::Affine3d Tsrc, Ttar;
    tf::poseMsgToEigen(request.Tsrc, Tsrc);
    tf::poseMsgToEigen(request.Ttarget, Ttar);
    src->sensor_origin_(0) = Tsrc.translation()(0);
    src->sensor_origin_(1) = Tsrc.translation()(1);
    src->sensor_origin_(2) = Tsrc.translation()(2);

    target->sensor_origin_(0) = Ttar.translation()(0);
    target->sensor_origin_(1) = Ttar.translation()(1);
    target->sensor_origin_(2) = Ttar.translation()(2);


    ac::ScanComparsion comp(src, target, radius_, downsample_, ac::entropy, rejection_);

    ros::Time t = ros::Time::now();
    Publish("coral_target", *target, world_id_, t);
    Publish("coral_src", *src, world_id_, t);
    response.quality=1;

    bool status = false;
    double aligned_merged = 0, aligned_sep = 0;
    comp.GetAlignmentQualityExtended(status, aligned_merged, aligned_sep);
    //cout<<"merged: "<<aligned_merged<<" separate: "<<aligned_sep<<", diff: "<<aligned_merged-aligned_sep<<endl;

    /*arma::Mat<double> online_pred(2,1);
    online_pred(0,0) = aligned_merged;
    online_pred(1,0) = aligned_sep;
    std::vector<double> vek{aligned_merged,aligned_sep};*/
    arma::Row<double> pred({aligned_merged,aligned_sep});
    size_t y_pred = l->Classify(pred, decision_th);
    arma::Mat<double> online_pred(2,1);
    online_pred(0,0) = aligned_merged;
    online_pred(1,0) = aligned_sep;
    arma::Mat<double> p_response(1,1);

    l->Classify(online_pred, p_response);
    double p_aligned = 1-p_response(0,0);
    bool aligned = (p_aligned >decision_th);
    cout<<"classify (merged,separate)=("<<aligned_merged<<","<<aligned_sep<<", diff="<<aligned_merged-aligned_sep<<endl;
    std::string s = aligned  ? "ALIGNED" : "NOT ALIGNED";
    cout<<"p( aligned) = "<<s<<endl;
    response.quality = p_aligned;
    //cout<<"src: "<<src->size()<<", tar: "<<target->size()<<", rad: "<<radius_<<", downsample: "<<downsample_<<", ent: "<<rejection_<<", ac::ScanType::max_swell: "<<ac::ScanType::max_swell<<", ac::ScanType::max_swell_dist: "<<ac::ScanType::max_swell_dist<<endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp  = comp.GetMergedDifferential();
    Publish("CorAl_diff", tmp, "/map", ros::Time::now());
    if(! aligned){
      m.lock();
      misaligned.push_back(tmp);
      m.unlock();
    }
    return true;

  }
  void MissalignedCloudsVisualizer(const ros::TimerEvent& event){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cld_all(new pcl::PointCloud<pcl::PointXYZI>());
    m.lock();
    for(int i=0 ; i<misaligned.size() ; i++)
    {
      *cld_all += *(misaligned[i]);
    }
    m.unlock();
    Publish("CorAl_all", cld_all, "/map", ros::Time::now());
  }


  ros::NodeHandle nh_;
  ros::ServiceServer service;
  bool visualize;
  double resolution_;
  std::string alignment_quality_service;
  std::string world_id_;
  bool disable_registration_;
  double radius_, rejection_;
  double Tplot;
  bool downsample_ = false;
  std::vector<std::string> header;
  std::map<int, std::vector<std::string>> Table;
  mlpack::regression::LogisticRegression<arma::Mat<double>> *l;
  double decision_th;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> misaligned;
  ros::Timer timer_plot;
  std::mutex m;



};

void Classify(){

  /*  Ptr<LogisticRegression> lr1 = LogisticRegression::create();
  lr1->setLearningRate(0.001);
  lr1->setIterations(10);
  lr1->setRegularization(LogisticRegression::REG_L2);
  lr1->setTrainMethod(LogisticRegression::BATCH);
  lr1->setMiniBatchSize(1);
  //! [init]
  lr1->train(data_train, ROW_SAMPLE, labels_train);
  cout << "done!" << endl;

  cout << "predicting...";
  Mat responses;
  lr1->predict(data_test, responses);
  cout << "done!" << endl;*/

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "coral_node");
  CorAlServer srv;



  ros::spin();

  return 0;
}



