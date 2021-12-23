#pragma once
#include "alignment_checker/AlignmentQuality.h"
#include "alignment_checker/DataHandler.h"
#include "stdio.h"
#include "vector"
#include "alignment_checker/Utils.h"
#include <sstream>      // std::stringstream


namespace CorAlignment {
using namespace alignment_checker;

class datapoint
{
public:
  datapoint(const int index, const std::vector<double>& residuals,const std::vector<double>& perturbation, const std::vector<double>& score, std::shared_ptr<PoseScan>& ref, std::shared_ptr<PoseScan>& src) : residuals_(residuals), perturbation_(perturbation), score_(score), index_(index) {
    distance_ = (ref->GetAffine().translation()-src->GetAffine().translation()).norm();
    src_id_ = src->pose_id;
    ref_id_ = ref->pose_id;
  }
  static std::string HeaderToString(){
    std::stringstream ss;
    ss<<"index,ref_id,src_id,distance,score1,score2,score3,aligned,error x,error y,error theta";
    return ss.str();
  }
  std::string ToString(){
    std::stringstream ss;
    ss<<index_<<","<<ref_id_<<","<<src_id_<<","<<distance_<<","<<score_[0]<<","<<score_[1]<<","<<score_[2]<<","<<aligned()<<","<<perturbation_[0]<<","<<perturbation_[1]<<","<<perturbation_[2];
    return ss.str();

  }

  double distance_;
  std::vector<double> residuals_;
  std::vector<double> perturbation_={0,0,0};
  std::vector<double> score_={0,0,0};
  int index_, ref_id_, src_id_;
  bool  aligned();
};
class scanEvaluator
{
public:


  class parameters
  {
  public:
    parameters() {}

    int scan_spacing = 1;

    // offset parameters
    // For inducing cartesian position error y=r*sin(t) t depends on theta range (coverage) and offset_rotation_steps (resolution)
    // Ideally, theta_range is 360 deg and theta_range -> infinity, however we can approximate this by looking at a 1/4 of the full sweep e.g. in offset_rotation_steps=2steps
    double range_error = 0.5;
    double theta_range = 2*M_PI/4.0;
    int offset_rotation_steps = 2;

    //For inducing rotation error
    double theta_error = 0.57*M_PI/180.0;

    std::string output_directory = "";
    std::string output_meta_file = "params.txt";
    std::string output_eval_file = "eval.txt";
    std::string output_residual_prefix = "residuals_";
    std::string eval_name ="noname";
    std::string dataset = "", sequence = "";

    //Visualization
    bool visualize = true;

    std::string ToString(){
      std::ostringstream stringStream;
      stringStream << "output_directory, "<<output_directory<<endl;
      stringStream << "output_meta_file, "<<output_meta_file<<endl;
      stringStream << "output_eval_file, "<<output_eval_file<<endl;
      stringStream << "output_residual_prefix, "<<output_residual_prefix<<endl;
      stringStream << "eval_name, "<<eval_name<<endl;
      stringStream << "dataset, "<<std::boolalpha<<dataset<<endl;
      stringStream << "scan_spacing, "<<scan_spacing<<endl;
      stringStream << "range_error, "<<range_error<<endl;
      stringStream << "theta_range, "<<theta_range<<endl;
      stringStream << "offset_rotation_steps, "<<offset_rotation_steps<<endl;
      stringStream << "theta_error, "<<theta_error<<endl;
      return stringStream.str();
    }


  };
  scanEvaluator( dataHandler_U& reader, const parameters& eval_par, const AlignmentQuality::parameters& alignment_par);

  void CreatePerturbations();

  void SaveEvaluation();



private:

  void InputSanityCheck();

  parameters par_;
  AlignmentQuality::parameters quality_par_;
  //dataHandler_U reader_;
  dataHandler_U reader_;

  std::vector< std::vector<double> > vek_perturbation_;
  std::vector<datapoint> datapoints_;

};

}
