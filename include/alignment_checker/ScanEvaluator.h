#pragma once
#include "alignment_checker/AlignmentQuality.h"
#include "alignment_checker/DataHandler.h"
#include "stdio.h"
#include "vector"
#include "alignment_checker/Utils.h"

namespace CorAlignment {
using namespace alignment_checker;

class datapoint
{
public:
  datapoint(const int index, const std::vector<double>& residuals,const std::vector<double>& perturbation, const std::vector<double>& score) : residuals_(residuals), perturbation_(perturbation), score_(score) {}

  std::vector<double> residuals_;
  std::vector<double> perturbation_={0,0,0};
  std::vector<double> score_={0,0,0};
  int index;
  bool  aligned();
};
class scanEvaluator
{
public:


  class parameters
  {
  public:
    parameters() {}

    std::string method;
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
    std::string dataset ="";


  };
  scanEvaluator( dataHandler_U& reader, const parameters& par, const AlignmentQuality::parameters& alignment_par);

  void CreatePerturbations();

  void SaveEvaluation();



private:

  void InputSanityCheck();

  parameters par_;
  //dataHandler_U reader_;
  dataHandler_U reader_;

  std::vector< std::vector<double> > vek_perturbation_;
  std::vector<datapoint> datapoints_;

};

}
