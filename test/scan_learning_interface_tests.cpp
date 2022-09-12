#include "alignment_checker/alignmentinterface.h"
#include "cfear_radarodometry/types.h"
#include <gtest/gtest.h>
#include <ros/package.h>

/* ScanLearninigInterface tests */

class ScanLearninigInterfaceTest : public ::testing::Test {
 protected:

  void SetUp() override {

    const std::string simple_graph_path = ros::package::getPath("alignment_checker") + "/data/simple_graph.sgh";

    CFEAR_Radarodometry::simple_graph sg;
    CFEAR_Radarodometry::LoadSimpleGraph(simple_graph_path, sg);

    for(int i = 0; i < sg.size()-1; i++){
      prev = current;
      CFEAR_Radarodometry::RadarScan scan = sg.at(i).first;
			current.T = scan.GetPose();
      current.cld = scan.cloud_nopeaks_;
      current.cldPeaks = scan.cloud_peaks_;
      current.CFEAR = scan.cloud_normal_;

      scan_learner.AddTrainingData(current);
    }
    
  }
  CorAlignment::ScanLearningInterface::s_scan current, prev;
  CorAlignment::ScanLearningInterface scan_learner;
};


TEST_F(ScanLearninigInterfaceTest, logisticRegressionPPredAlignmentTest){
  scan_learner.FitModels("LogisticRegression");

  std::map<std::string,double> quality;
  scan_learner.PredAlignment(current, prev, quality);

  const double coral_pred = quality["CorAl"];
  const double cfear_pred = quality["CFEAR"];

  EXPECT_TRUE((quality["CorAl"] >= 0) && (quality["CorAl"] <= 1));
  EXPECT_TRUE((quality["CFEAR"] >= 0) && (quality["CFEAR"] <= 1));
}

TEST_F(ScanLearninigInterfaceTest, decisionTreePPredAlignmentTest){
  scan_learner.FitModels("DecisionTreeClassifier");
  
  std::map<std::string,double> quality;
  scan_learner.PredAlignment(current, prev, quality);

  const double coral_pred = quality["CorAl"];
  const double cfear_pred = quality["CFEAR"];

  EXPECT_TRUE((quality["CorAl"] >= 0) && (quality["CorAl"] <= 1));
  EXPECT_TRUE((quality["CFEAR"] >= 0) && (quality["CFEAR"] <= 1));
}

TEST_F(ScanLearninigInterfaceTest, saveAndLoadDataTest)
{
  const std::string data_path = ros::package::getPath("alignment_checker") + "/data/";

  scan_learner.SaveData(data_path);

  CorAlignment::ScanLearningInterface scan_learner_loaded;
  scan_learner_loaded.LoadData(data_path);

  scan_learner.FitModels("LogisticRegression");
  scan_learner_loaded.FitModels("LogisticRegression");
  
  std::map<std::string,double> quality;
  scan_learner.PredAlignment(current, prev, quality);

  std::map<std::string,double> quality_loaded;
  scan_learner_loaded.PredAlignment(current, prev, quality_loaded);

  EXPECT_FLOAT_EQ(quality_loaded["CorAl"], quality["CorAl"]);
  EXPECT_FLOAT_EQ(quality_loaded["CFEAR"], quality["CFEAR"]);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_scan_learner_interface");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}