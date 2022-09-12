# Testing for [alignmentinterface](../include/alignment_checker/alignmentinterface.h) functionality

In [python_classifier_interface_tests.cpp](scan_learning_interface_tests.cpp) are tests for __PythonClassifierInterface__.

- logisticRegressionPredictTest
    - Testing __predict()__ using Logistic Regression with some dummy training data.
- logisticRegressionPredictProbaTest
    - Testing __predict_proba()__ using Logistic Regression with some dummy training data.
- decisionTreePredictTest
    - Testing __predict()__ using Decision Trees with some dummy training data.
- decisionTreePredictProbaTest
    - Testing __predict_proba()__ using Decision Trees with some dummy training data.
- saveAndLoadDataTest
    - Testing __SaveData()__ and __LoadData()__ by first saving data (*test_data.txt*) into [/data](../data), and then loading from saved file. 


In [scan_learning_interface_tests.cpp](scan_learning_interface_tests.cpp) are tests for __ScanLearningInterface__.

- predAlignmentTest
    - Testing __PredAlignment()__ using Logistic Regression with training data loaded from [/data/simple_graph.sgh](../data/simple_graph.sgh).
- saveAndLoadDataTest
    - Testing __SaveData()__ and __LoadData()__ by first saving data (*CFEAR.txt* & *CorAl.txt*) into [/data](../data), and then loading from saved files. 



Make tests 
```
catkin_make tests
```

Run __PythonClassifierInterface__ test
```
rosrun alignment_checker alignment_checker_python_classifier_test
```

Run __ScanLearningInterface__ test
```
rosrun alignment_checker alignment_checker_scan_learning_test
```