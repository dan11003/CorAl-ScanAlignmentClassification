%%Load data
clc
clear all
close all


T=[];
base='/home/iliad/workspace/iliad_ws/src/graph_map/graph_map/data/test_data/orkla.txt';

FullDataset= { 'orkla'};


sufix='';
T= readtable(base);


%TrainingData=Td(strcmp(Td.Method,methods{i}),:);
[trainedClassifier, validationAccuracy] = trainClassifier(T);
