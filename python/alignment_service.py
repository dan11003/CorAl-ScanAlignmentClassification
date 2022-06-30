#!/usr/bin/env python
from turtle import update
from tomlkit import string
import rospy

import roslib;
import numpy as np
import pandas as pd
import threading
import time
import logging
import sys
from threading import Thread, Lock
from utils import *

import os.path
from std_msgs.msg import Float64, Float64MultiArray
from alignment_checker.srv import *
import argparse

class callback_learner:

    def __init__(self, df_td : pd.DataFrame):
        self.lock = threading.Lock()
        self.df = df_td
        self.model = None
        self.update_model()
        self.sub = rospy.Subscriber("/coral_training", Float64MultiArray, self.callback)
        self.srv = rospy.Service('alignment_service', AlignmentData, self.callback_service)
        self.mutex = Lock()
        self.thread = threading.Thread(target=self.thread_learning, args=("name",))
        self.thread.daemon = True
        
        
    def thread_learning(self,name):
        print("Thread starting")
        while True:
            time.sleep(1.0)
            self.mutex.acquire()
            df_copy = self.df.copy()
            self.mutex.release()
            if( df_copy.shape[0] > 5 ):
                accuracy,cnf_matrix=TrainClassifier(df_copy)
                self.update_model()
                print("Accuracy: "+str(accuracy)+", Datapoints: "+str(df_copy.shape[0]))
                print("confusion: +\n"+str(cnf_matrix))
            #print(df_copy)
        print("Thread finishing")

    def callback(self,data):
        self.mutex.acquire()
        # self.df = self.df.append({'score1' : data.data[1], 'score2' : data.data[2], 'score3' :data.data[3], 'aligned'  :data.data[0]}, ignore_index = True)
        data_dict = [{'score1' : data.data[1], 'score2' : data.data[2], 'score3' :data.data[3], 'aligned'  :data.data[0]}]
        self.df = pd.concat([self.df, pd.DataFrame(data_dict)], ignore_index=True)
        self.mutex.release()


    def update_model(self):
        if( self.df.shape[0] > 5 ):
            df_copy = self.df.copy()
            self.model = get_trained_classifier(df_copy)


    def callback_service(self,req):
        if (self.model is not None and len(req.score) == 4):
            req_dict = {'score1' : req.score[1], 'score2' : req.score[2], 'score3' : req.score[3], 'aligned' : req.score[0]}
            req_df = pd.DataFrame(req_dict, index=[0])
            col_names=['score1','score2','score3']
            X = req_df[col_names]
            self.mutex.acquire()
            # y_pred = self.model.predict(X)
            y_prob = self.model.predict_proba(X)
            # print(y_prob)
            self.mutex.release()
            return AlignmentDataResponse(y_prob[0][1])
        else:
            print("Wrong service message...")

def get_df_from_csv(file_path : string) -> pd.DataFrame:
    if os.path.exists(file_path):
        print("Loaded data from", file_path)
        return pd.read_csv(file_path)
    else:
        print("Training data file not found...")
        return pd.DataFrame(columns=['score1', 'score2', 'score3','aligned'])

def get_trained_classifier(df : pd.DataFrame) -> LogisticRegression:
    col_names=['score1','score2','score3']
    #df = pd.read_csv(file_path)
    X = df[col_names]
    y = df.aligned
    y=y.astype('int')
    logreg = LogisticRegression(class_weight='balanced')
    logreg.fit(X,y)
    return logreg


def main(args):
    # Parse input arguments
    parser = argparse.ArgumentParser(description='Get script arguments.')
    parser.add_argument('--training_data', default="", type=str, help='training data csv file')
    args = parser.parse_args()

    # If using previous training data
    if (args.training_data):
        df_training_data = get_df_from_csv(args.training_data)
    else:
        df_training_data = pd.DataFrame(columns=['score1', 'score2', 'score3','aligned'])

    rospy.init_node('alignment_service', anonymous=True)
    l = callback_learner(df_td=df_training_data)
    # l.thread.start()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
