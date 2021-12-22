#!/usr/bin/env python
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


from std_msgs.msg import Float64, Float64MultiArray

class callback_learner:

    def __init__(self, start = 0):
        self.lock = threading.Lock()
        self.value = start
        self.df = pd.DataFrame(columns=['score1', 'score2', 'score3','aligned'])
        self.sub = rospy.Subscriber("/coral_training", Float64MultiArray, self.callback)
        self.mutex = Lock()
        self.thread = threading.Thread(target=self.thread_learning, args=("name",))
        self.thread.daemon = True
        self.thread.start()
    def callback(self,data):
        self.mutex.acquire()
        self.df = self.df.append({'score1' : data.data[1], 'score2' : data.data[2], 'score3' :data.data[3], 'aligned'  :data.data[0]}, ignore_index = True)
        self.mutex.release()


    def thread_learning(self,name):
        #print("Thread starting")
        while True:
            time.sleep(1.0)
            self.mutex.acquire()
            df_copy = self.df.copy()
            self.mutex.release()
            if( df_copy.shape[0] > 5 ):
                accuracy=TrainClassifier(df_copy)
                print("Accuracy: "+str(accuracy)+", Datapoints: "+str(df_copy.shape[0]))
            #print(df_copy)


        print("Thread finishing")

def main(args):
    rospy.init_node('alignment_service', anonymous=True)
    l = callback_learner()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
