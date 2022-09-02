#!/usr/bin/env python
# import pandas as pd
import threading
from threading import Thread, Lock
from utils import *
import os.path

class AlignmentClassifier:

    def __init__(self, training_data = ""):
        print("Created AlignmentClassifier")
        self.df = self.get_df_from_csv(training_data)
        self.model = None
        self.update_model()
        self.mutex = Lock()
        
    def update_model(self):
        if( self.df.shape[0] > 5 ):
            df_copy = self.df.copy()
            self.model = get_trained_classifier(df_copy)

    def get_prob(self,q1, q2):
        if (self.model is None):
            print("Model not loaded, or to little training data...")
            return 0

        req_dict = {'score1' : q1, 'score2' : q2, 'score3' : 0, 'aligned' : 0}
        req_df = pd.DataFrame(req_dict, index=[0])
        col_names=['score1','score2','score3']
        X = req_df[col_names]
        self.mutex.acquire()
        y_prob = self.model.predict_proba(X)
        self.mutex.release()
        return y_prob[0][1]

    def get_df_from_csv(self, training_data):
        file_path = os.environ["BAG_LOCATION"] + '/coral_training_data/' + training_data
        if os.path.exists(file_path):
            print("Loaded training data from", file_path)
            return pd.read_csv(file_path)
        else:
            print("Training data file not found...")
            return pd.DataFrame(columns=['score1', 'score2', 'score3','aligned'])

def get_trained_classifier(df):
    col_names=['score1','score2','score3']
    #df = pd.read_csv(file_path)
    X = df[col_names]
    y = df.aligned
    y=y.astype('int')
    logreg = LogisticRegression(class_weight='balanced')
    logreg.fit(X,y)
    y_pred=logreg.predict(X)
    accuracy = metrics.balanced_accuracy_score(y,y_pred)
    cnf_matrix = metrics.confusion_matrix(y, y_pred,normalize=None)
    print("Accuracy: "+str(accuracy)+", Datapoints: "+str(df.shape[0]))
    print("confusion: +\n"+str(cnf_matrix))
    return logreg
