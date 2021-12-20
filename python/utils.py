# This Python file uses the following encoding: utf-8
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
from sklearn import metrics
import os
def PrintConfusionMatric(cnf_matrix, directory):

    if not os.path.exists(directory):
        os.mkdir(directory)

    class_names=[0,1] # name  of classes
    fig, ax = plt.subplots()
    tick_marks = np.arange(len(class_names))
    plt.xticks(tick_marks, class_names)
    plt.yticks(tick_marks, class_names)
    # create heatmap
    sns.heatmap(pd.DataFrame(cnf_matrix), annot=True,fmt='g') #cmap="YlGnBu"
    ax.xaxis.set_label_position("top")
    plt.tight_layout()
    plt.title('Confusion matrix', y=1.1)
    plt.ylabel('Actual label')
    plt.xlabel('Predicted label')
    path = os.path.join(directory,"confusion")
    plt.savefig(path+".png")
    plt.savefig(path+".pdf", bbox_inches='tight')
    plt.show()
    return None

def PrintROC(logreg,X_test,y_test,directory):
    if not os.path.exists(directory):
        os.mkdir(directory)
    y_pred_proba = logreg.predict_proba(X_test)[::,1]
    fpr, tpr, _ = metrics.roc_curve(y_test,  y_pred_proba)
    auc = metrics.roc_auc_score(y_test, y_pred_proba)
    plt.plot(fpr,tpr,label="data 1, auc="+str(auc))
    plt.legend(loc=4)

    path = os.path.join(directory,"ROC")
    plt.savefig(path+".png")
    plt.savefig(path+".pdf", bbox_inches='tight')
    plt.show()
    return None