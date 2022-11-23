#!/usr/bin/python3
import json
import os
import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from fra333_hw2 import endEffectorJacobianHW2, checkSingularityHW2, computeEffortHW2
from HW2_utils import FKHW2


def import_file():
    for file in os.listdir(os.getcwd()):
        if "fra333" in file:
            print("source code file: " + file)
            exec("from {} import endEffectorJacobianHW2, checkSingularityHW2, computeEffortHW2".format(file[:-3]), globals())
            
def load_json():
    data_json = {}
    with open('testCase.json','r') as f:
        data_json = json.load(f)
    return data_json

def scoreCalculate(data):
    import_file()
    score_question1 = 0
    for i in range(10):
        q = data["question_q1"]["{}".format(i)]["q"]
        ts_j_e = np.array(data["question_q1"]["{}".format(i)]["j_e"])
        if endEffectorJacobianHW2(q) is None:
            continue
        j_e = np.array(endEffectorJacobianHW2(q))
        if (abs(j_e - ts_j_e) < 0.001).all():
            score_question1 += 1
    print("Question 1 score: {0:.2f}%".format(score_question1*10))

    score_question2 = 0
    for i in range(10):
        q = data["question_q2"]["{}".format(i)]["q"]
        ts_flag = np.array(data["question_q2"]["{}".format(i)]["flag"])
        if checkSingularityHW2(q) is None:
            continue
        flag = np.array(checkSingularityHW2(q))
        if (ts_flag and flag) or (~ts_flag and ~flag) == True:
            score_question2 += 1
    print("Question 2 score: {0:.2f}%".format(score_question2*10))
    
    score_question3 = 0
    for i in range(10):
        q = data["question_q3"]["{}".format(i)]["q"]
        w = data["question_q3"]["{}".format(i)]["w"]
        ts_tau = np.array(data["question_q3"]["{}".format(i)]["tau"])
        if computeEffortHW2(q,w) is None:
            continue
        tau = np.array(computeEffortHW2(q,w))
        if tau is None:
            continue
        if (abs(ts_tau - tau) < 0.001).all():
            score_question3 += 1
    print("Question 3 score: {0:.2f}%".format(score_question3*10))
    
    return (score_question1 + score_question2 + score_question3)*10/3

if __name__ == "__main__":
    data = load_json()
    print("Total Score: {0:.2f}%".format(scoreCalculate(data)))