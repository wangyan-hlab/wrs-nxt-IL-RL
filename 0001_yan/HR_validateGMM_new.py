import os
import numpy as np
import matplotlib.pyplot as plt
plt.style.use("seaborn")
from scipy.stats import multivariate_normal as mvn
from sklearn import metrics
from sklearn.mixture import GaussianMixture
from collections import Counter
from sklearn.decomposition import PCA
import random

def writefile(ls, filename, edit="w"):
    this_dir, this_filename = os.path.split(__file__)
    fl = open(os.path.join(this_dir, "document", filename), edit)
    for poses in ls:
        fl.write(str(poses)+'\n')
    fl.close()

def predict(X, n_components, W, M, C):
    """Returns predicted labels using Bayes Rule to
    Calculate the posterior distribution

    Parameters:
    -------------
    X: ?*d numpy array

    Returns:
    ----------
    labels: predicted cluster based on
    highest responsibility gamma.

    """
    labels = np.zeros((X.shape[0], n_components))

    for c in range(n_components):
        labels[:, c] = W[c] * mvn.pdf(X, M[c], C[c])
    labels = labels.argmax(1)
    return labels

def as_num(x):
    y = '{:.10f}'.format(x)  # .10f 保留10位小数
    return y

def readtest(filename,start,end):
    this_dir, this_filename = os.path.split(__file__)
    f = open(os.path.join(this_dir, "document", filename), "r")
    f1 = f.readlines()[start:end]
    para = []
    for string in f1:
        strlist = string.strip(" ").strip("[").replace("]", "").rstrip(" ")
        s = strlist.split()
        # print(s)
        for ss in s:
            if 'E' in ss or 'e' in ss:
                x = as_num(float(ss))
                # print(x)
                para.append(float(x))
            else:
                # print(float(ss))
                para.append(float(ss))
    return para

if __name__ == '__main__':
    ## Low level policy
    # parameters of the GMM (n_components = 4)
    seq = [0, 1, 2, 3]
    M = readtest("GMM_parameters_1_pose.txt", 0, 8)
    # M = readtest("GMM_parameters_2_pose.txt", 0, 8)     # best
    # M = readtest("GMM_parameters_3_pose.txt", 0, 8)
    # M = readtest("GMM_parameters_4_pose.txt", 0, 8)
    # M = readtest("GMM_parameters_1_force.txt", 0, 4)
    # M = readtest("GMM_parameters_2_force.txt", 0, 5)
    # M = readtest("GMM_parameters_3_force.txt", 0, 6)      # best
    # M = readtest("GMM_parameters_4_force.txt", 0, 4)
    m = np.empty(shape=[4, 6])
    m[seq[0]] = np.array(M[:6])
    m[seq[1]] = np.array(M[6:12])
    m[seq[2]] = np.array(M[12:18])
    m[seq[3]] = np.array(M[18:24])
    C = readtest("GMM_parameters_1_pose.txt", 8, 56)
    # C = readtest("GMM_parameters_2_pose.txt", 8, 56)
    # C = readtest("GMM_parameters_3_pose.txt", 8, 56)
    # C = readtest("GMM_parameters_4_pose.txt", 8, 56)
    # C = readtest("GMM_parameters_1_force.txt", 4, 46)
    # C = readtest("GMM_parameters_2_force.txt", 5, 53)
    # C = readtest("GMM_parameters_3_force.txt", 6, 54)
    # C = readtest("GMM_parameters_4_force.txt", 4, 52)
    c = np.empty(shape=[4, 6, 6])
    c[seq[0]] = np.array([C[:6], C[6:6 * 2], C[6 * 2:6 * 3], C[6 * 3:6 * 4], C[6 * 4:6 * 5], C[6 * 5:6 * 6]])
    c[seq[1]] = np.array([C[6 * 6:6 * 7], C[6 * 7:6 * 8], C[6 * 8:6 * 9], C[6 * 9:6 * 10], C[6 * 10:6 * 11], C[6 * 11:6 * 12]])
    c[seq[2]] = np.array([C[6 * 12:6 * 13], C[6 * 13:6 * 14], C[6 * 14:6 * 15], C[6 * 15:6 * 16], C[6 * 16:6 * 17], C[6 * 17:6 * 18]])
    c[seq[3]] = np.array([C[6 * 18:6 * 19], C[6 * 19:6 * 20], C[6 * 20:6 * 21], C[6 * 21:6 * 22], C[6 * 22:6 * 23], C[6 * 23:6 * 24]])
    W = readtest("GMM_parameters_1_pose.txt", 56, 60)
    # W = readtest("GMM_parameters_2_pose.txt", 56, 60)
    # W = readtest("GMM_parameters_3_pose.txt", 56, 60)
    # W = readtest("GMM_parameters_4_pose.txt", 56, 60)
    # W = readtest("GMM_parameters_1_force.txt", 46, 50)
    # W = readtest("GMM_parameters_2_force.txt", 53, 57)
    # W = readtest("GMM_parameters_3_force.txt", 54, 58)
    # W = readtest("GMM_parameters_4_force.txt", 52, 56)
    w = np.empty(shape=4)
    w[seq[0]] = W[0]
    w[seq[1]] = W[1]
    w[seq[2]] = W[2]
    w[seq[3]] = W[3]
    MEAN_L = [m[0], m[1], m[2], m[3]]
    COV_L = [c[0], c[1], c[2], c[3]]
    WEIGHT_L = [w[0], w[1], w[2], w[3]]
    THETA_L = []

    this_dir, this_filename = os.path.split(__file__)
    f = open(os.path.join(this_dir, "document", "Data_low.txt"), "r")
    fl = f.readlines()[2945*0:2945*60]
    # fl = f.readlines()
    # f1 = fl[2945 * 0:2945 * 40]
    # f2 = fl[2945 * 60:2945 * 80]  # [0:294500:50]
    # fl = f1 + f2
    # fl = f.readlines()
    # f1 = fl[2945*0:2945 * 20]
    # f2 = fl[2945*40:2945*80]  #[0:294500:50]
    # fl = f1 + f2
    # fl = f.readlines()[2945*20:2945*80]
    S = []
    A = []
    n = 0
    for string in fl:
        n += 1
        strlist = string.replace("(", "").replace(")", "")
        data = eval(strlist)
        state = []
        for i in range(6):
            state.append(data[i])
        for j in range(18, len(data)):
            state.append(data[j])
        S.append(state)
        action = []
        for k in range(6,12):
        # for k in range(12, 18):
            action.append(data[k])
        A.append(action)
    STATE_L = np.asarray(S)
    ACTION = np.asarray(A)
    a = np.linalg.inv(np.matmul(np.transpose(STATE_L), STATE_L))  # (X'*X)^-1
    theta_l = np.matmul(np.matmul(a, np.transpose(STATE_L)), ACTION)  # (X'*X)^-1*X'*y
    f.close()

    n_components = 4
    print("low level start")
    # # Validation
    f = open(os.path.join(this_dir, "document", "Data_low.txt"), "r")
    fl = f.readlines()[2945*60:2945*80]
    # fl = f.readlines()[2945 * 40:2945 * 60]
    # fl = f.readlines()[2945 * 20:2945 * 40]
    # fl = f.readlines()[2945 * 0:2945 * 20]
    S = []
    A = []
    n = 0
    for string in fl:
        n += 1
        strlist = string.replace("(", "").replace(")", "")
        data = eval(strlist)
        state = []
        for i in range(6):
            state.append(data[i])
        for j in range(18, len(data)):
            state.append(data[j])
        S.append(state)
        action = []
        for k in range(6, 12):
        # for k in range(12, 18):
            action.append(data[k])
        A.append(action)
    STATE = np.asarray(S)
    ACTION = np.asarray(A)

    for m in MEAN_L:
        mus = np.tile(m, [len(ACTION), 1])
        theta = np.matmul(np.linalg.pinv(STATE), mus)
        THETA_L.append(theta)
    print(type(THETA_L[0]), np.shape(THETA_L[0]))
    mse_old = 0
    mse_new = 0
    # match_l = 0
    # match_l_new = 0
    label_origin = []
    label_predict = []
    label_predict_new = []
    for m in range(len(STATE)):
        mse_old += sum((np.matmul(STATE[m], theta_l).T - ACTION[m]) ** 2)
        label_bm = predict((ACTION[m]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        label_origin.append(label_bm[0])
        # label_origin.append(label_bm[0])
        # print(label_bm)
        # label_pred = predict(np.matmul(STATE[m], THETA_L[0]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        # label_pred = predict(np.matmul(STATE[m], THETA_L[num]).T, n_components, WEIGHT_L, MEAN_L, COV_L) ##
        label_pred = predict(np.matmul(STATE[m], theta_l).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        label_predict.append(label_pred[0])
        mse_new += sum((np.matmul(STATE[m], THETA_L[label_pred[0]]).T - ACTION[m]) ** 2)
        label_pred_new = predict(np.matmul(STATE[m], THETA_L[label_pred[0]]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        label_predict_new.append(label_pred_new[0])
        # if label_pred[0] == label_bm[0]:
        #     match_l += 1
        # if label_pred_new[0] == label_bm[0]:
        #     match_l_new += 1

    ## pose
    # print("old MSE =", mse_old / len(STATE))
    # print("new MSE =", mse_new / len(STATE))
    # print("prediction/original data match rate =", match_l / len(STATE))
    # print("prediction data/model label accuracy rate =", score_s / len(STATE))
    # print("new prediction/original data match rate =", match_l_new / len(STATE))
    # print("new prediction data/model label accuracy rate =", score_s_new / len(STATE))
    # print("original data/model accuracy rate =", score_a / len(STATE))
    ## force
    print("old MSE =", mse_old / len(STATE))
    print("new MSE =", mse_new / len(STATE))
    print("prediction/original data match rate =", metrics.accuracy_score(label_origin, label_predict))
    # print("prediction data/model label accuracy rate =", score_s / len(STATE))
    print("new prediction/original data match rate =", metrics.accuracy_score(label_origin, label_predict_new))
    # print("new prediction data/model label accuracy rate =", score_s_new / len(STATE))
    # print("original data/model accuracy rate =", score_a / len(STATE))
    # print(metrics.adjusted_rand_score(label_origin, label_predict))
    print(metrics.calinski_harabasz_score(ACTION, label_predict_new))

    # # High level policy
    # # parameters of the GMM (n_components = 4)
    seq = [0, 1, 2, 3]
    # M = readtest("GMM_parameters_1_H.txt", 0, 8)
    M = readtest("GMM_parameters_2_H.txt", 0, 8)        # best
    # M = readtest("GMM_parameters_3_H.txt", 0, 8)
    # M = readtest("GMM_parameters_4_H.txt", 0, 8)
    m = np.empty(shape=[4, 6])
    m[seq[0]] = np.array(M[:6])
    m[seq[1]] = np.array(M[6:12])
    m[seq[2]] = np.array(M[12:18])
    m[seq[3]] = np.array(M[18:24])
    # C = readtest("GMM_parameters_1_H.txt", 8, 44)
    C = readtest("GMM_parameters_2_H.txt", 8, 50)
    # C = readtest("GMM_parameters_3_H.txt", 8, 44)
    # C = readtest("GMM_parameters_4_H.txt", 8, 50)
    c = np.empty(shape=[4, 6, 6])
    c[seq[0]] = np.array([C[:6], C[6:6 * 2], C[6 * 2:6 * 3], C[6 * 3:6 * 4], C[6 * 4:6 * 5], C[6 * 5:6 * 6]])
    c[seq[1]]  = np.array([C[6 * 6:6 * 7], C[6 * 7:6 * 8], C[6 * 8:6 * 9], C[6 * 9:6 * 10], C[6 * 10:6 * 11], C[6 * 11:6 * 12]])
    c[seq[2]]  = np.array([C[6 * 12:6 * 13], C[6 * 13:6 * 14], C[6 * 14:6 * 15], C[6 * 15:6 * 16], C[6 * 16:6 * 17], C[6 * 17:6 * 18]])
    c[seq[3]]  = np.array([C[6 * 18:6 * 19], C[6 * 19:6 * 20], C[6 * 20:6 * 21], C[6 * 21:6 * 22], C[6 * 22:6 * 23], C[6 * 23:6 * 24]])
    # W = readtest("GMM_parameters_1_H.txt", 44, 48)
    W = readtest("GMM_parameters_2_H.txt", 50, 54)
    # W = readtest("GMM_parameters_3_H.txt", 44, 48)
    # W = readtest("GMM_parameters_4_H.txt", 50, 54)
    w = np.empty(shape=4)
    w[seq[0]] = W[0]
    w[seq[1]] = W[1]
    w[seq[2]] = W[2]
    w[seq[3]] = W[3]
    MEAN_H = [m[0], m[1], m[2], m[3]]
    COV_H = [c[0], c[1], c[2], c[3]]
    WEIGHT_H = [w[0], w[1], w[2], w[3]]
    THETA_H = []
    this_dir, this_filename = os.path.split(__file__)

    f = open(os.path.join(this_dir, "document", "Data_high.txt"), "r")
    # fh = f.readlines()[11180*0:11180*60]
    fh = f.readlines()
    f1 = fh[11180 * 0:11180 * 40]
    f2 = fh[11180 * 60:11180 * 80]
    fh = f1 + f2
    # fh = f.readlines()
    # f1 = fh[11180*0:11180 * 20]
    # f2 = fh[11180*40:11180*80]
    # fh = f1 + f2
    # fh = f.readlines()[11180 * 20:11180 * 80]
    S = []
    A = []
    n = 0
    for string in fh:
        n += 1
        strlist = string.replace("(", "").replace(")", "")
        data = eval(strlist)
        state_h = []
        for i in range(6):
            state_h.append(data[i])
        for j in range(12, len(data)):
            state_h.append(data[j])
        S.append(state_h)
        state_l = []
        for k in range(6, 12):
            state_l.append(data[k])
        A.append(state_l)
    STATE_H = np.asarray(S)
    STATE_L = np.asarray(A)
    a = np.linalg.inv(np.matmul(np.transpose(STATE_H), STATE_H))  # (X'*X)^-1
    theta_h = np.matmul(np.matmul(a, np.transpose(STATE_H)), STATE_L)  # (X'*X)^-1*X'*y
    f.close()

    print(np.shape(theta_h))

    n_components = 4
    print("high level start")
    ## Validation
    f = open(os.path.join(this_dir, "document", "Data_high.txt"), "r")
    # fh = f.readlines()[11180*60:11180*80]
    fh = f.readlines()[11180 * 40:11180 * 60]
    # fh = f.readlines()[11180 * 20:11180 * 40]
    # fh = f.readlines()[11180 * 0:11180 * 20]
    S_H = []
    S_L = []
    n = 0
    for string in fh:
        n += 1
        strlist = string.replace("(", "").replace(")", "")
        data = eval(strlist)
        state_now = []
        state_h = []
        for i in range(6):
            state_h.append(data[i])
        for j in range(12,len(data)):
            state_h.append(data[j])
        # print(state)
        S_H.append(state_h)
        state_l = []
        for k in range(6,12):
            state_l.append(data[k])
        S_L.append(state_l)
    STATE_H = np.asarray(S_H)
    STATE_L = np.asarray(S_L)

    for m in MEAN_H:
        mus = np.tile(m, [len(STATE_L), 1])
        theta = np.matmul(np.linalg.pinv(STATE_H), mus)
        THETA_H.append(theta)

    mse_old = 0
    mse_new = 0
    # match_h = 0
    # match_h_new = 0
    label_origin = []
    label_predict = []
    label_predict_new = []
    for m in range(len(STATE_H)):
        mse_old += sum((np.matmul(STATE_H[m], theta_h).T - STATE_L[m]) ** 2)
        label_bm = predict((STATE_L[m]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        label_origin.append(label_bm[0])
        # print(label_bm)
        # label_pred = predict(np.matmul(STATE_H[m], THETA_H[0]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        # label_pred = predict(np.matmul(STATE_H[m], THETA_H[num]).T, n_components, WEIGHT_H, MEAN_H, COV_H) ##
        label_pred = predict(np.matmul(STATE_H[m], theta_h).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        label_predict.append(label_pred[0])
        mse_new += sum((np.matmul(STATE_H[m], THETA_H[label_pred[0]]).T - STATE_L[m]) ** 2)
        label_pred_new = predict(np.matmul(STATE_H[m], THETA_H[label_pred[0]]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        label_predict_new.append(label_pred_new[0])
        # if label_pred[0] == label_bm[0]:
        #     match_h += 1
        # if label_pred_new[0] == label_bm[0]:
        #     match_h_new += 1

    print("old MSE =", mse_old/len(STATE_H))
    print("new MSE =", mse_new/len(STATE_H))
    print("prediction/original data match rate =", metrics.accuracy_score(label_origin, label_predict))
    # print("prediction data/model label accuracy rate =", score_sh / len(STATE_H))
    print("new prediction/original data match rate =", metrics.accuracy_score(label_origin, label_predict_new))
    # print("new prediction data/model label accuracy rate =", score_sh_new / len(STATE_H))
    # print("original data/model accuracy rate =", score_sl / len(STATE_H))
