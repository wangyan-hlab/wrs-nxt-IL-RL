import os
import numpy as np
import matplotlib.pyplot as plt
plt.style.use("seaborn")
from scipy.stats import multivariate_normal as mvn
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
    # M = readtest("GMM_parameters_1.txt", 0, 12)
    # M = readtest("GMM_parameters_2.txt", 0, 12)
    # M = readtest("GMM_parameters_3.txt", 0, 12)
    # M = readtest("GMM_parameters_4.txt", 0, 12)
    M = readtest("GMM_parameters_1_pose.txt", 0, 8)
    # M = readtest("GMM_parameters_2_pose.txt", 0, 8)
    # M = readtest("GMM_parameters_3_pose.txt", 0, 8)
    # M = readtest("GMM_parameters_4_pose.txt", 0, 8)
    # M = readtest("GMM_parameters_1_force.txt", 0, 4)
    # M = readtest("GMM_parameters_2_force.txt", 0, 5)
    # M = readtest("GMM_parameters_3_force.txt", 0, 6)
    # M = readtest("GMM_parameters_4_force.txt", 0, 4)
    # m1 = np.array(M[:12])
    # m2 = np.array(M[12:24])
    # m3 = np.array(M[24:36])
    # m4 = np.array(M[36:48])
    m1 = np.array(M[:6])
    m2 = np.array(M[6:12])
    m3 = np.array(M[12:18])
    m4 = np.array(M[18:24])
    # C = readtest("GMM_parameters_1.txt", 12, 156)
    # C = readtest("GMM_parameters_2.txt", 12, 156)
    # C = readtest("GMM_parameters_3.txt", 12, 156)
    # C = readtest("GMM_parameters_4.txt", 12, 156)
    C = readtest("GMM_parameters_1_pose.txt", 8, 56)
    # C = readtest("GMM_parameters_2_pose.txt", 8, 56)
    # C = readtest("GMM_parameters_3_pose.txt", 8, 56)
    # C = readtest("GMM_parameters_4_pose.txt", 8, 56)
    # C = readtest("GMM_parameters_1_force.txt", 4, 46)
    # C = readtest("GMM_parameters_2_force.txt", 5, 53)
    # C = readtest("GMM_parameters_3_force.txt", 6, 54)
    # C = readtest("GMM_parameters_4_force.txt", 4, 52)
    # c1 = np.array([C[:12],C[12:12*2],C[12*2:12*3],C[12*3:12*4],
    #                C[12*4:12*5],C[12*5:12*6],C[12*6:12*7],C[12*7:12*8],
    #                C[12*8:12*9],C[12*9:12*10],C[12*10:12*11],C[12*11:12*12]])
    # c2 = np.array([C[12*12:12*13], C[12*13:12 * 14], C[12 * 14:12 * 15], C[12 * 15:12 * 16],
    #                C[12 * 16:12 * 17], C[12 * 17:12 * 18], C[12 * 18:12 * 19], C[12 * 19:12 * 20],
    #                C[12 * 20:12 * 21], C[12 * 21:12 * 22], C[12 * 22:12 * 23], C[12 * 23:12 * 24]])
    # c3 = np.array([C[12 * 24:12 * 25], C[12 * 25:12 * 26], C[12 * 26:12 * 27], C[12 * 27:12 * 28],
    #                C[12 * 28:12 * 29], C[12 * 29:12 * 30], C[12 * 30:12 * 31], C[12 * 31:12 * 32],
    #                C[12 * 32:12 * 33], C[12 * 33:12 * 34], C[12 * 34:12 * 35], C[12 * 35:12 * 36]])
    # c4 = np.array([C[12 * 36:12 * 37], C[12 * 37:12 * 38], C[12 * 38:12 * 39], C[12 * 39:12 * 40],
    #                C[12 * 40:12 * 41], C[12 * 41:12 * 42], C[12 * 42:12 * 43], C[12 * 43:12 * 44],
    #                C[12 * 44:12 * 45], C[12 * 45:12 * 46], C[12 * 46:12 * 47], C[12 * 47:12 * 48]])
    c1 = np.array([C[:6], C[6:6 * 2], C[6 * 2:6 * 3], C[6 * 3:6 * 4], C[6 * 4:6 * 5], C[6 * 5:6 * 6]])
    c2 = np.array([C[6 * 6:6 * 7], C[6 * 7:6 * 8], C[6 * 8:6 * 9], C[6 * 9:6 * 10], C[6 * 10:6 * 11], C[6 * 11:6 * 12]])
    c3 = np.array([C[6 * 12:6 * 13], C[6 * 13:6 * 14], C[6 * 14:6 * 15], C[6 * 15:6 * 16], C[6 * 16:6 * 17], C[6 * 17:6 * 18]])
    c4 = np.array([C[6 * 18:6 * 19], C[6 * 19:6 * 20], C[6 * 20:6 * 21], C[6 * 21:6 * 22], C[6 * 22:6 * 23], C[6 * 23:6 * 24]])
     # W = readtest("GMM_parameters_1.txt", 156, 160)
    # W = readtest("GMM_parameters_2.txt", 156, 160)
    # W = readtest("GMM_parameters_3.txt", 156, 160)
    # W = readtest("GMM_parameters_4.txt", 156, 160)
    W = readtest("GMM_parameters_1_pose.txt", 56, 60)
    # W = readtest("GMM_parameters_2_pose.txt", 56, 60)
    # W = readtest("GMM_parameters_3_pose.txt", 56, 60)
    # W = readtest("GMM_parameters_4_pose.txt", 56, 60)
    # W = readtest("GMM_parameters_1_force.txt", 46, 50)
    # W = readtest("GMM_parameters_2_force.txt", 53, 57)
    # W = readtest("GMM_parameters_3_force.txt", 54, 58)
    # W = readtest("GMM_parameters_4_force.txt", 52, 56)
    w1 = W[0] #3
    w2 = W[1] #0
    w3 = W[2] #2
    w4 = W[3] #1
    MEAN_L = [m1, m2, m3, m4]
    COV_L = [c1, c2, c3, c4]
    WEIGHT_L = [w1, w2, w3, w4]
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
    # n_components = 5
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
    score_a = 0
    score_s = 0
    match_l = 0
    score_s_new = 0
    match_l_new = 0
    num_per_group = int(len(STATE) / 20)
    print(num_per_group)
    part1 = int(num_per_group * w1 + 0.5)
    part2 = int(num_per_group * w2 + 0.5)
    part3 = int(num_per_group * w3 + 0.5)
    part4 = num_per_group - (part1 + part2 + part3)

    group_1 = []
    group_2 = []
    group_3 = []
    group_4 = []
    # group_5 = []  ##
    # group_6 = []  ##
    for i in range(20):
        group_1 += range(num_per_group * i, num_per_group * i + part1)
        group_2 += range(num_per_group * i + part1, num_per_group * i + part1 + part2)
        group_3 += range(num_per_group * i + part1 + part2, num_per_group * i + part1 + part2 + part3)
        group_4 += range(num_per_group * i + part1 + part2 + part3, num_per_group * i + part1 + part2 + part3 + part4)

    label_origin = []
    mse_old = 0
    mse_new = 0
    for m in group_1:
        # num = random.choices(population=[0, 1, 2, 3], weights=WEIGHT_L, k=1)[0] ##
        # mse += sum((np.matmul(STATE[m], THETA_L[0]).T - ACTION[m])**2)
        # mse += sum((np.matmul(STATE[m], THETA_L[num]).T - ACTION[m]) ** 2)  ##
        mse_old += sum((np.matmul(STATE[m], theta_l).T - ACTION[m]) ** 2)
        label_bm = predict((ACTION[m]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        label_origin.append(label_bm[0])
        # print(label_bm)
        # label_pred = predict(np.matmul(STATE[m], THETA_L[0]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        # label_pred = predict(np.matmul(STATE[m], THETA_L[num]).T, n_components, WEIGHT_L, MEAN_L, COV_L) ##
        label_pred = predict(np.matmul(STATE[m], theta_l).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        mse_new += sum((np.matmul(STATE[m], THETA_L[label_pred[0]]).T - ACTION[m]) ** 2)
        label_pred_new = predict(np.matmul(STATE[m], THETA_L[label_pred[0]]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        # print("label:0     ",label_pred)
        print(label_bm, label_pred)
        if label_pred[0] == label_bm[0]:
            match_l += 1
        if label_pred[0] == 0:
            score_s += 1
        if label_bm[0] == 0:
            score_a += 1
        if label_pred_new[0] == label_bm[0]:
            match_l_new += 1
        if label_pred_new[0] == 0:
            score_s_new += 1
        print("group 1")
    for m in group_2:
        # num = random.choices(population=[0, 1, 2, 3], weights=WEIGHT_L, k=1)[0]  ##
        # mse += sum((np.matmul(STATE[m], THETA_L[1]).T - ACTION[m])**2)
        # mse += sum((np.matmul(STATE[m], THETA_L[num]).T - ACTION[m]) ** 2)  ##
        mse_old += sum((np.matmul(STATE[m], theta_l).T - ACTION[m]) ** 2)
        label_bm = predict((ACTION[m]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        label_origin.append(label_bm[0])
        # print(label_bm)
        # label_pred = predict(np.matmul(STATE[m], THETA_L[1]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        # label_pred = predict(np.matmul(STATE[m], THETA_L[num]).T, n_components, WEIGHT_L, MEAN_L, COV_L)  ##
        label_pred = predict(np.matmul(STATE[m], theta_l).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        mse_new += sum((np.matmul(STATE[m], THETA_L[label_pred[0]]).T - ACTION[m]) ** 2)
        label_pred_new = predict(np.matmul(STATE[m], THETA_L[label_pred[0]]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        # print("label:1     ",label_pred)
        print(label_bm, label_pred)
        if label_pred[0] == label_bm[0]:
            match_l += 1
        if label_pred[0] == 1:
            score_s += 1
        if label_bm[0] == 1:
            score_a += 1
        if label_pred_new[0] == label_bm[0]:
            match_l_new += 1
        if label_pred_new[0] == 1:
            score_s_new += 1
        print("group 2")
    for m in group_3:
        # num = random.choices(population=[0, 1, 2, 3], weights=WEIGHT_L, k=1)[0]  ##
        # mse += sum((np.matmul(STATE[m], THETA_L[2]).T - ACTION[m])**2)
        # mse += sum((np.matmul(STATE[m], THETA_L[num]).T - ACTION[m]) ** 2)  ##
        mse_old += sum((np.matmul(STATE[m], theta_l).T - ACTION[m]) ** 2)
        label_bm = predict((ACTION[m]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        label_origin.append(label_bm[0])
        # print(label_bm)
        # label_pred = predict(np.matmul(STATE[m], THETA_L[2]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        # label_pred = predict(np.matmul(STATE[m], THETA_L[num]).T, n_components, WEIGHT_L, MEAN_L, COV_L)  ##
        label_pred = predict(np.matmul(STATE[m], theta_l).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        mse_new += sum((np.matmul(STATE[m], THETA_L[label_pred[0]]).T - ACTION[m]) ** 2)
        label_pred_new = predict(np.matmul(STATE[m], THETA_L[label_pred[0]]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        # print("label:2     ",label_pred)
        print(label_bm, label_pred)
        if label_pred[0] == label_bm[0]:
            match_l += 1
        if label_pred[0] == 2:
            score_s += 1
        if label_bm[0] == 2:
            score_a += 1
        if label_pred_new[0] == label_bm[0]:
            match_l_new += 1
        if label_pred_new[0] == 2:
            score_s_new += 1
        print("group 3")
    for m in group_4:
        # num = random.choices(population=[0, 1, 2, 3], weights=WEIGHT_L, k=1)[0]  ##
        # mse += sum((np.matmul(STATE[m], THETA_L[3]).T - ACTION[m])**2)
        # mse += sum((np.matmul(STATE[m], THETA_L[num ]).T - ACTION[m]) ** 2) ##
        mse_old += sum((np.matmul(STATE[m], theta_l).T - ACTION[m]) ** 2)
        label_bm = predict((ACTION[m]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        label_origin.append(label_bm[0])
        # print(label_bm)
        # label_pred = predict(np.matmul(STATE[m], THETA_L[3]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        # label_pred = predict(np.matmul(STATE[m], THETA_L[num]).T, n_components, WEIGHT_L, MEAN_L, COV_L)  ##
        label_pred = predict(np.matmul(STATE[m], theta_l).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        mse_new += sum((np.matmul(STATE[m], THETA_L[label_pred[0]]).T - ACTION[m]) ** 2)
        label_pred_new = predict(np.matmul(STATE[m], THETA_L[label_pred[0]]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
        # print("label:3     ",label_pred)
        print(label_bm, label_pred)
        if label_pred[0] == label_bm[0]:
            match_l += 1
        if label_pred[0] == 3:
            score_s += 1
        if label_bm[0] == 3:
            score_a += 1
        if label_pred_new[0] == label_bm[0]:
            match_l_new += 1
        if label_pred_new[0] == 3:
            score_s_new += 1
        print("group 4")
    print(Counter(label_origin))
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
    print("prediction/original data match rate =", match_l / len(STATE))
    print("prediction data/model label accuracy rate =", score_s / len(STATE))
    print("new prediction/original data match rate =", match_l_new / len(STATE))
    print("new prediction data/model label accuracy rate =", score_s_new / len(STATE))
    print("original data/model accuracy rate =", score_a / len(STATE))

# --------------------------------------------------------------------------

    # High level policy
    # parameters of the GMM (n_components = 4)
    # M = readtest("GMM_parameters_1_H.txt", 0, 8)
    # M = readtest("GMM_parameters_2_H.txt", 0, 8)
    # M = readtest("GMM_parameters_3_H.txt", 0, 8)
    M = readtest("GMM_parameters_4_H.txt", 0, 8)
    m1 = np.array(M[:6])
    m2 = np.array(M[6:12])
    m3 = np.array(M[12:18])
    m4 = np.array(M[18:24])
    # C = readtest("GMM_parameters_1_H.txt", 8, 44)
    # C = readtest("GMM_parameters_2_H.txt", 8, 50)
    # C = readtest("GMM_parameters_3_H.txt", 8, 44)
    C = readtest("GMM_parameters_4_H.txt", 8, 50)
    c1 = np.array([C[:6], C[6:6 * 2], C[6 * 2:6 * 3], C[6 * 3:6 * 4], C[6 * 4:6 * 5], C[6 * 5:6 * 6]])
    c2 = np.array([C[6 * 6:6 * 7], C[6 * 7:6 * 8], C[6 * 8:6 * 9], C[6 * 9:6 * 10], C[6 * 10:6 * 11], C[6 * 11:6 * 12]])
    c3 = np.array([C[6 * 12:6 * 13], C[6 * 13:6 * 14], C[6 * 14:6 * 15], C[6 * 15:6 * 16], C[6 * 16:6 * 17], C[6 * 17:6 * 18]])
    c4 = np.array([C[6 * 18:6 * 19], C[6 * 19:6 * 20], C[6 * 20:6 * 21], C[6 * 21:6 * 22], C[6 * 22:6 * 23], C[6 * 23:6 * 24]])
    # W = readtest("GMM_parameters_1_H.txt", 44, 48)
    # W = readtest("GMM_parameters_2_H.txt", 50, 54)
    # W = readtest("GMM_parameters_3_H.txt", 44, 48)
    W = readtest("GMM_parameters_4_H.txt", 50, 54)
    w1 = W[0]
    w2 = W[1]
    w3 = W[2]
    w4 = W[3]
    MEAN_H = [m1, m2, m3, m4]
    COV_H = [c1, c2, c3, c4]
    WEIGHT_H = [w1, w2, w3, w4]
    THETA_H = []

    this_dir, this_filename = os.path.split(__file__)
    f = open(os.path.join(this_dir, "document", "Data_high.txt"), "r")
    # fh = f.readlines()[11180*0:11180*60]
    # fh = f.readlines()
    # f1 = fh[11180 * 0:11180 * 40]
    # f2 = fh[11180 * 60:11180 * 80]
    # fh = f1 + f2
    # fh = f.readlines()
    # f1 = fh[11180*0:11180 * 20]
    # f2 = fh[11180*40:11180*80]
    # fh = f1 + f2
    fh = f.readlines()[11180 * 20:11180 * 80]
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

    n_components = 4
    print("high level start")
    ## Validation
    f = open(os.path.join(this_dir, "document", "Data_high.txt"), "r")
    # fh = f.readlines()[11180*60:11180*80]
    # fh = f.readlines()[11180 * 40:11180 * 60]
    # fh = f.readlines()[11180 * 20:11180 * 40]
    fh = f.readlines()[11180 * 0:11180 * 20]
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

    score_sl = 0
    score_sh = 0
    score_sh_new = 0
    match_h_new = 0
    match_h = 0
    num_per_group = int(len(STATE_H) / 20)
    # print(num_per_group)
    part1 = int(num_per_group * w1 + 0.5)
    part2 = int(num_per_group * w2 + 0.5)
    part3 = int(num_per_group * w3 + 0.5)
    part4 = num_per_group - (part1 + part2 + part3)

    group_1 = []
    group_2 = []
    group_3 = []
    group_4 = []
    for i in range(20):
        group_1 += range(num_per_group * i, num_per_group * i + part1)
        group_2 += range(num_per_group * i + part1, num_per_group * i + part1 + part2)
        group_3 += range(num_per_group * i + part1 + part2, num_per_group * i + part1 + part2 + part3)
        group_4 += range(num_per_group * i + part1 + part2 + part3, num_per_group * i + part1 + part2 + part3 + part4)

    mse_old = 0
    mse_new = 0
    n_h1 = 0
    n_h2 = 0
    n_h3 = 0
    n_h4 = 0
    for m in group_1:
        # mse += sum((np.matmul(STATE_H[m], THETA_H[0]).T - STATE_L[m]) ** 2)
        mse_old += sum((np.matmul(STATE_H[m], theta_h).T - STATE_L[m]) ** 2)
        label_bm = predict((STATE_L[m]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        # print(label_bm)
        # label_pred = predict(np.matmul(STATE_H[m], THETA_H[0]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        label_pred = predict(np.matmul(STATE_H[m], theta_h).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        mse_new += sum((np.matmul(STATE_H[m], THETA_H[label_pred[0]]).T - STATE_L[m]) ** 2)
        label_pred_new = predict(np.matmul(STATE_H[m], THETA_H[label_pred[0]]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        # print("label:0     ",label_pred)
        # print(label_bm, label_pred)
        if label_pred[0] == label_bm[0]:
            match_h += 1
        if label_pred[0] == 0:
            score_sh += 1
        if label_bm[0] == 0:
            score_sl += 1
        if label_pred_new[0] == label_bm[0]:
            match_h_new += 1
        if label_pred_new[0] == 0:
            score_sh_new += 1

    for m in group_2:
        # mse += sum((np.matmul(STATE_H[m], THETA_H[1]).T - STATE_L[m]) ** 2)
        mse_old += sum((np.matmul(STATE_H[m], theta_h).T - STATE_L[m]) ** 2)
        label_bm = predict((STATE_L[m]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        # print(label_bm)
        # label_pred = predict(np.matmul(STATE_H[m], THETA_H[1]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        label_pred = predict(np.matmul(STATE_H[m], theta_h).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        label_pred_new = predict(np.matmul(STATE_H[m], THETA_H[label_pred[0]]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        mse_new += sum((np.matmul(STATE_H[m], THETA_H[label_pred[1]]).T - STATE_L[m]) ** 2)
        # print("label:1     ",label_pred)
        # print(label_bm, label_pred)
        if label_pred[0] == label_bm[0]:
            match_h += 1
        if label_pred[0] == 1:
            score_sh += 1
        if label_bm[0] == 1:
            score_sl += 1
        if label_pred_new[0] == label_bm[0]:
            match_h_new += 1
        if label_pred_new[0] == 1:
            score_sh_new += 1

    for m in group_3:
        # mse += sum((np.matmul(STATE_H[m], THETA_H[2]).T - STATE_L[m]) ** 2)
        mse_old += sum((np.matmul(STATE_H[m], theta_h).T - STATE_L[m]) ** 2)
        label_bm = predict((STATE_L[m]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        # print(label_bm)
        # label_pred = predict(np.matmul(STATE_H[m], THETA_H[2]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        label_pred = predict(np.matmul(STATE_H[m], theta_h).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        label_pred_new = predict(np.matmul(STATE_H[m], THETA_H[label_pred[0]]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        mse_new += sum((np.matmul(STATE_H[m], THETA_H[label_pred[2]]).T - STATE_L[m]) ** 2)
        # print("label:2     ",label_pred)
        # print(label_bm, label_pred)
        if label_pred[0] == label_bm[0]:
            match_h += 1
        if label_pred[0] == 2:
            score_sh += 1
        if label_bm[0] == 2:
            score_sl += 1
        if label_pred_new[0] == label_bm[0]:
            match_h_new += 1
        if label_pred_new[0] == 2:
            score_sh_new += 1

    for m in group_4:
        # mse += sum((np.matmul(STATE_H[m], THETA_H[3]).T - STATE_L[m]) ** 2)
        mse_old += sum((np.matmul(STATE_H[m], theta_h).T - STATE_L[m]) ** 2)
        label_bm = predict((STATE_L[m]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        # print(label_bm)
        # label_pred = predict(np.matmul(STATE_H[m], THETA_H[3]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        label_pred = predict(np.matmul(STATE_H[m], theta_h).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        label_pred_new = predict(np.matmul(STATE_H[m], THETA_H[label_pred[0]]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
        mse_new += sum((np.matmul(STATE_H[m], THETA_H[label_pred[3]]).T - STATE_L[m]) ** 2)
        # print("label:3     ",label_pred)
        # print(label_bm, label_pred)
        if label_pred[0] == label_bm[0]:
            match_h += 1
        if label_pred[0] == 3:
            score_sh += 1
        if label_bm[0] == 3:
            score_sl += 1
        if label_pred_new[0] == label_bm[0]:
            match_h_new += 1
        if label_pred_new[0] == 3:
            score_sh_new += 1

    print("old MSE =", mse_old/len(STATE_H))
    print("new MSE =", mse_new/len(STATE_H))
    print("prediction/original data match rate =", match_h / len(STATE_H))
    print("prediction data/model label accuracy rate =", score_sh / len(STATE_H))
    print("new prediction/original data match rate =", match_h_new / len(STATE_H))
    print("new prediction data/model label accuracy rate =", score_sh_new / len(STATE_H))
    print("original data/model accuracy rate =", score_sl / len(STATE_H))
