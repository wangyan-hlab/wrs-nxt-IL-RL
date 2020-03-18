import os
import numpy as np
from scipy.stats import multivariate_normal as mvn

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
                para.append(float(x))
            else:
                para.append(float(ss))
    return para

def predictlowpose(state_given):
    ## Low level policy
    # parameters of the GMM (n_components = 4)
    seq = [0, 1, 2, 3]
    M = readtest("GMM_parameters_2_pose.txt", 0, 8)     # best
    m = np.empty(shape=[4, 6])
    m[seq[0]] = np.array(M[:6])
    m[seq[1]] = np.array(M[6:12])
    m[seq[2]] = np.array(M[12:18])
    m[seq[3]] = np.array(M[18:24])
    C = readtest("GMM_parameters_2_pose.txt", 8, 56)
    c = np.empty(shape=[4, 6, 6])
    c[seq[0]] = np.array([C[:6], C[6:6 * 2], C[6 * 2:6 * 3], C[6 * 3:6 * 4], C[6 * 4:6 * 5], C[6 * 5:6 * 6]])
    c[seq[1]] = np.array([C[6 * 6:6 * 7], C[6 * 7:6 * 8], C[6 * 8:6 * 9], C[6 * 9:6 * 10], C[6 * 10:6 * 11], C[6 * 11:6 * 12]])
    c[seq[2]] = np.array([C[6 * 12:6 * 13], C[6 * 13:6 * 14], C[6 * 14:6 * 15], C[6 * 15:6 * 16], C[6 * 16:6 * 17], C[6 * 17:6 * 18]])
    c[seq[3]] = np.array([C[6 * 18:6 * 19], C[6 * 19:6 * 20], C[6 * 20:6 * 21], C[6 * 21:6 * 22], C[6 * 22:6 * 23], C[6 * 23:6 * 24]])
    W = readtest("GMM_parameters_2_pose.txt", 56, 60)
    w = np.empty(shape=4)
    w[seq[0]] = W[0]
    w[seq[1]] = W[1]
    w[seq[2]] = W[2]
    w[seq[3]] = W[3]
    MEAN_L = [m[0], m[1], m[2], m[3]]
    COV_L = [c[0], c[1], c[2], c[3]]
    WEIGHT_L = [w[0], w[1], w[2], w[3]]
    THETA_L = []

    theta_l_pose = [[-4.58393904e-01, 2.14213490e-02, 9.69631512e-03, -2.90531456e-01, -2.02771275e-01, 1.42180135e-01],
                    [-8.05955745e-01, -5.11666875e-02, -2.49423925e-01, 9.90358299e-02, -1.22899797e-01, -1.47473582e+00],
                    [-2.35072845e-01, 5.01353484e-03, -1.25161167e-02, -1.21721099e-01, -8.73408310e-02, -1.87571995e-02],
                    [-2.48562382e+0, -3.79032090e-02, -1.30978524e-01, -3.02386432e+00, -4.92601555e-01, 2.05909572e+00],
                    [-9.18606883e-01, 5.92196743e-02, 9.04995827e-02, -8.73696278e-01, -4.28024702e-01, 9.06175633e-01],
                    [ 2.09917458e+00, 2.47071132e-01, 1.09366174e+00, -1.37491246e+00, -4.44882567e-02, 6.80040468e+00],
                    [ 1.25339605e+00, -2.11918910e-02, -2.55856331e-02, 1.91775155e-01, 1.39263079e-01, -2.07174824e-01],
                    [-8.53202193e-01, 1.04934478e+00, 6.98949951e-02, -7.55284791e-01, -3.88523275e-01, 6.85829255e-01],
                    [ 9.97950948e-02, -5.07046812e-03, 1.00521320e+00,  4.71385967e-02, 4.83884415e-02, -1.24239983e-02],
                    [-7.18867988e-01, 4.28773663e-01, -1.44344235e-01,  2.38830364e+00, 2.92457243e-01, -3.99659266e+00],
                    [ 1.18690078e+00, -8.11811733e-02, 3.62265117e+00, -1.28958031e+00, 7.87573312e-01, 1.02924017e+00],
                    [-1.26419522e+00, -3.98479777e+00, -1.05073806e+00, 1.88733331e+00, 4.99069403e-01, -5.65971726e+00]]

    this_dir, this_filename = os.path.split(__file__)
    n_components = 4
    f = open(os.path.join(this_dir, "document", "Data_low.txt"), "r")
    fl = f.readlines()[2945 * 40:2945 * 60]     # for pose prediction
    S = []
    A = []
    for string in fl:
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
            action.append(data[k])
        A.append(action)
    STATE = np.asarray(S)
    ACTION = np.asarray(A)

    for m in MEAN_L:
        mus = np.tile(m, [len(ACTION), 1])
        theta = np.matmul(np.linalg.pinv(STATE), mus)
        THETA_L.append(theta)

    label_pred = predict(np.matmul(state_given, theta_l_pose).T, n_components, WEIGHT_L, MEAN_L, COV_L)
    label_pred_new = predict(np.matmul(state_given, THETA_L[label_pred[0]]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
    action_pose = np.matmul(state_given, THETA_L[label_pred_new[0]])

    return action_pose

def predictlowforce(state_given):
    ## Low level policy
    # parameters of the GMM (n_components = 4)
    seq = [0, 1, 2, 3]
    M = readtest("GMM_parameters_3_force.txt", 0, 6)      # best
    m = np.empty(shape=[4, 6])
    m[seq[0]] = np.array(M[:6])
    m[seq[1]] = np.array(M[6:12])
    m[seq[2]] = np.array(M[12:18])
    m[seq[3]] = np.array(M[18:24])
    C = readtest("GMM_parameters_3_force.txt", 6, 54)
    c = np.empty(shape=[4, 6, 6])
    c[seq[0]] = np.array([C[:6], C[6:6 * 2], C[6 * 2:6 * 3], C[6 * 3:6 * 4], C[6 * 4:6 * 5], C[6 * 5:6 * 6]])
    c[seq[1]] = np.array(
        [C[6 * 6:6 * 7], C[6 * 7:6 * 8], C[6 * 8:6 * 9], C[6 * 9:6 * 10], C[6 * 10:6 * 11], C[6 * 11:6 * 12]])
    c[seq[2]] = np.array([C[6 * 12:6 * 13], C[6 * 13:6 * 14], C[6 * 14:6 * 15], C[6 * 15:6 * 16], C[6 * 16:6 * 17],
                          C[6 * 17:6 * 18]])
    c[seq[3]] = np.array([C[6 * 18:6 * 19], C[6 * 19:6 * 20], C[6 * 20:6 * 21], C[6 * 21:6 * 22], C[6 * 22:6 * 23],
                          C[6 * 23:6 * 24]])
    W = readtest("GMM_parameters_3_force.txt", 54, 58)
    w = np.empty(shape=4)
    w[seq[0]] = W[0]
    w[seq[1]] = W[1]
    w[seq[2]] = W[2]
    w[seq[3]] = W[3]
    MEAN_L = [m[0], m[1], m[2], m[3]]
    COV_L = [c[0], c[1], c[2], c[3]]
    WEIGHT_L = [w[0], w[1], w[2], w[3]]
    THETA_L = []

    theta_l_force = [[7.13224685e-01, 3.56570406e-01, 7.10668021e-01, 3.89772123e-02, -2.31187789e-01, 7.86257791e-02],
                     [1.84657030e+00, 1.91488262e+00, 1.90537957e+00, 1.81653786e-01, -6.48289336e-01, 5.70511006e-01],
                     [-2.68036911e-02, 3.12947088e-01, 8.16301544e-01, 1.06248095e-02, -2.01092737e-01, 5.71786300e-02],
                     [1.77279741e+00, 2.31999862e+00, 2.54907141e+00, -1.65739380e+00, -1.66144120e+00, 2.80884386e-01],
                     [1.00038962e+00, 1.81963073e-01, 3.76552324e+00, -1.71426585e-01, -1.06641192e+00, -2.41244527e-02],
                     [1.46649628e+00, 1.17772780e+00, -1.63196395e+01, 3.79293939e-01, 3.93223404e+00, -1.39753977e+00],
                     [-8.42195619e-01, -2.98748671e-01, -8.51818997e-01, -3.09907943e-02, 2.77058379e-01, -7.57598523e-02],
                     [-1.19420522e+00, -2.21819008e+00, -2.09475317e+00, -2.25731872e-01, 6.02364868e-01, -5.99999460e-01],
                     [-1.18535484e-01, -2.45090508e-01, -1.17439465e+00, -2.94580466e-03, 2.99711700e-01, -5.42530013e-02],
                     [-4.46948908e+00, -1.74544697e+00, 4.96653439e+00, 1.45804431e+00, 3.07103083e-01, -4.02281494e-01],
                     [-7.38270107e-01, -1.61165836e-01, -3.10667867e+00, 1.61641624e-01, 9.65715575e-01, 2.23043917e-02],
                     [2.29029941e-01, -1.55854954e+00, 1.42728724e+01, -4.04122814e-01, -3.87311983e+00, 1.19491958e+00]]

    this_dir, this_filename = os.path.split(__file__)
    n_components = 4
    f = open(os.path.join(this_dir, "document", "Data_low.txt"), "r")
    fl = f.readlines()[2945 * 20:2945 * 40]       # for force prediction
    S = []
    A = []
    for string in fl:
        strlist = string.replace("(", "").replace(")", "")
        data = eval(strlist)
        state = []
        for i in range(6):
            state.append(data[i])
        for j in range(18, len(data)):
            state.append(data[j])
        S.append(state)
        action = []
        for k in range(12, 18):
            action.append(data[k])
        A.append(action)
    STATE = np.asarray(S)
    ACTION = np.asarray(A)

    for m in MEAN_L:
        mus = np.tile(m, [len(ACTION), 1])
        theta = np.matmul(np.linalg.pinv(STATE), mus)
        THETA_L.append(theta)

    label_pred = predict(np.matmul(state_given, theta_l_force).T, n_components, WEIGHT_L, MEAN_L, COV_L)
    label_pred_new = predict(np.matmul(state_given, THETA_L[label_pred[0]]).T, n_components, WEIGHT_L, MEAN_L, COV_L)
    action_force = np.matmul(state_given, THETA_L[label_pred_new[0]])

    return action_force

def predicthigh(state_given):
    # High level policy
    # parameters of the GMM (n_components = 4)
    seq = [0, 1, 2, 3]
    M = readtest("GMM_parameters_2_H.txt", 0, 8)        # best
    m = np.empty(shape=[4, 6])
    m[seq[0]] = np.array(M[:6])
    m[seq[1]] = np.array(M[6:12])
    m[seq[2]] = np.array(M[12:18])
    m[seq[3]] = np.array(M[18:24])
    C = readtest("GMM_parameters_2_H.txt", 8, 50)
    c = np.empty(shape=[4, 6, 6])
    c[seq[0]] = np.array([C[:6], C[6:6 * 2], C[6 * 2:6 * 3], C[6 * 3:6 * 4], C[6 * 4:6 * 5], C[6 * 5:6 * 6]])
    c[seq[1]]  = np.array([C[6 * 6:6 * 7], C[6 * 7:6 * 8], C[6 * 8:6 * 9], C[6 * 9:6 * 10], C[6 * 10:6 * 11], C[6 * 11:6 * 12]])
    c[seq[2]]  = np.array([C[6 * 12:6 * 13], C[6 * 13:6 * 14], C[6 * 14:6 * 15], C[6 * 15:6 * 16], C[6 * 16:6 * 17], C[6 * 17:6 * 18]])
    c[seq[3]]  = np.array([C[6 * 18:6 * 19], C[6 * 19:6 * 20], C[6 * 20:6 * 21], C[6 * 21:6 * 22], C[6 * 22:6 * 23], C[6 * 23:6 * 24]])
    W = readtest("GMM_parameters_2_H.txt", 50, 54)
    w = np.empty(shape=4)
    w[seq[0]] = W[0]
    w[seq[1]] = W[1]
    w[seq[2]] = W[2]
    w[seq[3]] = W[3]
    MEAN_H = [m[0], m[1], m[2], m[3]]
    COV_H = [c[0], c[1], c[2], c[3]]
    WEIGHT_H = [w[0], w[1], w[2], w[3]]
    THETA_H = []

    theta_h = [[ 6.61576237e-01, -2.12584017e-02, 3.42573249e-02, 5.27144791e-04, -8.83038108e-03, -2.81995565e-03],
               [-5.77080847e-02, 5.06755195e-01, 6.28463245e-02, -1.43630205e-02, 3.61737030e-02, -2.29471380e-02],
               [-1.42164263e-01, -4.22864786e-03, 7.56971690e-01, -1.15263886e-03, 1.46679737e-02, -1.37446706e-03],
               [-1.08315342e+00, -1.36196922e-01, -2.20305615e-01, 6.90617602e-01, -3.38938668e-01, -3.71691547e-02],
               [-2.80526394e-01, 3.26463450e-02, 5.18689229e-03, -3.73486892e-03, 7.64448711e-01, 2.96673239e-03],
               [ 1.21943544e+00, 1.52097870e-01, 1.11493384e-01, -2.29644177e-02, 1.39130809e-01, 6.78217923e-01],
               [ 3.25074486e-01, 1.76644731e-02, 9.46191199e-03, -6.55497027e-04, 9.95186390e-03, 2.66285735e-03],
               [ 1.31949749e-01, 5.00306052e-01, -2.57566976e-01, 1.12023691e-02, -1.28720424e-02, 2.41167509e-02],
               [ 1.18682326e-01, -1.73033481e-03,  2.96756650e-01, 3.10698668e-04, -5.21700229e-03, 1.20279789e-03],
               [ 6.99894511e-01, 1.60971721e-01, -5.05913924e-02, 3.29724625e-01, 2.62639911e-01, 3.74563482e-02],
               [ 2.98280747e-01, -2.86930257e-02, -7.87622803e-02, 6.51676976e-03, 2.40155002e-01, -2.69806992e-03],
               [-7.35062230e-01, -1.66466837e-01, -9.73651373e-02, 5.85013888e-03, -7.15352666e-02, 3.31218094e-01]]

    this_dir, this_filename = os.path.split(__file__)
    n_components = 4
    f = open(os.path.join(this_dir, "document", "Data_high.txt"), "r")
    fh = f.readlines()[11180 * 40:11180 * 60]
    S_H = []
    S_L = []
    for string in fh:
        strlist = string.replace("(", "").replace(")", "")
        data = eval(strlist)
        state_h = []
        for i in range(6):
            state_h.append(data[i])
        for j in range(12,len(data)):
            state_h.append(data[j])
        S_H.append(state_h)
        state_l = []
        for k in range(6, 12):
            state_l.append(data[k])
        S_L.append(state_l)
    STATE_H = np.asarray(S_H)
    STATE_L = np.asarray(S_L)

    for m in MEAN_H:
        mus = np.tile(m, [len(STATE_L), 1])
        theta = np.matmul(np.linalg.pinv(STATE_H), mus)
        THETA_H.append(theta)

    label_pred = predict(np.matmul(state_given, theta_h).T, n_components, WEIGHT_H, MEAN_H, COV_H)
    label_pred_new = predict(np.matmul(state_given, THETA_H[label_pred[0]]).T, n_components, WEIGHT_H, MEAN_H, COV_H)
    subgoal_state = np.matmul(state_given, THETA_H[label_pred_new[0]])

    return subgoal_state

if __name__ == '__main__':
    s_c = [625.535, 72.599, -260.742, -1.162, 21.429, -4.17]
    s_h = [634.824, 71.948, -266.456, -1.255, 21.233, -4.289]
    goal = s_c + s_h
    print(goal)
    subgoal = s_c + list(predicthigh(goal))
    print(subgoal)
    pose = predictlowpose(subgoal)
    action = predictlowforce(subgoal)
    print(pose)
    print(action)
