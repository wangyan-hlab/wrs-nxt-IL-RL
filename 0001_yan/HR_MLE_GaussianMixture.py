import os
import numpy as np
import matplotlib.pyplot as plt
plt.style.use("seaborn")
from scipy.stats import multivariate_normal
from sklearn.mixture import GaussianMixture
from collections import Counter

def writefile(ls, filename, edit="w"):
    this_dir, this_filename = os.path.split(__file__)
    fl = open(os.path.join(this_dir, "document", filename), edit)
    for poses in ls:
        fl.write(str(poses)+'\n')
    fl.close()

if __name__ == '__main__':
    # determine means and covariances for low and high level GMMs
    this_dir, this_filename = os.path.split(__file__)
    ## Low level policy
    # Training
    f = open(os.path.join(this_dir, "document", "Data_low.txt"), "r")
    # fl = f.readlines()[2945*0:2945 * 60]
    # fl = f.readlines()
    # f1 = fl[2945*0:2945 * 40]
    # f2 = fl[2945*60:2945*80]  #[0:294500:50]
    # fl = f1 + f2
    # fl = f.readlines()
    # f1 = fl[2945*0:2945 * 20]
    # f2 = fl[2945*40:2945*80]  #[0:294500:50]
    # fl = f1 + f2
    fl = f.readlines()[2945 * 20:2945 * 80]
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
        # for k in range(6, 12):
        for k in range(12, 18):
            action.append(data[k])
        A.append(action)
        # print(n)
    STATE = np.asarray(S)
    ACTION = np.asarray(A)
    print(np.shape(ACTION))
    rdc_dim = 2
    u1, s1, v1 = np.linalg.svd(np.cov(np.transpose(ACTION)))
    u1_reduce = u1[:, :rdc_dim]
    ACTION_rdc = np.transpose(np.matmul(np.transpose(u1_reduce), np.transpose(ACTION)))
    u_te, s_te, v_te = np.linalg.svd(np.cov(np.transpose(ACTION_rdc)))
    u11, s11, v11 = np.linalg.svd(np.cov(np.transpose(STATE)))
    u11_reduce = u11[:, :rdc_dim]
    STATE_rdc = np.transpose(np.matmul(np.transpose(u11_reduce), np.transpose(STATE)))
    u_te1, s_te1, v_te1 = np.linalg.svd(np.cov(np.transpose(STATE_rdc)))
    print("A score =", sum(s_te)/sum(s1))     # A good score of PCA should be no less than 0.99
    print("S score =", sum(s_te1) / sum(s11))
    print(np.shape(ACTION_rdc))

    n_components = 4
    # n_components = 5
    GMM = GaussianMixture(n_components=n_components).fit(ACTION)  # Instantiate and fit the model
    print('Converged:', GMM.converged_)  # Check if the model has converged
    means = GMM.means_
    covariances = GMM.covariances_
    weights = GMM.weights_
    PARAS_L = []
    MEAN_L = []
    COV_L = []
    WEIGHT_L = []
    THETA_L = []
    for m, c, w in zip(means, covariances, weights):
        # print(m, c, w)
        MEAN_L.append(m)
        COV_L.append(c)
        WEIGHT_L.append(w)
        mus = np.tile(m, [len(ACTION), 1])
        # theta = np.matmul(np.linalg.pinv(STATE), mus)
        theta = np.matmul(np.linalg.pinv(STATE), mus)
        THETA_L.append(theta)
        paras = [theta, c, w]
        PARAS_L.append(paras)
    # print(len(PARAS_L), PARAS_L)  # print parameters of each GMM component in form of [theta, cov, weight]
    print(MEAN_L)
    print(COV_L)
    print(WEIGHT_L)

    ## Store the GMM parameters with n_components = 4
    writefile(MEAN_L, "GMM_parameters_4_force.txt")
    writefile(COV_L, "GMM_parameters_4_force.txt", edit="a")
    writefile(WEIGHT_L, "GMM_parameters_4_force.txt", edit="a")

    # x,y = np.meshgrid(np.sort(ACTION_rdc[:,0]),np.sort(ACTION_rdc[:,1]))
    # XY = np.array([x.flatten(),y.flatten()]).T
    # # Plot
    # fig = plt.figure(figsize=(10, 10))
    # ax0 = fig.add_subplot(111)
    # ax0.scatter(ACTION_rdc[:, 0], ACTION_rdc[:, 1], c='g',s=3, alpha=0.3)
    # ax0.scatter(Y[0, :], Y[1, :], c='orange', zorder=10, s=100)
    # for m, c in zip(means, covariances):
    #     multi_normal = multivariate_normal(mean=m, cov=c)
    #     ax0.contour(np.sort(ACTION_rdc[:, 0]), np.sort(ACTION_rdc[:, 1]),
    #                 multi_normal.pdf(XY).reshape(len(ACTION_rdc), len(ACTION_rdc)), colors='red', alpha=0.7)
    #     ax0.scatter(m[0], m[1], c='black', zorder=10, s=100)


    # ## High level policy
    # f = open(os.path.join(this_dir, "document", "Data_high.txt"), "r")
    # # fh = f.readlines()[0:11180*60]  #[0:1118000:400]
    # # fh = f.readlines()
    # # f1 = fh[11180 * 0:11180 * 40]
    # # f2 = fh[11180 * 60:11180 * 80]
    # # fh = f1 + f2
    # # fh = f.readlines()
    # # f1 = fh[11180 * 0:11180 * 20]
    # # f2 = fh[11180 * 40:11180 * 80]
    # # fh = f1 + f2
    # fh = f.readlines()[11180 * 20:11180 * 80]
    # S_H = []
    # S_L = []
    # n = 0
    # for string in fh:
    #     n += 1
    #     strlist = string.replace("(", "").replace(")", "")
    #     data = eval(strlist)
    #     state_now = []
    #     state_h = []
    #     for i in range(6):
    #         state_h.append(data[i])
    #     for j in range(12,len(data)):
    #         state_h.append(data[j])
    #     # print(state)
    #     S_H.append(state_h)
    #     state_l = []
    #     for k in range(6,12):
    #         state_l.append(data[k])
    #     S_L.append(state_l)
    #     # print(n)
    # STATE_H = np.asarray(S_H)
    # STATE_L = np.asarray(S_L)
    # rdc_dim = 2
    # u4, s4, v4 = np.linalg.svd(np.cov(np.transpose(STATE_L)))
    # u4_reduce = u4[:, :rdc_dim]
    # STATE_L_rdc = np.transpose(np.matmul(np.transpose(u4_reduce), np.transpose(STATE_L)))
    #
    # n_components = 4
    # GMM = GaussianMixture(n_components=n_components).fit(STATE_L)  # Instantiate and fit the model
    # print('Converged:', GMM.converged_) # Check if the model has converged
    # means = GMM.means_
    # covariances = GMM.covariances_
    # weights = GMM.weights_
    # PARAS_H = []
    # MEAN_H = []
    # COV_H = []
    # WEIGHT_H = []
    # THETA_H = []
    # for m, c, w in zip(means, covariances, weights):
    #     # print(m, c, w)
    #     MEAN_H.append(m)
    #     COV_H.append(c)
    #     WEIGHT_H.append(w)
    #     mus = np.tile(m, [len(STATE_L), 1])
    #     theta = np.matmul(np.linalg.pinv(STATE_H), mus)
    #     THETA_H.append(theta)
    #     paras = [theta, c, w]
    #     PARAS_H.append(paras)
    # # print(len(PARAS_L), PARAS_L)  # print parameters of each GMM component in form of [theta, cov, weight]
    # print(MEAN_H)
    # print(COV_H)
    # print(WEIGHT_H)
    #
    # ## Store the GMM parameters with n_components = 4
    # writefile(MEAN_H, "GMM_parameters_4_H.txt")
    # writefile(COV_H, "GMM_parameters_4_H.txt", edit="a")
    # writefile(WEIGHT_H, "GMM_parameters_4_H.txt", edit="a")
    #
    # # x,y = np.meshgrid(np.sort(STATE_L_rdc[:,0]),np.sort(STATE_L_rdc[:,1]))
    # # XY = np.array([x.flatten(),y.flatten()]).T
    # # # Plot
    # # fig = plt.figure(figsize=(10, 10))
    # # ax0 = fig.add_subplot(111)
    # # ax0.scatter(STATE_L_rdc[:, 0], STATE_L_rdc[:, 1], c='m',s=3, alpha=0.3)
    # # ax0.scatter(Y[0, :], Y[1, :], c='orange', zorder=10, s=100)
    # # for m, c in zip(means, covariances):
    # #     multi_normal = multivariate_normal(mean=m, cov=c)
    # #     ax0.contour(np.sort(STATE_L_rdc[:, 0]), np.sort(STATE_L_rdc[:, 1]),
    # #                 multi_normal.pdf(XY).reshape(len(STATE_L_rdc), len(STATE_L_rdc)), colors='blue', alpha=0.7)
    # #     ax0.scatter(m[0], m[1], c='black', zorder=10, s=100)
    # #
    # # plt.show()
