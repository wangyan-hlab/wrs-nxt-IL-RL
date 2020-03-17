# using Gaussian Distribution as the pdf
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal

if __name__ == '__main__':
    this_dir, this_filename = os.path.split(__file__)
    ## Low level policy
    # f = open(os.path.join(this_dir, "document", "Data_low.txt"), "r")
    # # fl = f.readlines()[2945*0:2945*60]
    # fl = f.readlines()
    # f1 = fl[2945 * 0:2945 * 40]
    # f2 = fl[2945 * 60:2945 * 80]  # [0:294500:50]
    # fl = f1 + f2
    # # fl = f.readlines()
    # # f1 = fl[2945*0:2945 * 20]
    # # f2 = fl[2945*40:2945*80]  #[0:294500:50]
    # # fl = f1 + f2
    # # fl = f.readlines()[2945*20:2945*80]
    # S = []
    # A = []
    # n = 0
    # for string in fl:
    #     n += 1
    #     strlist = string.replace("(", "").replace(")", "")
    #     data = eval(strlist)
    #     state = []
    #     for i in range(6):
    #         state.append(data[i])
    #     for j in range(18,len(data)):
    #         state.append(data[j])
    #     # print(state)
    #     S.append(state)
    #     action = []
    #     # for k in range(6,12):
    #     for k in range(12, 18):
    #         action.append(data[k])
    #     # print(action)
    #     A.append(action)
    #     # print(n)
    # STATE_L = np.asarray(S)
    # ACTION = np.asarray(A)
    # print(np.shape(STATE_L), np.shape(ACTION))
    #
    # rdc_dim = 2
    # u1, s1, v1 = np.linalg.svd(np.cov(np.transpose(ACTION)))
    # u1_reduce = u1[:, :rdc_dim]
    # ACTION_rdc = np.transpose(np.matmul(np.transpose(u1_reduce), np.transpose(ACTION)))
    # print(np.shape(ACTION_rdc))
    # a = np.linalg.inv(np.matmul(np.transpose(STATE_L), STATE_L))    #(X'*X)^-1
    # theta_l = np.matmul(np.matmul(a, np.transpose(STATE_L)), ACTION)    #(X'*X)^-1*X'*y
    # print(np.shape(theta_l), theta_l)
    # # A = np.matmul(STATE_L, theta_l)
    # # print(len(A))
    # # alist = []
    # # for action in A:
    # #     alist.append(action[0])
    # # print(len(alist), alist)
    # # start = 0
    # # x = np.linspace(0, 2945, len(alist[2945*start:2945*(start+1)]))
    # # plt.plot(x, alist[2945*start:2945*(start+1)])
    # # plt.ylim(-30,20)
    #
    # sigma_l = 1/np.shape(STATE_L)[1] * np.matmul(np.transpose(ACTION - np.matmul(STATE_L, theta_l)),
    #                                          ACTION - np.matmul(STATE_L, theta_l))
    # # print(np.shape(sigma_l))
    # # print(sigma_l)
    # mse = 0
    # for n in range(len(STATE_L)):
    #     a = np.matmul(STATE_L[n], theta_l)
    #     mse += sum((a.T - ACTION[n]) ** 2)
    # #     # print('demo =', ACTION[n])
    # #     # print('prediction =', a)
    # #
    # #     plt.figure(1)
    # #     plt.plot([1,2,3,4,5,6,7,8,9,10,11,12], ACTION[n], 'go-', [1,2,3,4,5,6,7,8,9,10,11,12], a, 'r.-')
    # print(mse/len(STATE_L))
    # # plt.show()
    #
    # # x, y = np.meshgrid(np.sort(ACTION_rdc[:2945*4, 0]), np.sort(ACTION_rdc[:2945*4, 1]))
    # # XY = np.array([x.flatten(), y.flatten()]).T
    # # fig = plt.figure(figsize=(10, 10))
    # # plt.axis([-360, -60, 350, 480])
    # # plt.scatter(ACTION_rdc[:2945*4, 0], ACTION_rdc[:2945*4, 1], c='g', s=3, alpha=0.3)
    # # print('m =', theta_l)
    # # print('c =', sigma_l)
    # # multi_normal = multivariate_normal(mean=theta_l, cov=sigma_l)
    # # plt.contour(np.sort(ACTION_rdc[:2945*4, 0]), np.sort(ACTION_rdc[:2945*4, 1]),
    # #             multi_normal.pdf(XY).reshape(len(ACTION_rdc), len(ACTION_rdc)), colors='red', alpha=0.7)
    # # plt.scatter(theta_l[0], theta_l[1], c='black', zorder=10, s=100)
    # # plt.show()



    ## High level policy
    f = open(os.path.join(this_dir, "document", "Data_high.txt"), "r")
    fh = f.readlines()
    S_H = []
    S_L = []
    n = 0
    for string in fh:
        n += 1
        strlist = string.replace("(", "").replace(")", "")
        data = eval(strlist)
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
        # print(action)
        S_L.append(state_l)
        # print(n)
    STATE_H = np.asarray(S_H)
    STATE_L = np.asarray(S_L)
    # print(np.shape(STATE_H), np.shape(STATE_L))
    a = np.linalg.inv(np.matmul(np.transpose(STATE_H), STATE_H))    #(X'*X)^-1
    theta_h = np.matmul(np.matmul(a, np.transpose(STATE_H)), STATE_L)    #(X'*X)^-1*X'*y
    print(np.shape(theta_h), theta_h)
    sigma_h = 1/np.shape(STATE_H)[1] * np.matmul(np.transpose(STATE_L - np.matmul(STATE_H, theta_h)),
                                               STATE_L - np.matmul(STATE_H, theta_h))
    # print(np.shape(sigma_h))
    mse = 0
    for n in range(0, 1118000):
        pred = np.matmul(STATE_H[n], theta_h)
        mse += sum((pred.T - STATE_L[n]) ** 2)
    #     print('demo =', STATE_L[n])
    #     print('prediction =', pred)
    #     plt.figure(2)
    #     plt.plot([1, 2, 3, 4, 5, 6], STATE_L[n], 'go-', [1, 2, 3, 4, 5, 6], pred, 'r.-')
    # print(sigma_h)
    print(mse/len(STATE_L))
    # plt.show()