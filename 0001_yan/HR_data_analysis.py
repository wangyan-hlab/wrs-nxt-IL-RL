import os
import numpy as np
import matplotlib.pyplot as plt
import sklearn.decomposition as skd

this_dir, this_filename = os.path.split(__file__)
## Low level policy
f = open(os.path.join(this_dir, "document", "Data_low.txt"), "r")
fl = f.readlines()
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
    # print(state)
    S.append(state)
    action = []
    for k in range(6, 12):
        action.append(data[k])
    # print(action)
    A.append(action)
    print(n)
STATE = np.asarray(S)
ACTION = np.asarray(A)
rdc_dim = 2
u1, s1, v1 = np.linalg.svd(np.cov(np.transpose(ACTION)))
u1_reduce = u1[:, :rdc_dim]
ACTION_rdc = np.transpose(np.matmul(np.transpose(u1_reduce), np.transpose(ACTION)))
for i in range(10):
    for n in range(2945*i, 2945*(i+1)):
        # a = np.matmul(STATE_L[n], theta_l)
        # print('demo =', ACTION[n])
        # print('prediction =', a)
        plt.figure(i+1)
        plt.scatter(ACTION_rdc[n][0], ACTION_rdc[n][1], s=3, color='g')
        # plt.xlim((-15, 20))
        # plt.ylim((-18, 15))
plt.scatter(ACTION_rdc[0][0], ACTION_rdc[0][1], s=3, color='g', label='ACTION(dim=2)')
plt.legend().set_draggable(True)
plt.show()



## High level policy
# f = open(os.path.join(this_dir, "document", "Data_high.txt"), "r")
# fh = f.readlines()
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
#     for j in range(12, len(data)):
#         state_h.append(data[j])
#     # print(state)
#     S_H.append(state_h)
#     state_l = []
#     for k in range(6, 12):
#         state_l.append(data[k])
#     S_L.append(state_l)
#     print(n)
# STATE_H = np.asarray(S_H)
# STATE_L = np.asarray(S_L)
# # print(np.shape(STATE_N), np.shape(STATE_L), np.shape(STATE_H))
# rdc_dim = 2
# u4, s4, v4 = np.linalg.svd(np.cov(np.transpose(STATE_L)))
# u4_reduce = u4[:, :rdc_dim]
# STATE_L_rdc = np.transpose(np.matmul(np.transpose(u4_reduce), np.transpose(STATE_L)))
# # a = np.linalg.inv(np.matmul(np.transpose(STATE_H), STATE_H))    #(X'*X)^-1
# # theta_h = np.matmul(np.matmul(a, np.transpose(STATE_H)), STATE_L)    #(X'*X)^-1*X'*y
# # print(np.shape(theta_h), theta_h)
# # sigma_h = 1/np.shape(STATE_H)[1] * np.matmul(np.transpose(STATE_L - np.matmul(STATE_H, theta_h)),
# #                                            STATE_L - np.matmul(STATE_H, theta_h))
# # print(np.shape(sigma_h))
# for i in range(10):
#     for n in range(11180*i, 11180*(i+1), 20):
#         # pred = np.matmul(STATE_H[n], theta_h)
#         # print('demo =', STATE_L[n])
#         # print('prediction =', pred)
#         # plt.figure(i+1)
#         plt.scatter(STATE_L_rdc[n][0], STATE_L_rdc[n][1], s=3, color='m')
# plt.scatter(STATE_L_rdc[0][0], STATE_L_rdc[0][1], s=3, color='m', label='STATE_L(dim=2)')
# # print(sigma_h)
# plt.legend().set_draggable(True)
#
# plt.show()
