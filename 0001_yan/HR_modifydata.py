# Data processing
import os
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.signal as ss
import csv
import utiltools.robotmath as rm
import pandaplotutils.pandageom as pg

# this_dir, this_filename = os.path.split(__file__)
# path = os.path.join(this_dir, "csv") #文件夹目录
# files = os.listdir(path) #得到文件夹下的所有文件名称
# n = 0
# for file in files: #遍历文件夹
#     n += 1
#     if not os.path.isdir(file): #判断是否是文件夹，不是文件夹才打开
#         f = open(os.path.join(this_dir, "csv", file), "r")
#         f1 = f.readlines()[1:]
#         strlist = []
#         checkpos = []
#         for line in f1:
#             items = line.strip().split()
#             item = ' '.join(items)
#             s = eval(item)
#             if [s[1], s[2], s[3], s[4], s[5], s[6]] != checkpos:
#                 data = str(s[1:])
#                 strlist.append(data.strip("(").strip(" ").replace(")", "").rstrip(" "))
#             checkpos = [s[1], s[2], s[3], s[4], s[5], s[6]]
#         f.close()
#
#         f = open(os.path.join(this_dir, "csv_new", file), "w")
#         f.write('time[sec],px_robot[mm],py_robot[mm],pz_robot[mm],prx_robot[deg],pry_robot[deg],prz_robot[deg],fx_robot[N],fy_robot[N],fz_robot[N],mx_robot[Nm],my_robot[Nm],mz_robot[Nm],fx_human[N],fy_human[N],fz_human[N],mx_human[Nm],my_human[Nm],mz_human[Nm]'+'\n')
#         time = 0
#         dt = datetime(2020, 3, 2, 20, 00)
#         starttime = dt.timestamp()
#         for pose in strlist:
#             time += 1
#             # datetime.fromtimestamp(starttime + round(time * 0.004, 3))
#             f.write(str(round(time*0.004, 3)) + ', ' + pose.strip("[").strip(" ").replace("]", "").rstrip(" ") + '\n')
#         f.close()
#     print("第%d个文件处理完毕" % n)
#-------------------------------------------------------------------------------------------------------

## Check the length of each data sample
# this_dir, this_filename = os.path.split(__file__)
# path = os.path.join(this_dir, "csv_new") #文件夹目录# files= os.listdir(path) #得到文件夹下的所有文件名称
# files = os.listdir(path) #得到文件夹下的所有文件名称
# s = []
# for file in files: #遍历文件夹
#      if not os.path.isdir(file): #判断是否是文件夹，不是文件夹才打开
#         f = open(path+"/"+file) #打开文件
#         iter_f = iter(f) #创建迭代器
#         # str = ""
#         n = 0
#         for line in iter_f: #遍历文件，一行行遍历，读取文本
#             n += 1
#         s.append(n) #每个文件的文本存到list中
# #
# print(s) #打印结果
# print(max(s), s.index(max(s)), min(s), s.index(min(s)))
# x = np.linspace(1, 100, 100)
# plt.plot(x, s)
# plt.show()
#-------------------------------------------------------------------------------------------------------

# this_dir, this_filename = os.path.split(__file__)
# path = os.path.join(this_dir, "csv_new") #文件夹目录
# files = os.listdir(path) #得到文件夹下的所有文件名称
# n = 0
# for file in files: #遍历文件夹
#     if not os.path.isdir(file): #判断是否是文件夹，不是文件夹才打开
#         series = pd.read_csv(path + '/' + file, header=0, parse_dates=[0], index_col=0, squeeze=True)
#         upsampled = series.resample('8ms')
#         # interpolated = pd.DataFrame(upsampled).interpolate(method='linear')
#         # print(series)
#         print(pd.DataFrame(upsampled, index=None, columns=None))
#-------------------------------------------------------------------------------------------------------

this_dir, this_filename = os.path.split(__file__)
path = os.path.join(this_dir, "csv_new") #文件夹目录
files = os.listdir(path) #得到文件夹下的所有文件名称
n = 0
for file in files: #遍历文件夹
    px = []
    py = []
    pz = []
    prx = []
    pry = []
    prz = []
    fx = []
    fy = []
    fz = []
    frx = []
    fry =[]
    frz = []
    if not os.path.isdir(file): #判断是否是文件夹，不是文件夹才打开
        f = open(os.path.join(this_dir, "csv_new", file), "r")
        f1 = f.readlines()[1:]
        strlist = []
        for line in f1:
            items = line.strip().split()
            item = ' '.join(items)
            s = eval(item)
            px.append(s[1])
            py.append(s[2])
            pz.append(s[3])
            prx.append(s[4])
            pry.append(s[5])
            prz.append(s[6])
            fx.append(s[7])
            fy.append(s[8])
            fz.append(s[9])
            frx.append(s[10])
            fry.append(s[11])
            frz.append(s[12])
            data = str(s[1:])
            strlist.append(data.strip("(").strip(" ").replace(")", "").rstrip(" "))

        resample_num = 300
        del_num = 20
        px_resample = ss.resample(np.array(px), resample_num+del_num)
        py_resample = ss.resample(np.array(py), resample_num+del_num)
        pz_resample = ss.resample(np.array(pz), resample_num+del_num)
        prx_resample = ss.resample(np.array(prx), resample_num+del_num)
        pry_resample = ss.resample(np.array(pry), resample_num+del_num)
        prz_resample = ss.resample(np.array(prz), resample_num+del_num)
        fx_resample = ss.resample(np.array(fx), resample_num+del_num)
        fy_resample = ss.resample(np.array(fy), resample_num+del_num)
        fz_resample = ss.resample(np.array(fz), resample_num+del_num)
        frx_resample = ss.resample(np.array(frx), resample_num+del_num)
        fry_resample = ss.resample(np.array(fry), resample_num+del_num)
        frz_resample = ss.resample(np.array(frz), resample_num+del_num)

        # 计算物体在机器人坐标系下的位姿
        pobj = []
        robj = []
        for i in range(resample_num+del_num):
            rot_joint = rm.euler_matrix(prx_resample[i], pry_resample[i], prz_resample[i])
            pos_joint = np.array([px_resample[i], py_resample[i], pz_resample[i]])
            joint_pose_to_base = rm.homobuild(pos_joint, rot_joint)
            rot_obj0 = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
            pos_obj = np.array([0, 21, 54 + 15 + 130 + 15])
            obj_pose_to_joint0 = rm.homobuild(pos_obj, rot_obj0)
            obj_pose_to_joint = np.dot(rm.homobuild(np.array([0, 0, 0]), rm.euler_matrix(0, 0, 225 + 22.5)),
                                       obj_pose_to_joint0)
            obj_pose_to_base = np.dot(joint_pose_to_base, obj_pose_to_joint)
            p_obj = obj_pose_to_base[:3, 3]
            # print(p_obj)
            pobj.append(p_obj)
            rpy_obj = rm.euler_from_matrix(pg.npToMat3(np.transpose(obj_pose_to_base[:3, :3])))
            # print(rpy_obj)
            robj.append(rpy_obj)

        # with open(os.path.join(this_dir, "resampledata", file), "w", newline='') as resamplefile:
        #     writer = csv.writer(resamplefile)
        #     writer.writerow(['px_robot[mm]', 'py_robot[mm]', 'pz_robot[mm]', 'prx_robot[deg]', 'pry_robot[deg]', 'prz_robot[deg]',
        #                      'fx_robot[N]', 'fy_robot[N]', 'fz_robot[N]', 'mx_robot[Nm]', 'my_robot[Nm]', 'mz_robot[Nm]',
        #                      'px_obj[mm]', 'py_obj[mm]', 'pz_obj[mm]', 'prx_obj[deg]', 'pry_obj[deg]', 'prz_obj[deg]'])
        #     for i in range(10, resample_num+del_num-10):
        #         writer.writerow([round(px_resample[i], 3), round(py_resample[i], 3), round(pz_resample[i], 3),
        #                          round(prx_resample[i], 3), round(pry_resample[i], 3), round(prz_resample[i], 3),
        #                          round(fx_resample[i], 3), round(fy_resample[i], 3), round(fz_resample[i], 3),
        #                          round(frx_resample[i], 3), round(fry_resample[i], 3), round(frz_resample[i], 3),
        #                          round(pobj[i][0], 3), round(pobj[i][1], 3), round(pobj[i][2], 3),
        #                          round(robj[i][0], 3), round(robj[i][1], 3), round(robj[i][2], 3)])
        # print("Resample data %s written" % file)

        # plt.figure(1)
        # # plt.subplot(211)
        # x = np.linspace(0, 40000, len(px), endpoint=False)
        # y = np.array(px)
        # y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        # x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        # plt.plot(x, y, 'go-', markersize=2, label='px')
        # # plt.scatter(x_new, y_new, c='red', s = 1, alpha=0.5, label='px_resample')
        # plt.plot(x_new, y_new, 'r.-', markersize=.5, label='px_resample')
        # plt.legend().set_draggable(True)
        # #
        # plt.figure(2)
        # x = np.linspace(0, 40000, len(py), endpoint=False)
        # y = np.array(py)
        # y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        # x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        # plt.plot(x, y, 'yo-', markersize=2, label='py')
        # plt.plot(x_new, y_new, 'b.-', markersize=.5,  label='py_resample')
        # plt.legend().set_draggable(True)
        # #
        # plt.figure(3)
        # x = np.linspace(0, 40000, len(pz), endpoint=False)
        # y = np.array(pz)
        # y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        # x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        # plt.plot(x, y, 'ko-', markersize=2, label='pz')
        # plt.plot(x_new, y_new, 'c.-', markersize=.5, label='pz_resample')
        # plt.legend().set_draggable(True)
        # #
        # plt.figure(4)
        # # plt.subplot(212)
        # x = np.linspace(0, 40000, len(prx), endpoint=False)
        # y = np.array(prx)
        # y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        # x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        # plt.plot(x, y, 'go-', markersize=2, label='prx')
        # plt.plot(x_new, y_new, 'r.-', markersize=.5, label='prx_resample')
        # plt.legend().set_draggable(True)
        #
        # plt.figure(5)
        # x = np.linspace(0, 40000, len(pry), endpoint=False)
        # y = np.array(pry)
        # y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        # # print(len(y_new), y_new)
        # x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        # plt.plot(x, y, 'yo-', markersize=2, label='pry')
        # plt.plot(x_new, y_new, 'b.-', markersize=.5, label='pry_resample')
        # plt.legend().set_draggable(True)
        #
        # plt.figure(6)
        # x = np.linspace(0, 40000, len(prz), endpoint=False)
        # y = np.array(prz)
        # y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        # x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        # plt.plot(x, y, 'ko-', markersize=2, label='prz')
        # plt.plot(x_new, y_new, 'c.-', markersize=.5, label='prz_resample')
        # plt.legend().set_draggable(True)
        #
        plt.figure(7)
        # plt.subplot(211)
        x = np.linspace(0, 40000, len(fx), endpoint=False)
        y = np.array(fx)
        y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        plt.plot(x, y, 'go-', markersize=5, label='fx')
        plt.plot(x_new, y_new, 'r.-', markersize=.5, label='fx_resample')
        plt.legend().set_draggable(True)
        #
        plt.figure(8)
        x = np.linspace(0, 40000, len(fy), endpoint=False)
        y = np.array(fy)
        y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        plt.plot(x, y, 'yo-', markersize=5, label='fy')
        plt.plot(x_new, y_new, 'b.-', markersize=.5, label='fy_resample')
        plt.legend().set_draggable(True)

        plt.figure(9)
        x = np.linspace(0, 40000, len(fz), endpoint=False)
        y = np.array(fz)
        y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        plt.plot(x, y, 'ko-', markersize=5, label='fz')
        plt.plot(x_new, y_new, 'c.-', markersize=.5, label='fz_resample')
        plt.legend().set_draggable(True)
        #
        plt.figure(10)
        plt.subplot(212)
        x = np.linspace(0, 40000, len(frx), endpoint=False)
        y = np.array(frx)
        y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        plt.plot(x, y, 'go-', markersize=5, label='mx')
        plt.plot(x_new, y_new, 'r.-', markersize=.5, label='mx_resample')
        plt.legend().set_draggable(True)
        #
        plt.figure(11)
        x = np.linspace(0, 40000, len(fry), endpoint=False)
        y = np.array(fry)
        y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        plt.plot(x, y, 'yo-', markersize=5, label='my')
        plt.plot(x_new, y_new, 'b.-', markersize=.5, label='my_resample')
        plt.legend().set_draggable(True)
        #
        plt.figure(12)
        x = np.linspace(0, 40000, len(frz), endpoint=False)
        y = np.array(frz)
        y_new = ss.resample(y, resample_num+del_num)[int(del_num/2):int(resample_num+del_num/2)]
        x_new = np.linspace(0, 40000, resample_num, endpoint=False)
        plt.plot(x, y, 'ko-', markersize=5, label='mz')
        plt.plot(x_new, y_new, 'c.-', markersize=.5, label='mz_resample')
        plt.legend().set_draggable(True)
        plt.show()

        f.close()
