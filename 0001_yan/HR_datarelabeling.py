import os
import csv

# def writefile(ls, filename):
#     this_dir, this_filename = os.path.split(__file__)
#     fl = open(os.path.join(this_dir, "document", filename), "w")
#     for poses in ls:
#         fl.write(str(poses)+'\n')
#     fl.close()

if __name__ == '__main__':
    this_dir, this_filename = os.path.split(__file__)
    path = os.path.join(this_dir, "resampledata") #文件夹目录
    files = os.listdir(path) #得到文件夹下的所有文件名称
    alltraj = []
    for file in files: #遍历文件夹
        if not os.path.isdir(file): #判断是否是文件夹，不是文件夹才打开
            f = open(path+"/"+file, "r")
            f1 = f.readlines()[1:]
            traj = []
            for line in f1:
                items = line.strip().split()
                item = ' '.join(items)
                s = eval(item)
                traj.append(s)
            # print(len(traj), traj)
            alltraj.append(traj)
    # print(len(alltraj), alltraj)
    ## data relabeling
    w_l = 10  # low level window
    w_h = 40  # high level window
    D_l = []
    D_h = []
    for traj in alltraj:
        for t in range(len(traj) - 1):
            action_low = []
            for w in range(1, w_l + 1):
                if t + w > len(traj) - 1:
                    break
                else:
                    D_l.append((traj[t][12:], traj[t+w][:12], traj[t + w][12:]))
                    ## state is represented with (pose, angle vel, angle acc),and pose is (x, z, P) because y=R=Y=0.0
    # writefile(D_l, "Data_low.txt")
    with open(os.path.join(this_dir, "document", "Data_low.csv"), "w", newline='') as datalow:
        writer = csv.writer(datalow)
        for ld in D_l:
            writer.writerow([ld[0][0],ld[0][1],ld[0][2],ld[0][3],ld[0][4],ld[0][5],
                            ld[1][0],ld[1][1],ld[1][2],ld[1][3],ld[1][4],ld[1][5],
                            ld[1][6],ld[1][7],ld[1][8],ld[1][9],ld[1][10],ld[1][11],
                            ld[2][0],ld[2][1],ld[2][2],ld[2][3],ld[2][4],ld[2][5]])

    print(len(D_l))  # len(D_l) = 294500
    for traj in alltraj:
        for t in range(len(traj) - 1):
            action_high = []
            for w in range(1, w_h + 1):
                if t + w > len(traj) - 1:
                    break
                else:
                    D_h.append((traj[t][12:], traj[t + min(w, w_l)][12:], traj[t + w][12:]))
                    ## state is represented with (pose, angle vel, angle acc),and pose is (x, z, P) because y=R=Y=0.0
    # writefile(D_h, "Data_high.txt")
    with open(os.path.join(this_dir, "document", "Data_high.csv"), "w", newline='') as datahigh:
        writer = csv.writer(datahigh)
        for hd in D_h:
            writer.writerow([hd[0][0],hd[0][1],hd[0][2],hd[0][3],hd[0][4],hd[0][5],
                            hd[1][0],hd[1][1],hd[1][2],hd[1][3],hd[1][4],hd[1][5],
                            hd[2][0],hd[2][1],hd[2][2],hd[2][3],hd[2][4],hd[2][5]])

    print(len(D_h)) # len(D_h) = 1118000