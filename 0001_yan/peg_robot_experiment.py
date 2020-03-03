import os
import robotcon.rpc.nxtrobot.nxtrobot_client as nxt
import math
import time

def readfile(filename, startline, endline):
    this_dir, this_filename = os.path.split(__file__)
    f = open(os.path.join(this_dir, "document", filename), "r")
    f1 = f.readlines()[startline: endline]
    poselist = []
    for string in f1:
        strlist = string.strip("[").strip(" ").replace("]", "").rstrip(" ")
        # s = " ".join(strlist.split()).replace(" ", ",")
        s = " ".join(strlist.split())
        # print(s)
        list1 = eval(s)
        poselist.append([list1[0], list1[1], list1[2],
                         list1[3], list1[4], list1[5], list1[6], list1[7], list1[8],
                         list1[9], list1[10], list1[11], list1[12], list1[13], list1[14]])
    return poselist

if __name__ == '__main__':

    nxtu = nxt.NxtRobot(host="10.0.1.114:15005")
    # nxtu.goInitial()
    # nxtu.goOffPose()
    # # -------------- PREPARE TO GRIP ------------------- #
    # prepare_angles = readfile("Planned_pegpath_deriv2_robot.txt", 0, 26)
    # print(len(prepare_angles), prepare_angles)
    # t = []
    # for i in range(len(prepare_angles)):
    #     t.append(0.3)
    # print(t)
    # nxtu.playPattern(prepare_angles, t)
    # -------------- GRIP THE OBJ HERE!!! ------------------- #
    # nxtu.closeHandToolRgt()
    # # -------------- LIFT THE OBJ ------------------- #
    # lift_angles = readfile("Planned_pegpath_deriv2_robot.txt", 26, 38)
    # print(len(lift_angles), lift_angles)
    # t = []
    # for i in range(len(lift_angles)):
    #     t.append(0.6)
    # print(t)
    # nxtu.playPattern(lift_angles, t)
    # # -------------- ROTATE THE ROBOT ------------------- #
    # rot_angles = readfile("Planned_pegpath_deriv2_robot.txt", 38, 48)
    # print(len(rot_angles), rot_angles)
    # t = []
    # for i in range(len(rot_angles)):
    #     t.append(1)
    # print(t)
    # nxtu.playPattern(rot_angles, t)
    # # -------------- GET CLOSE TO THE BASE ------------------- #
    # close_angles = readfile("Planned_pegpath_deriv2_robot.txt", 48, 59)
    # print(len(close_angles), close_angles)
    # t = []
    # for i in range(len(close_angles)):
    #     t.append(1)
    # print(t)
    # nxtu.playPattern(close_angles, t)
    # -------------- INSERT INTO THE BASE ------------------- #
    insert_angles = readfile("Planned_pegpath_deriv2_robot.txt", 59, 311)
    print(len(insert_angles), insert_angles)
    t = []
    for i in range(len(insert_angles)):
        t.append(0.3)
    print(t)
    nxtu.playPattern(insert_angles, t)
