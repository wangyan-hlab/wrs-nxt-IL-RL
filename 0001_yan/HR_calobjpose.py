# Calculate the object pose from the joint pose
import numpy as np
import utiltools.robotmath as rm
import pandaplotutils.pandageom as pg
import pandaplotutils.pandactrl as pc
import os


def readfile(filename, start=1):
    this_dir, this_filename = os.path.split(__file__)
    f = open(os.path.join(this_dir, "csv", filename), "r")
    f1 = f.readlines()[start:]
    poselist = []
    for string in f1:
        s = " ".join(string.split()).replace(" ", ",")
        # s = " ".join(strlist.split())
        # print(s)
        list1 = eval(s)
        poselist.append([list1[1], list1[2], list1[3], list1[4], list1[5], list1[6]])
    return poselist


if __name__ == '__main__':
    poselist = readfile("data20200227_164419.csv", start=1)
    base = pc.World(camp=[0, 3000, 0], lookatp=[200, 0, 0], up=[0, 0, 1], fov=40, w=1920, h=1080)
    for pose in poselist:
        # rot_joint = rm.euler_matrix(-135.523, -60.799, -52.4953)
        # pos_joint = np.array([399.265, 86.996, -176.666])
        # print(pose[3], pose[4], pose[5])
        print(pose)
        rot_joint = rm.euler_matrix(pose[3], pose[4], pose[5])
        # print("rot_joint =", rot_joint)
        pos_joint = np.array([pose[0], pose[1], pose[2]])
        joint_pose_to_base = rm.homobuild(pos_joint, rot_joint)
        # print("joint_pose_to_base =", joint_pose_to_base)

        rot_obj0 = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
        # print(rot_obj)
        pos_obj = np.array([0, 21, 54+15+130+15])
        obj_pose_to_joint0 = rm.homobuild(pos_obj, rot_obj0)
        obj_pose_to_joint = np.dot(rm.homobuild(np.array([0, 0, 0]), rm.euler_matrix(0, 0, 225+22.5)),
                                   obj_pose_to_joint0)
        # print(obj_pose_to_joint)
        obj_pose_to_base = np.dot(joint_pose_to_base, obj_pose_to_joint)
        # print("obj pose to base is", obj_pose_to_base)

        objpos0 = np.array([0, 0, 0])
        rot0 = rm.euler_matrix(0, 0, 0)
        base.pggen.plotAxis(nodepath=base.render, spos=objpos0, pandamat3=pg.npToMat3(rot0))
        base.pggen.plotAxis(nodepath=base.render, spos=joint_pose_to_base[:3, 3],
                            pandamat3=pg.npToMat3(joint_pose_to_base[:3, :3]), length=200)
        rpy_jnt = rm.euler_from_matrix((pg.npToMat3(np.transpose(joint_pose_to_base[:3, :3]))))
        print("joint euler angle to base=", rpy_jnt)
        base.pggen.plotAxis(nodepath=base.render, spos=obj_pose_to_base[:3, 3],
                            pandamat3=pg.npToMat3(obj_pose_to_base[:3, :3]), length=225)
        rpy_obj = rm.euler_from_matrix(pg.npToMat3(np.transpose(obj_pose_to_base[:3, :3])))
        print("obj euler angle to base=", rpy_obj)

        # rpy_obj_new = np.array([0, rpy_obj[1], 0])
        # print("obj euler angle to base=", rpy_obj_new)
        # base.pggen.plotAxis(nodepath=base.render, spos=obj_pose_to_base[:3, 3],
        #                     pandamat3=pg.npToMat3(rm.euler_matrix(rpy_obj_new[0], rpy_obj_new[1], rpy_obj_new[2])),
        #                     length=225)

    base.run()
