import rpyc
import pickle
import time

# # nextage-left
conn = rpyc.connect("10.0.1.102", port=15010)
# # nextage-right
# conn = rpyc.connect("10.0.1.74", port=15010)

# conn.root.checkEncoders()
# conn.root.servoOn()
# conn.root.goInitial()
init_angles = pickle.loads(conn.root.getJointAngles())
print init_angles

# # set joint angles of chest
# init_angles[0:1] = [-4.3]
# angles = pickle.dumps(init_angles[0:1])
# gname = "torso"
# tm = 7
# conn.root.setJointAnglesOfGroup(gname, angles, tm, True)
# # set joint angles of head
# init_angles[1:3] = [0, 0]
# angles = pickle.dumps(init_angles[1:3])
# gname = "head"
# tm = 7
# conn.root.setJointAnglesOfGroup(gname, angles, tm, True)
# # set joint angles of rgtarm
init_angles[3:8] = [-20, 0, -143.2, 0, 50, 0, 0, 0]
angles = pickle.dumps(init_angles[3:8])
gname = "rarm"
tm = 7
conn.root.setJointAnglesOfGroup(gname, angles, tm)
# # set joint angles of lftarm
init_angles[9:15] = [20, 0, -143.2, 0, 50, 0, 0, 0]
angles = pickle.dumps(init_angles[9:15])
gname = "larm"
tm = 7
conn.root.setJointAnglesOfGroup(gname, angles, tm)

# # # set target pose of rgtarm joint
# lname = "RARM_JOINT5"
# pos = pickle.loads(conn.root.getCurrentPosition(lname))
# print pos
# rpy = pickle.loads(conn.root.getCurrentRPY(lname))
# print rpy
# gname = "rarm"
# tm = 7
# # pos[0] = 0.40
# pos[2] = 0.15
# print pos
# pos = pickle.dumps(pos)
# rpy = pickle.dumps(rpy)
# conn.root.setTargetPose(gname, pos, rpy, tm)

# para_w = pickle.dumps(["torso", [[0.3]], [1.0]])
# root.playPatternOfGroup(para_w)
# # para_all = pickle.dumps([[[0,0,0,-0.26,0,-2.50,0,0,0,0.26,0,-2.50,0,0,0]],[],[],[5.0]])
# para_group = pickle.dumps(["larm", [[0.26, 0, -2.50, 0, 0, 0],[0.30, 0, -2.50, 0, 0, 0]], [5.0,5.0]])
# # root.playPattern(para_all)
# root.playPatternOfGroup(para_group)

# conn.root.goOffPose()
