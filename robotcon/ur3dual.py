# !/usr/bin/env python

import logging
import math
import time

from robotcon.robotiq.rtq85 import Robotiq_Two_Finger_Gripper
from robotcon.robotiq.rtqft300 import Robotiq_FT300_Sensor
from utiltools import robotmath as rm
import urx.urrobot as urrobot
import robotcon.programbuilder as pb
import numpy as np

import socket
import struct
import copy
import os

class Ur3DualUrx():
    """
    urx 50, right arm 51, left arm 52

    author: weiwei
    date: 20180131
    """

    def __init__(self):
        iprgt = '10.2.0.50'
        iplft = '10.2.0.51'
        self.__rgtarm_ftsocket_ipad = (iprgt, 63351)
        self.__lftarm_ftsocket_ipad = (iplft, 63351)

        logging.basicConfig()
        self.__lftarm = urrobot.URRobot(iplft)
        self.__lftarm.set_tcp((0, 0, 0, 0, 0, 0))
        self.__lftarm.set_payload(1.28)
        self.__rgtarm = urrobot.URRobot(iprgt)
        self.__rgtarm.set_tcp((0, 0, 0, 0, 0, 0))
        self.__rgtarm.set_payload(1.28)

        self.__hand = Robotiq_Two_Finger_Gripper()

        self.__lftarmbase = [0, 235.00, 965.00]
        self.__rgtarmbase = [0, -235.00, 965.00]
        self.__sqrt2o2 = math.sqrt(2.0)/2.0

        self.__ftsensor = Robotiq_FT300_Sensor()
        self.__ftsensorscript = self.__ftsensor.ret_program_to_run()

        # setup server socket
        ipurx = '10.2.0.100'
        self.__urx_urmdsocket_ipad = (ipurx, 50001)
        self.__urx_urmdsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__urx_urmdsocket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.__urx_urmdsocket.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
        self.__urx_urmdsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__urx_urmdsocket.bind(self.__urx_urmdsocket_ipad)
        self.__urx_urmdsocket.listen(5)

        self.__jointscaler = 1000000
        self.__pb = pb.ProgramBuilder()
        script_dir = os.path.dirname(__file__)
        rel_path = "urscripts/moderndriver.script"
        self.__pb.loadprog(os.path.join(script_dir, rel_path))
        # set up right arm urscript
        self.__rgtarm_urscript = self.__pb.ret_program_to_run()
        self.__rgtarm_urscript = self.__rgtarm_urscript.replace("parameter_ip", self.__urx_urmdsocket_ipad[0])
        self.__rgtarm_urscript = self.__rgtarm_urscript.replace("parameter_port", str(self.__urx_urmdsocket_ipad[1]))
        self.__rgtarm_urscript = self.__rgtarm_urscript.replace("parameter_jointscaler", str(self.__jointscaler))
        # set up left arm urscript
        self.__lftarm_urscript = self.__pb.ret_program_to_run()
        self.__lftarm_urscript = self.__lftarm_urscript.replace("parameter_ip", self.__urx_urmdsocket_ipad[0])
        self.__lftarm_urscript = self.__lftarm_urscript.replace("parameter_port", str(self.__urx_urmdsocket_ipad[1]))
        self.__lftarm_urscript = self.__lftarm_urscript.replace("parameter_jointscaler", str(self.__jointscaler))

    @property
    def rgtarm(self):
        # read-only property
        return self.__rgtarm

    @property
    def lftarm(self):
        # read-only property
        return self.__lftarm

    def opengripper(self, armname = 'rgt', speedpercentange = 255, forcepercentage = 50, fingerdistance = 85):
        """
        open the rtq85 hand on the arm specified by armname

        :param armname:
        :return:

        author: weiwei
        date: 20180220
        """

        targetarm = self.__rgtarm
        if armname == 'lft':
            targetarm = self.__lftarm
        self.__hand.open_gripper(speedpercentange, forcepercentage, fingerdistance)
        targetarm.send_program(self.__hand.ret_program_to_run())

    def closegripper(self, armname = 'rgt', speedpercentange = 255, forcepercentage = 50, fingerdistance = 0):
        """
        close the rtq85 hand on the arm specified by armname

        :param armname:
        :return:

        author: weiwei
        date: 20180220
        """

        targetarm = self.__rgtarm
        if armname == 'lft':
            targetarm = self.__lftarm
        self.__hand.close_gripper(speedpercentange, forcepercentage, fingerdistance)
        targetarm.send_program(self.__hand.ret_program_to_run())

    def recvft(self, armname = 'rgt'):
        """
        receive force torque values from robotiq ft300 sensor

        TODO: 1. formatting
        2. reset coordinates

        :param armname:
        :return: [fx, fy, fz, tx, ty, tz] in N and Nm

        author: weiwei
        date: 20180220
        """

        self.__rgtarm.send_program(self.__ftsensorscript)
        self.__lftarm.send_program(self.__ftsensorscript)

        rgtarm_ftsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rgtarm_ftsocket.connect(self.__rgtarm_ftsocket_ipad)
        lftarm_ftsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lftarm_ftsocket.connect(self.__lftarm_ftsocket_ipad)

        targetftsocket = rgtarm_ftsocket
        if armname == 'lft':
            targetftsocket = lftarm_ftsocket
        rawft = targetftsocket.recv(1024)
        # todo mean filter
        numlist = rawft[1:].split(')(')
        while len(numlist)<3:
            time.sleep(.01)
            rawft = targetftsocket.recv(1024)
            numlist = rawft[1:].split(')(')
            print("sleep .2 for more data")
        rawft = [float(i) for i in numlist[-2].split(' , ')]

        rgtarm_ftsocket.close()
        lftarm_ftsocket.close()

        return [rawft[0], rawft[1], rawft[2], rawft[3], rawft[4], rawft[5]]

    def movejntsin360(self):
        """
        the function move all joints back to -360,360
        due to multi R problem, the last joint could be out of 360
        this function moves the last joint back

        :return:

        author: weiwei
        date: 20180202
        """

        rgtarmjnts = ur3u.getjnts('rgt')
        lftarmjnts = ur3u.getjnts('lft')
        rgtarmjnts = rm.cvtRngPM360(rgtarmjnts)
        lftarmjnts = rm.cvtRngPM360(lftarmjnts)
        ur3u.movejntssgl(rgtarmjnts, armname='rgt')
        ur3u.movejntssgl(lftarmjnts, armname='lft')
        # print ur3u.getjnts('rgt', cvtrange=None)
        # print ur3u.getjnts('lft', cvtrange=None)

    def movejntssgl(self, joints, armname='rgt', radius = 0.01):
        """

        :param joints: a 1-by-6 list in degree
        :param armname:
        :return:

        author: weiwei
        date: 20170411
        """

        targetarm = self.__rgtarm
        if armname == 'lft':
            targetarm = self.__lftarm

        jointsrad = [math.radians(angdeg) for angdeg in joints]
        targetarm.movej(jointsrad, acc = 1, vel = 1, wait = True)
        # targetarm.movejr(jointsrad, acc = 1, vel = 1, radius = radius, wait = False)

    def movejntsall(self, joints):
        """
        move all joints of the ur5 dual-arm robot
        NOTE that the two arms are moved sequentially
        use wait=False for simultaneous motion

        :param joints:  a 1-by-12 vector in degree, 6 for right, 6 for left
        :return: bool

        author: weiwei
        date: 20170411
        """

        jointsrad = [math.radians(angdeg) for angdeg in joints[0:6]]
        self.__rgtarm.movej(jointsrad, wait = False)
        jointsrad = [math.radians(angdeg) for angdeg in joints[6:12]]
        self.__lftarm.movej(jointsrad, wait = False)

    def movetposesgl_cont(self, tposelist, armname='rgt', acc = 1, vel = .1, radius = 0.1, wait = True):
        """
        move robot continuously by inputing a list of tcp poses

        :param tposelist:
        :param armname:
        :param acc:
        :param vel:
        :param radius:
        :return:

        author: weiwei
        date: 20180420
        """

        targetarm = self.__rgtarm
        if armname == 'lft':
            targetarm = self.__lftarm

        targetarm.movels(tposelist, acc=acc, vel=vel, radius=radius, wait=wait, threshold=None)

    def movejntssgl_cont(self, jointslist, armname='rgt', vel = 1.0, inpfunc = "cubic"):
        """
        move robot continuously using servoj and urscript

        :param jointslist:
        :param armname:
        :param vel: time to move between adjacent joints, = expandis/speed, speed = degree/second
        :param inpfunc: call cubic by default, candidate values: cubic or quintic
        :return:

        author: weiwei
        date: 20180518
        """

        def cubic(t, timestamp, q0array, v0array, q1array, v1array):
            a0 = q0array
            a1 = v0array
            a2 = (-3*(q0array-q1array)-(2*v0array+v1array)*timestamp)/(timestamp**2)
            a3 = (2*(q0array-q1array)+(v0array+v1array)*timestamp)/(timestamp**3)
            qt = a0+a1*t+a2*(t**2)+a3*(t**3)
            vt = a1+2*a2*t+3*a3*(t**2)
            return qt.tolist(), vt.tolist()

        def quintic(t, timestamp, q0array, v0array,
                  q1array, v1array, a0array=np.array([0.0]*6), a1array=np.array([0.0]*6)):
            a0 = q0array
            a1 = v0array
            a2 = a0array/2.0
            a3 = (20*(q1array-q0array)-(8*v1array+12*v0array)*timestamp-
                  (3*a1array-a0array)*(timestamp**2))/(2*(timestamp**3))
            a4 = (30*(q0array-q1array)+(14*v1array+16*v0array)*timestamp+
                  (3*a1array-2*a0array)*(timestamp**2))/(2*(timestamp**4))
            a5 = (12*(q1array-q0array)-6*(v1array+v0array)*timestamp-
                  (a1array-a0array)*(timestamp**2))/(2*(timestamp**5))
            qt = a0+a1*t+a2*(t**2)+a3*(t**3)+a4*(t**4)+a5*(t**5)
            vt = a1+2*a2*t+3*a3*(t**2)+4*a4*(t**3)+5*a5*(t**4)
            return qt.tolist(), vt.tolist()

        if inpfunc != "cubic" and inpfunc != "quintic":
            raise ValueError("Interpolation functions must be cubic or quintic")
        inpfunccallback = cubic
        if inpfunc == "quintic":
            inpfunccallback = quintic

        timestep = 0.008
        timepathstep = vel
        timesstamplist = []
        speedsradlist = []
        jointsradlist = []
        for id, joints in enumerate(jointslist):
            jointsrad = [math.radians(angdeg) for angdeg in joints[0:6]]
            # print jointsrad
            jointsradlist.append(jointsrad)
            if id == 0:
                timesstamplist.append([0.0]*6)
            else:
                timesstamplist.append([timepathstep]*6)
            if id == 0 or id == len(jointslist)-1:
                speedsradlist.append([0.0]*6)
            else:
                thisjointsrad = jointsrad
                prejointsrad = [math.radians(angdeg) for angdeg in jointslist[id-1][0:6]]
                nxtjointsrad = [math.radians(angdeg) for angdeg in jointslist[id+1][0:6]]
                presarray = (np.array(thisjointsrad)-np.array(prejointsrad))/timepathstep
                nxtsarray = (np.array(nxtjointsrad)-np.array(thisjointsrad))/timepathstep
                # set to 0 if signs are different
                selectid = np.where((np.sign(presarray)+np.sign(nxtsarray))==0)
                sarray = (presarray+nxtsarray)/2.0
                sarray[selectid]=0.0
                # print presarray
                # print nxtsarray
                # print sarray
                speedsradlist.append(sarray.tolist())
        t = 0
        jointsradlist008 = []
        speedsradlist008 = []
        for idlist, timesstamp in enumerate(timesstamplist):
            if idlist == 0:
                continue
            timesstampnp = np.array(timesstamp)
            jointsradprenp = np.array(jointsradlist[idlist-1])
            speedsradprenp = np.array(speedsradlist[idlist-1])
            jointsradnp = np.array(jointsradlist[idlist])
            speedsradnp = np.array(speedsradlist[idlist])
            # reduce timestep in the last step to avoid overfitting
            if idlist == len(timesstamplist)-1:
                while t <= timesstampnp.max():
                    jsrad, vsrad = inpfunccallback(t, timesstampnp,
                                                   jointsradprenp, speedsradprenp,
                                                   jointsradnp, speedsradnp)
                    jointsradlist008.append(jsrad)
                    speedsradlist008.append(vsrad)
                    t = t+timestep/4
            else:
                while t <= timesstampnp.max():
                    jsrad, vsrad = inpfunccallback(t, timesstampnp,
                                                   jointsradprenp, speedsradprenp,
                                                   jointsradnp, speedsradnp)
                    jointsradlist008.append(jsrad)
                    speedsradlist008.append(vsrad)
                    t = t+timestep
                t = 0

        ## for debug (show the curves in pyplot)
        # import matplotlib.pyplot as plt
        # for id in range(6):
        #     plt.subplot(121)
        #     q0list008 = [joint[id] for joint in jointsradlist008]
        #     plt.plot(np.linspace(0,0.008*len(jointsradlist008),len(jointsradlist008)),q0list008)
        #     plt.subplot(122)
        #     v0list008 = [speed[id] for speed in speedsradlist008]
        #     plt.plot(np.linspace(0,0.008*len(speedsradlist008),len(speedsradlist008)),v0list008)
        # plt.show()

        # for jointsrad in jointsradlist008:
        #     print jointsrad
        # print len(jointsradlist008)

        arm = self.__rgtarm
        arm_urscript = self.__rgtarm_urscript
        if armname == 'lft':
            arm = self.__lftarm
            arm_urscript = self.__lftarm_urscript
        arm.send_program(arm_urscript)
        # accept arm socket
        urmdsocket, urmdsocket_addr = self.__urx_urmdsocket.accept()
        print("Connected by ", urmdsocket_addr)

        keepalive = 1
        for id, jointsrad in enumerate(jointsradlist008):
            if id == len(jointsradlist008)-1:
                keepalive = 0
            jointsradint = [int(jointrad*self.__jointscaler) for jointrad in jointsrad]
            bindata = struct.pack('!iiiiiii', jointsradint[0], jointsradint[1], jointsradint[2],
                                    jointsradint[3], jointsradint[4], jointsradint[5], keepalive)
            urmdsocket.send(bindata)
            time.sleep(0.002)

        urmdsocket.close()

    def movejntssgl_cont2(self, jointslist, armname='rgt', vel = 1.0, inpfunc = "cubic"):
        """
        move robot continuously using servoj and urscript
        movejntssgl_cont2 aims at smooth slow down motion

        :param jointslist:
        :param armname:
        :param vel: time to move between adjacent joints, = expandis/speed, speed = degree/second
        :param inpfunc: call cubic by default, candidate values: cubic or quintic
        :return:

        author: weiwei
        date: 20180606
        """

        def cubic(t, timestamp, q0array, v0array, q1array, v1array):
            a0 = q0array
            a1 = v0array
            a2 = (-3*(q0array-q1array)-(2*v0array+v1array)*timestamp)/(timestamp**2)
            a3 = (2*(q0array-q1array)+(v0array+v1array)*timestamp)/(timestamp**3)
            qt = a0+a1*t+a2*(t**2)+a3*(t**3)
            vt = a1+2*a2*t+3*a3*(t**2)
            return qt.tolist(), vt.tolist()

        def quintic(t, timestamp, q0array, v0array,
                  q1array, v1array, a0array=np.array([0.0]*6), a1array=np.array([0.0]*6)):
            a0 = q0array
            a1 = v0array
            a2 = a0array/2.0
            a3 = (20*(q1array-q0array)-(8*v1array+12*v0array)*timestamp-
                  (3*a1array-a0array)*(timestamp**2))/(2*(timestamp**3))
            a4 = (30*(q0array-q1array)+(14*v1array+16*v0array)*timestamp+
                  (3*a1array-2*a0array)*(timestamp**2))/(2*(timestamp**4))
            a5 = (12*(q1array-q0array)-6*(v1array+v0array)*timestamp-
                  (a1array-a0array)*(timestamp**2))/(2*(timestamp**5))
            qt = a0+a1*t+a2*(t**2)+a3*(t**3)+a4*(t**4)+a5*(t**5)
            vt = a1+2*a2*t+3*a3*(t**2)+4*a4*(t**3)+5*a5*(t**4)
            return qt.tolist(), vt.tolist()

        if inpfunc != "cubic" and inpfunc != "quintic":
            raise ValueError("Interpolation functions must be cubic or quintic")
        inpfunccallback = cubic
        if inpfunc == "quintic":
            inpfunccallback = quintic

        timepathstep = vel
        timepathsteplastratio = .25
        timesstamplist = []
        speedsradlist = []
        jointsradlist = []
        for id, joints in enumerate(jointslist):
            jointsrad = [math.radians(angdeg) for angdeg in joints[0:6]]
            # print jointsrad
            jointsradlist.append(jointsrad)
            if id == 0:
                timesstamplist.append([0.0]*6)
            else:
                timesstamplist.append([timepathstep]*6)
            if id == 0 or id == len(jointslist)-1:
                speedsradlist.append([0.0]*6)
            else:
                thisjointsrad = jointsrad
                prejointsrad = [math.radians(angdeg) for angdeg in jointslist[id-1][0:6]]
                nxtjointsrad = [math.radians(angdeg) for angdeg in jointslist[id+1][0:6]]
                presarray = (np.array(thisjointsrad)-np.array(prejointsrad))/timepathstep
                nxtsarray = (np.array(nxtjointsrad)-np.array(thisjointsrad))/timepathstep
                if id+1 == len(jointslist)-1:
                    nxtsarray = nxtsarray/timepathsteplastratio
                # set to 0 if signs are different
                selectid = np.where((np.sign(presarray)+np.sign(nxtsarray))==0)
                sarray = (presarray+nxtsarray)/2.0
                sarray[selectid]=0.0
                # print presarray
                # print nxtsarray
                # print sarray
                speedsradlist.append(sarray.tolist())
        t = 0
        timestep = 0.008
        jointsradlist008 = []
        speedsradlist008 = []
        for idlist, timesstamp in enumerate(timesstamplist):
            if idlist == 0:
                continue
            timesstampnp = np.array(timesstamp)
            jointsradprenp = np.array(jointsradlist[idlist-1])
            speedsradprenp = np.array(speedsradlist[idlist-1])
            jointsradnp = np.array(jointsradlist[idlist])
            speedsradnp = np.array(speedsradlist[idlist])
            while t <= timesstampnp.max():
                jsrad, vsrad = inpfunccallback(t, timesstampnp,
                                               jointsradprenp, speedsradprenp,
                                               jointsradnp, speedsradnp)
                jointsradlist008.append(jsrad)
                speedsradlist008.append(vsrad)
                t = t+timestep
            t = 0

        ## for debug (show the curves in pyplot)
        # import matplotlib.pyplot as plt
        # for id in range(6):
        #     plt.subplot(121)
        #     q0list008 = [joint[id] for joint in jointsradlist008]
        #     plt.plot(np.linspace(0,0.008*len(jointsradlist008),len(jointsradlist008)),q0list008)
        #     plt.subplot(122)
        #     v0list008 = [speed[id] for speed in speedsradlist008]
        #     plt.plot(np.linspace(0,0.008*len(speedsradlist008),len(speedsradlist008)),v0list008)
        # plt.show()

        # for jointsrad in jointsradlist008:
        #     print jointsrad
        # print len(jointsradlist008)

        arm = self.__rgtarm
        arm_urscript = self.__rgtarm_urscript
        if armname == 'lft':
            arm = self.__lftarm
            arm_urscript = self.__lftarm_urscript
        arm.send_program(arm_urscript)
        # accept arm socket
        urmdsocket, urmdsocket_addr = self.__urx_urmdsocket.accept()
        print("Connected by ", urmdsocket_addr)

        keepalive = 1
        for id, jointsrad in enumerate(jointsradlist008):
            # if id == len(jointsradlist008)-1:
            #     keepalive = 0
            jointsradint = [int(jointrad*self.__jointscaler) for jointrad in jointsrad]
            bindata = struct.pack('!iiiiiii', jointsradint[0], jointsradint[1], jointsradint[2],
                                    jointsradint[3], jointsradint[4], jointsradint[5], keepalive)
            urmdsocket.send(bindata)
            time.sleep(0.002)

        urmdsocket.close()

    def getjnts(self, armname = 'rgt'):
        """
        get the joint angles of the specified arm

        :param armname:
        :return:

        author: ochi
        date: 20180410
        """

        targetarm = self.__rgtarm
        if armname == 'lft':
            targetarm = self.__lftarm
        armjnts_radian = targetarm.getj()
        armjnts_degree = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i, ajr in enumerate(armjnts_radian):
            armjnts_degree[i] = math.degrees(ajr)

        return armjnts_degree

if __name__ == '__main__':
    import math3d
    import robotsim.ur3dual.ur3dual as u3d
    import robotsim.ur3dual.ur3dualmesh as u3dm
    import pandaplotutils.pandactrl as pc
    from manipulation.grip.robotiq85 import rtq85

    base = pc.World(camp = [3000,0,3000], lookatp = [0,0,700])

    rgthand = rtq85.Rtq85()
    lfthand = rtq85.Rtq85()

    ur3dualrobot = u3d.Ur3DualRobot()
    ur3dualrobot.goinitpose()
    ur3u = Ur3DualUrx()

    initpose = ur3dualrobot.initjnts
    initrgt = initpose[3:9]
    initlft = initpose[9:15]
    ur3u.movejntssgl(initrgt, armname='rgt')
    ur3u.movejntssgl(initlft, armname='lft')

    # goalrgt = copy.deepcopy(initrgt)
    # goalrgt[0] = goalrgt[0]-10.0
    # goalrgt1 = copy.deepcopy(initrgt)
    # goalrgt1[0] = goalrgt1[0]-5.0
    # goallft = copy.deepcopy(initlft)
    # goallft[0] = goallft[0]+10.0

    # ur3u.movejntssgl_cont([initrgt, goalrgt, goalrgt1], armname='rgt')
    #
    # postcp_robot, rottcp_robot =  ur3dualrobot.gettcp_robot(armname='rgt')
    # print math3d.Transform(rottcp_robot, postcp_robot).get_pose_vector()
    # print "getl ", ur3u.rgtarm.getl()
    #
    # postcp_robot, rottcp_robot =  ur3dualrobot.gettcp_robot(armname='lft')
    # print math3d.Transform(rottcp_robot, postcp_robot).get_pose_vector()
    # print "getl ", ur3u.lftarm.getl()
    #
    # # tcpsimrobot =  ur5dualrobot.lftarm[-1]['linkpos']
    # # print tcprobot
    # # print tcpsimrobot
    # u3dmgen = u3dm.Ur3DualMesh(rgthand, lfthand)
    # ur3dualmnp = u3dmgen.genmnp(ur3dualrobot, togglejntscoord=True)
    # ur3dualmnp.reparentTo(base.render)
    # # armname = 'rgt'
    # # armname = 'lft'
    # # ur3u.movejntsall(ur3dualrobot.initjnts)
    # # ur3u.movejntsin360()
    # # print ur3u.getjnts('rgt')
    # # print ur3u.getjnts('lft')

    base.run()
