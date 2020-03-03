import numpy as np
import utiltools.robotmath as rm
import robotsim.ur3dual.ur3dualik as ur3dualik

class Ur3DualRobot():
    def __init__(self, position=np.zeros(3), rotmat=np.eye(3)):
        # initialize the ur3 dual-arm robot
        self.__name = 'ur3dual'
        self.__pos = np.array(position)
        self.__rotmat = np.array(rotmat)
        # initjnts has 15 elements where the first three are for the waist and head
        # the first three are dummy since the realrobot does not have those joints
        # after that, the first six are for the right arm
        # the remaining 6 are for the left arm
        self.__zerojnts = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], dtype='float64')
        self.__initjnts = np.array([0.0,0.0,0.0,-30.0,-90.0,120.0,0.0,120.0,0.0,30.0,-90.0,-120.0,180.0,-120.0,180.0], dtype='float64')
        # self.__initjnts = np.array([0.0,0.0,0.0,-60.0,-30.0,60.0,0.0,120.0,0.0,60.0,-150.0,-100.0,180.0,-120.0,180.0])
        # self.__initjnts = np.array([0.0,0.0,0.0,-30.0,0.0,-90.0,0.0,90.0,0.0,30.0,0.0,90.0,180.0,-90.0,180.0])
        self.__rgtarm = self.__initrgtarmj()
        self.__lftarm = self.__initlftarmj()
        self.__targetjoints = [1,2,3,4,5,6]
        self.__base = self.__rgtarm[0]
        self.goinitpose()

    @property
    def name(self):
        # read-only property
        return self.__name

    @property
    def initjnts(self):
        # read-only property
        return self.__initjnts

    @property
    def initrgtjnts(self):
        return np.array([self.__initjnts[i+2] for i in self.__targetjoints])

    @property
    def initlftjnts(self):
        return np.array([self.__initjnts[i+2+6] for i in self.__targetjoints])

    @property
    def initrgtjntsr(self):
        return np.array([self.__initjnts[0]]+[self.__initjnts[i+2] for i in self.__targetjoints])

    @property
    def initlftjntsr(self):
        return np.array([self.__initjnts[0]]+[self.__initjnts[i+2+6] for i in self.__targetjoints])

    @property
    def rgtarm(self):
        # read-only property
        return self.__rgtarm

    @property
    def lftarm(self):
        # read-only property
        return self.__lftarm

    @property
    def zerojnts(self):
        # read-only property
        return self.__zerojnts

    @property
    def base(self):
        # read-only property
        return self.__base

    @property
    def targetjoints(self):
        # read-only property
        return self.__targetjoints

    @targetjoints.setter
    def targetjoints(self, value):
        """
        change target joints
        NOTE: waist should not be included here since it is treated as an independent element

        :param value: a list
        :return:

        author: weiwei
        date: 20190402
        """

        nfrom = len(set(value).intersection(range(3,len(self.__initjnts))))
        if nfrom ==len(value):
            self.__targetjoints = value
        else:
            raise Exception("The elements of the value parameter must be from the joint ids!")

    def getarm(self, armname = 'rgt'):
        if armname == 'rgt':
            return self.rgtarm
        elif armname == 'lft':
            return self.lftarm

    def getinitarmjnts(self, armname = 'rgt'):
        """
        get init arm jnts by specifying armname

        :param armname:
        :return:

        date: 20180602 for ik udpate
        """

        if armname!="rgt" and armname!="lft":
            raise ValueError

        armjnts = self.initrgtjnts
        if armname == "lft":
            armjnts = self.initlftjnts

        return armjnts

    def movewaist(self, rotangle=0):
        """
        rotate the base of the robot

        :param rotangle: in degree
        :return: null

        author: weiwei
        date: 201701211
        """

        # setting right and left 0 is for compatibility
        # # right arm
        # self.rgtarm[0]['rotangle'] = rotangle
        # self.rgtarm[0]['rotmat'] = rm.rodrigues(self.rgtarm[0]['rotax'], self.rgtarm[0]['rotangle'])
        # self.rgtarm[0]['linkend'] = np.squeeze(np.dot(self.rgtarm[0]['rotmat'], self.rgtarm[0]['linkvec'].reshape((-1,))))+self.rgtarm[0]['linkpos']
        #
        # # left arm
        # self.lftarm[0]['rotangle'] = rotangle
        # self.lftarm[0]['rotmat'] = rm.rodrigues(self.lftarm[0]['rotax'], self.lftarm[0]['rotangle'])
        # self.lftarm[0]['linkend'] = np.squeeze(np.dot(self.lftarm[0]['rotmat'], self.lftarm[0]['linkvec'].reshape((-1,))))+self.lftarm[0]['linkpos']

        self.__updatefk(self.rgtarm)
        self.__updatefk(self.lftarm)

    def getarmjntsrng(self, armname='rgt'):
        """
        get jntsrnage by arm

        :param armname:
        :return:

        date: 20180602 for ik udpate
        """

        if armname!="rgt" and armname!="lft":
            raise ValueError

        jointlimits = []
        if armname=='rgt':
            for i in self.__targetjoints:
                jointlimits.append([self.rgtarm[i]['rngmin'], self.rgtarm[i]['rngmax']])
        elif armname=='lft':
            for i in self.__targetjoints:
                jointlimits.append([self.lftarm[i]['rngmin'], self.lftarm[i]['rngmax']])
        return jointlimits

    def movearmfk(self, armjnts, armname="rgt"):
        """
        move the joints of armlj specified by targetjoints using forward kinematics, waist is not included

        :param armjnts: a 1-by-6 ndarray where each element indicates the angle of a joint (in degree)
        :param armname: a string indicating the rgtlj or lftlj robot structure, could "lft" or "rgt"
        :return: null

        author: weiwei
        date: 20161205
        """

        if armname != "rgt" and armname != "lft":
            raise ValueError

        armlj = self.rgtarm
        if armname == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            armlj[i]['rotangle'] = armjnts[counter]
            counter += 1
        self.__updatefk(armlj)

    def movearmfkr(self, armjnts, armname ="rgt"):
        """
        move the redundant joints of armlj using forward kinematics

        :param armjnts: [waistrot, shoulderrot, armjnts6] all in angle
        :param armname: a string indicating the rgtlj or lftlj robot structure, could "lft" or "rgt"
        :return: null

        author: weiwei
        date: 20171211
        """

        if armname!="rgt" and armname!="lft":
            raise ValueError

        armlj = self.rgtarm
        if armname == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            armlj[i]['rotangle'] = armjnts[1][counter]
            counter += 1

        self.movewaist(armjnts[0])

    def movealljnts(self, robotjnts):
        """
        move all joints of the robot

        :param robotjnts: the same definition as self.initjntss
        :return: null

        author: weiwei
        date: 20171211
        """

        # right arm
        i = 1
        while i != -1:
            self.rgtarm[i]['rotangle'] = robotjnts[i+2]
            i = self.rgtarm[i]['child']
        # left arm
        i = 1
        while i != -1:
            self.lftarm[i]['rotangle'] = robotjnts[i+8]
            i = self.lftarm[i]['child']

        self.__updatefk(self.rgtarm)
        self.__updatefk(self.lftarm)

    def goinitpose(self):
        """
        move the robot to initial pose

        :return: null

        author: weiwei
        date: 20161211, Osaka
        """

        self.movealljnts(self.initjnts)

    def gozeropose(self):
        """
        move the robot to initial pose

        :return: null

        author: weiwei
        date: 20161211, Osaka
        """

        self.movealljnts(self.zerojnts)

    def gettcp(self, armname = 'rgt'):
        """
        get the tcppos, and tcprot of the speficied armname
        in the world coordinate system

        :return: [tcppos, tcprot] in nparray

        author: weiwei
        date: 20170412
        """

        armlj = self.rgtarm
        if armname == "lft":
            armlj = self.lftarm

        tcppos = armlj[-2]['linkend']
        tcprot = armlj[-2]['rotmat']
        return [tcppos, tcprot]

    def getee(self, armname = 'rgt'):
        """
        get the eepos, and eerot of the speficied armname
        in the world coordinate system
        
        :return: [eepos, eerot] in nparray

        author: weiwei
        date: 20190323
        """

        arm = self.rgtarm
        if armname == "lft":
            arm = self.lftarm

        eepos = arm[-1]['linkpos']
        eerot = arm[-1]['rotmat']

        return [eepos, eerot]

    def getarmjnts(self, armname="rgt"):
        """
        get the target joints of the specified armname

        :param armname:
        :return: armjnts: a 1-by-x numpy ndarray

        author: weiwei
        date: 20161205, tsukuba
        """

        if armname!="rgt" and armname!="lft":
            raise ValueError

        armlj = self.rgtarm
        if armname == "lft":
            armlj = self.lftarm

        armjnts = np.zeros(len(self.__targetjoints))
        counter = 0
        for i in self.__targetjoints:
            armjnts[counter] = armlj[i]['rotangle']
            counter += 1

        return armjnts

    def getwaist(self):
        """
        get the rot angle of robot waist

        :return: waistangle in degree

         author: weiwei
         date: 20170112
        """

        return self.base['rotangle']

    def chkrng(self, armjnts, armname="rgt"):
        """
        check if the given armjnts is inside the oeprating range of the speificed armname
        this function doesn't check the waist

        :param armjnts: a 1-by-x numpy ndarray indicating the targejoints of a manipulator
        :param armname: a string "rgt" or "lft"
        :return: True or False indicating inside the range or not

        author: weiwei
        date: 20161205
        """

        if armname!="rgt" and armname!="lft":
            raise ValueError

        armlj = self.rgtarm
        if armname == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            if armjnts[counter] < armlj[i]["rngmin"] or armjnts[counter] > armlj[i]["rngmax"]:
                print("Joint "+ str(i) + " of the " + armname + " arm is out of range")
                print("Angle is " + str(armjnts[counter]))
                print("Range is (" + str(armlj[i]["rngmin"]) + ", " + str(armlj[i]["rngmax"]) + ")")
                return False
            counter += 1

        return True

    def chkrngdrag(self, armjnts, armname="rgt"):
        """
        check if the given armjnts is inside the oeprating range of the speificed armname
        this function doesn't check the waist
        The joint angles out of range will be pulled back to their maxima

        :param armjnts: a 1-by-6 numpy ndarray indicating the targejoints of a manipulator
        :param armname: a string "rgt" or "lft"
        :return: Two parameters, one is true or false indicating if the joint angles are inside the range or not
                The other is the joint angles after draggin.
                If the joints were not dragged, the same joint angles will be returned

        author: weiwei
        date: 20161205
        """

        if armname!="rgt" and armname!="lft":
            raise ValueError

        armlj = self.rgtarm

        counter = 0
        bdragged = True
        jntanglesdrag = []
        for i in self.__targetjoints:
            if armjnts[counter] < armlj[i]["rngmin"]:
                bdragged = True
                jntanglesdrag.append(armlj[i]["rngmin"])
            elif armjnts[counter] > armlj[i]["rngmax"]:
                bdragged = True
                jntanglesdrag.append(armlj[i]["rngmax"])
            else:
                bdragged = False
                jntanglesdrag.append(armjnts[counter])

            counter += 1

        return bdragged, jntanglesdrag

    def __initrgtarmj(self):
        """
        Init rgt arm links and joints

        ## note
        x is facing forward, y is facing left, z is facing upward
        each element of rgtlj is a dictionary
        rgtlj[i]['linkpos'] indicates the position of a link
        rgtlj[i]['linkvec'] indicates the vector of a link that points from start to end
        rgtlj[i]['inherentR'] indicates the inherent mechanical rotation of the link
        rgtlj[i]['refcs"] indicates the reference coordinate system of the joint
        rgtlj[i]['rotmat'] indicates the frame of this link
        rgtlj[i]['rotax'] indicates the rotation axis of the link
        rgtlj[i]['rotangle'] indicates the rotation angle of the link around the rotax
        rgtlj[i]['linkend'] indicates the end position of the link (passively computed)

        rgtlj[i]['inherentR'] is the inherent rotation of the joint
        rgtlj[i]['refcs'] is the reference coordinate system of the joint

        ## more note:
        rgtlj[1]['linkpos'] is the position of the first joint
        rgtlj[i]['linkend'] is the same as rgtlj[i+1]['linkpos'],
        I am keeping this value for the eef (end-effector)

        ## even more note:
        joint is attached to the linkpos of a link
        for the first link, the joint is fixed and the rotax = 0,0,0

        ## inherentR is only available at the first link

        :return:
        rgtlj:
            a list of dictionaries with each dictionary holding name, mother, child,
        linkpos (link position), linkvec (link vector), rotmat (orientation), rotax (rotation axis of a joint),
        rotangle (rotation angle of the joint around rotax)
        linkend (the end position of a link) is computed using rotmat, rotax, linkpos, and linklen
        lj means link and joint, the joint attached to the link is at the linkend

        author: weiwei
        date: 20161202, tsukuba, 20190328, toyonaka
        """

        # create a arm with 6 joints
        rgtlj = [dict() for i in range(7)]
        rngsafemargin = 0

        # the 0th link and joint
        rgtlj[0]['name'] = 'link0'
        rgtlj[0]['mother'] = -1
        rgtlj[0]['child'] = 1
        rgtlj[0]['linkpos'] = self.__pos
        rgtlj[0]['linkvec'] = np.array([0, -258.485281374, 1610.51471863])+\
                              np.dot(rm.rodrigues([1,0,0],135), np.array([0,0,0])) #89.159
        rgtlj[0]['rotax'] = np.array([0,0,1])
        rgtlj[0]['rotangle'] = 0
        rgtlj[0]['inherentR'] = self.__rotmat
        rgtlj[0]['refcs'] = rgtlj[0]['inherentR']
        rgtlj[0]['rotmat'] = np.dot(rgtlj[0]['refcs'], rm.rodrigues(rgtlj[0]['rotax'], rgtlj[0]['rotangle']))
        rgtlj[0]['linkend'] = np.dot(rgtlj[0]['rotmat'], rgtlj[0]['linkvec'])+rgtlj[0]['linkpos']
        rgtlj[0]['rngmin'] = -(0-rngsafemargin)
        rgtlj[0]['rngmax'] = +(0-rngsafemargin)

        # the 1st joint and link
        rgtlj[1]['name'] = 'link1'
        rgtlj[1]['mother'] = 0
        rgtlj[1]['child'] = 2
        rgtlj[1]['linkpos'] = rgtlj[0]['linkend']
        rgtlj[1]['linkvec'] = np.array([0,-119.8,151.9])
        rgtlj[1]['rotax'] = np.array([0,0,1])
        rgtlj[1]['rotangle'] = 0
        rgtlj[1]['inherentR'] = np.dot(rm.rodrigues([1, 0, 0], 135), rm.rodrigues([0, 0, 1], -180))
        rgtlj[1]['refcs'] = np.dot(rgtlj[0]['rotmat'], rgtlj[1]['inherentR'])
        rgtlj[1]['rotmat'] = np.dot(rgtlj[1]['refcs'], rm.rodrigues(rgtlj[1]['rotax'], rgtlj[1]['rotangle']))
        rgtlj[1]['linkend'] = np.dot(rgtlj[1]['rotmat'], rgtlj[1]['linkvec'])+rgtlj[1]['linkpos']
        rgtlj[1]['rngmin'] = -(360-rngsafemargin)
        rgtlj[1]['rngmax'] = +(360-rngsafemargin)

        # the 2nd joint and link
        rgtlj[2]['name'] = 'link2'
        rgtlj[2]['mother'] = 1
        rgtlj[2]['child'] = 3
        rgtlj[2]['linkpos'] = rgtlj[1]['linkend']
        rgtlj[2]['linkvec'] = np.array([-243.65,92.5,0])
        rgtlj[2]['rotax'] = np.array([0,-1,0])
        rgtlj[2]['rotangle'] = 0
        rgtlj[2]['inherentR'] = np.eye(3)
        rgtlj[2]['refcs'] = np.dot(rgtlj[1]['rotmat'], rgtlj[2]['inherentR'])
        rgtlj[2]['rotmat'] = np.dot(rgtlj[2]['refcs'], rm.rodrigues(rgtlj[2]['rotax'], rgtlj[2]['rotangle']))
        rgtlj[2]['linkend'] = np.dot(rgtlj[2]['rotmat'], rgtlj[2]['linkvec'])+rgtlj[2]['linkpos']
        rgtlj[2]['rngmin'] = -(360-rngsafemargin)
        rgtlj[2]['rngmax'] = +(360-rngsafemargin)

        # the 3rd joint and link
        rgtlj[3]['name'] = 'link3'
        rgtlj[3]['mother'] = 2
        rgtlj[3]['child'] = 4
        rgtlj[3]['linkpos'] = rgtlj[2]['linkend']
        rgtlj[3]['linkvec'] = np.array([-213.25,0,0])
        rgtlj[3]['rotax'] = np.array([0,-1,0])
        rgtlj[3]['rotangle'] = 0
        rgtlj[3]['inherentR'] = np.eye(3)
        rgtlj[3]['refcs'] = np.dot(rgtlj[2]['rotmat'], rgtlj[3]['inherentR'])
        rgtlj[3]['rotmat'] = np.dot(rgtlj[3]['refcs'], rm.rodrigues(rgtlj[3]['rotax'], rgtlj[3]['rotangle']))
        rgtlj[3]['linkend'] = np.dot(rgtlj[3]['rotmat'], rgtlj[3]['linkvec'])+rgtlj[3]['linkpos']
        rgtlj[3]['rngmin'] = -(360-rngsafemargin)
        rgtlj[3]['rngmax'] = +(360-rngsafemargin)

        # the 4th joint and link
        rgtlj[4]['name'] = 'link4'
        rgtlj[4]['mother'] = 3
        rgtlj[4]['child'] = 5
        rgtlj[4]['linkpos'] = rgtlj[3]['linkend']
        rgtlj[4]['linkvec'] = np.array([0,-85.35,0])
        rgtlj[4]['rotax'] = np.array([0,-1,0])
        rgtlj[4]['rotangle'] = 0
        rgtlj[4]['inherentR'] = np.eye(3)
        rgtlj[4]['refcs'] = np.dot(rgtlj[3]['rotmat'], rgtlj[4]['inherentR'])
        rgtlj[4]['rotmat'] = np.dot(rgtlj[4]['refcs'], rm.rodrigues(rgtlj[4]['rotax'], rgtlj[4]['rotangle']))
        rgtlj[4]['linkend'] = np.dot(rgtlj[4]['rotmat'], rgtlj[4]['linkvec'])+rgtlj[4]['linkpos']
        rgtlj[4]['rngmin'] = -(360-rngsafemargin)
        rgtlj[4]['rngmax'] = +(360-rngsafemargin)

        # the 5th joint and link
        rgtlj[5]['name'] = 'link5'
        rgtlj[5]['mother'] = 4
        rgtlj[5]['child'] = 6
        rgtlj[5]['linkpos'] = rgtlj[4]['linkend']
        rgtlj[5]['linkvec'] = np.array([0,-81.9,-85])
        rgtlj[5]['rotax'] = np.array([0,0,-1])
        rgtlj[5]['rotangle'] = 0
        rgtlj[5]['inherentR'] = np.eye(3)
        rgtlj[5]['refcs'] = np.dot(rgtlj[4]['rotmat'], rgtlj[5]['inherentR'])
        rgtlj[5]['rotmat'] = np.dot(rgtlj[5]['refcs'], rm.rodrigues(rgtlj[5]['rotax'], rgtlj[5]['rotangle']))
        rgtlj[5]['linkend'] = np.dot(rgtlj[5]['rotmat'], rgtlj[5]['linkvec'])+rgtlj[5]['linkpos']
        rgtlj[5]['rngmin'] = -(360-rngsafemargin)
        rgtlj[5]['rngmax'] = +(360-rngsafemargin)

        # the 6th joint and link
        rgtlj[6]['name'] = 'link6'
        rgtlj[6]['mother'] = 5
        rgtlj[6]['child'] = -1
        rgtlj[6]['linkpos'] = rgtlj[5]['linkend']
        rgtlj[6]['linkvec'] = np.array([0,0,195.0])
        rgtlj[6]['rotax'] = np.array([0,0,1])
        rgtlj[6]['rotangle'] = 0
        rgtlj[6]['inherentR'] = rm.rodrigues([1,0,0],90)
        rgtlj[6]['refcs'] = np.dot(rgtlj[5]['rotmat'], rgtlj[6]['inherentR'])
        rgtlj[6]['rotmat'] = np.dot(rgtlj[6]['refcs'], rm.rodrigues(rgtlj[6]['rotax'], rgtlj[6]['rotangle']))
        rgtlj[6]['linkend'] = np.dot(rgtlj[6]['rotmat'], rgtlj[6]['linkvec'])+rgtlj[6]['linkpos']
        rgtlj[6]['rngmin'] = -(360-rngsafemargin)
        rgtlj[6]['rngmax'] = +(360-rngsafemargin)

        return rgtlj

    def __initlftarmj(self):
        """
        Init hrp5's lft arm links and joints

        ## note
        x is facing forward, y is facing left, z is facing upward
        each element of lftlj is a dictionary
        lftlj[i]['linkpos'] indicates the position of a link
        lftlj[i]['linkvec'] indicates the vector of a link that points from start to end
        lftlj[i]['rotmat'] indicates the frame of this link
        lftlj[i]['rotax'] indicates the rotation axis of the link
        lftlj[i]['rotangle'] indicates the rotation angle of the link around the rotax
        lftlj[i]['linkend'] indicates the end position of the link (passively computed)

        lftlj[i]['inherentR'] is the inherent rotation of the joint
        lftlj[i]['refcs'] is the reference coordinate system of the joint

        ## more note:
        lftlj[1]['linkpos'] is the position of the first joint
        lftlj[i]['linkend'] is the same as lftlj[i+1]['linkpos'],
        I am keeping this value for the eef (end-effector)

        ## even more note:
        joint is attached to the linkpos of a link
        for the first link, the joint is fixed and the rotax = 0,0,0

        :return:
        lftlj:
            a list of dictionaries with each dictionary holding name, mother, child,
        linkpos (link position), linkvec (link vector), rotmat (orientation), rotax (rotation axis of a joint),
        rotangle (rotation angle of the joint around rotax)
        linkend (the end position of a link) is computed using rotmat, rotax, linkpos, and linklen
        lj means link and joint, the joint attached to the link is at the linkend

        author: weiwei
        date: 20161202, tsukuba, 20190328, toyonaka
        """

        # create a arm with 6 joints
        lftlj = [dict() for i in range(7)]
        rngsafemargin = 0

        # the 0th link and joint
        lftlj[0]['name'] = 'link0'
        lftlj[0]['mother'] = -1
        lftlj[0]['child'] = 1
        lftlj[0]['linkpos'] = self.__pos
        lftlj[0]['linkvec'] = np.array([0, 258.485281374, 1610.51471863])+\
                              np.dot(rm.rodrigues([1,0,0], -135), np.array([0,0,0])) #89.159
        lftlj[0]['rotax'] = np.array([0,0,1])
        lftlj[0]['rotangle'] = 0
        lftlj[0]['inherentR'] = self.__rotmat
        lftlj[0]['refcs'] = lftlj[0]['inherentR']
        lftlj[0]['rotmat'] = np.dot(lftlj[0]['refcs'], rm.rodrigues(lftlj[0]['rotax'], lftlj[0]['rotangle']))
        lftlj[0]['linkend'] = np.dot(lftlj[0]['rotmat'], lftlj[0]['linkvec'])+lftlj[0]['linkpos']
        lftlj[0]['rngmin'] = -(0-rngsafemargin)
        lftlj[0]['rngmax'] = +(0-rngsafemargin)

        # the 1st joint and link
        lftlj[1]['name'] = 'link1'
        lftlj[1]['mother'] = 0
        lftlj[1]['child'] = 2
        lftlj[1]['linkpos'] = lftlj[0]['linkend']
        lftlj[1]['linkvec'] = np.array([0,-119.8,151.9])
        lftlj[1]['rotax'] = np.array([0,0,1])
        lftlj[1]['rotangle'] = 0
        lftlj[1]['inherentR'] = rm.rodrigues([1,0,0],-135)
        lftlj[1]['refcs'] = np.dot(lftlj[0]['rotmat'], lftlj[1]['inherentR'])
        lftlj[1]['rotmat'] = np.dot(lftlj[1]['refcs'], rm.rodrigues(lftlj[1]['rotax'], lftlj[1]['rotangle']))
        lftlj[1]['linkend'] = np.dot(lftlj[1]['rotmat'], lftlj[1]['linkvec'])+lftlj[1]['linkpos']
        lftlj[1]['rngmin'] = -(360-rngsafemargin)
        lftlj[1]['rngmax'] = +(360-rngsafemargin)

        # the 2nd joint and link
        lftlj[2]['name'] = 'link2'
        lftlj[2]['mother'] = 1
        lftlj[2]['child'] = 3
        lftlj[2]['linkpos'] = lftlj[1]['linkend']
        lftlj[2]['linkvec'] = np.array([-243.65,92.5,0])
        lftlj[2]['rotax'] = np.array([0,-1,0])
        lftlj[2]['rotangle'] = 0
        lftlj[2]['inherentR'] = np.eye(3)
        lftlj[2]['refcs'] = np.dot(lftlj[1]['rotmat'], lftlj[2]['inherentR'])
        lftlj[2]['rotmat'] = np.dot(lftlj[2]['refcs'], rm.rodrigues(lftlj[2]['rotax'], lftlj[2]['rotangle']))
        lftlj[2]['linkend'] = np.dot(lftlj[2]['rotmat'], lftlj[2]['linkvec'])+lftlj[2]['linkpos']
        lftlj[2]['rngmin'] = -(360-rngsafemargin)
        lftlj[2]['rngmax'] = +(360-rngsafemargin)

        # the 3rd joint and link
        lftlj[3]['name'] = 'link3'
        lftlj[3]['mother'] = 2
        lftlj[3]['child'] = 4
        lftlj[3]['linkpos'] = lftlj[2]['linkend']
        lftlj[3]['linkvec'] = np.array([-213.25,0,0])
        lftlj[3]['rotax'] = np.array([0,-1,0])
        lftlj[3]['rotangle'] = 0
        lftlj[3]['inherentR'] = np.eye(3)
        lftlj[3]['refcs'] = np.dot(lftlj[2]['rotmat'], lftlj[3]['inherentR'])
        lftlj[3]['rotmat'] = np.dot(lftlj[3]['refcs'], rm.rodrigues(lftlj[3]['rotax'], lftlj[3]['rotangle']))
        lftlj[3]['linkend'] = np.dot(lftlj[3]['rotmat'], lftlj[3]['linkvec'])+lftlj[3]['linkpos']
        lftlj[3]['rngmin'] = -(360-rngsafemargin)
        lftlj[3]['rngmax'] = +(360-rngsafemargin)

        # the 4th joint and link
        lftlj[4]['name'] = 'link4'
        lftlj[4]['mother'] = 3
        lftlj[4]['child'] = 5
        lftlj[4]['linkpos'] = lftlj[3]['linkend']
        lftlj[4]['linkvec'] = np.array([0,-85.35,0])
        lftlj[4]['rotax'] = np.array([0,-1,0])
        lftlj[4]['rotangle'] = 0
        lftlj[4]['inherentR'] = np.eye(3)
        lftlj[4]['refcs'] = np.dot(lftlj[3]['rotmat'], lftlj[4]['inherentR'])
        lftlj[4]['rotmat'] = np.dot(lftlj[4]['refcs'], rm.rodrigues(lftlj[4]['rotax'], lftlj[4]['rotangle']))
        lftlj[4]['linkend'] = np.dot(lftlj[4]['rotmat'], lftlj[4]['linkvec'])+lftlj[4]['linkpos']
        lftlj[4]['rngmin'] = -(360-rngsafemargin)
        lftlj[4]['rngmax'] = +(360-rngsafemargin)

        # the 5th joint and link
        lftlj[5]['name'] = 'link5'
        lftlj[5]['mother'] = 4
        lftlj[5]['child'] = 6
        lftlj[5]['linkpos'] = lftlj[4]['linkend']
        lftlj[5]['linkvec'] = np.array([0,-81.9,-85])
        lftlj[5]['rotax'] = np.array([0,0,-1])
        lftlj[5]['rotangle'] = 0
        lftlj[5]['inherentR'] = np.eye(3)
        lftlj[5]['refcs'] = np.dot(lftlj[4]['rotmat'], lftlj[5]['inherentR'])
        lftlj[5]['rotmat'] = np.dot(lftlj[5]['refcs'], rm.rodrigues(lftlj[5]['rotax'], lftlj[5]['rotangle']))
        lftlj[5]['linkend'] = np.dot(lftlj[5]['rotmat'], lftlj[5]['linkvec'])+lftlj[5]['linkpos']
        lftlj[5]['rngmin'] = -(360-rngsafemargin)
        lftlj[5]['rngmax'] = +(360-rngsafemargin)

        # the 6th joint and link
        lftlj[6]['name'] = 'link6'
        lftlj[6]['mother'] = 5
        lftlj[6]['child'] = -1
        lftlj[6]['linkpos'] = lftlj[5]['linkend']
        lftlj[6]['linkvec'] = np.array([0,0,195.0])
        lftlj[6]['rotax'] = np.array([0,0,1])
        lftlj[6]['rotangle'] = 0
        lftlj[6]['inherentR'] = rm.rodrigues([1,0,0],90)
        lftlj[6]['refcs'] = np.dot(lftlj[5]['rotmat'], lftlj[6]['inherentR'])
        lftlj[6]['rotmat'] = np.dot(lftlj[6]['refcs'], rm.rodrigues(lftlj[6]['rotax'], lftlj[6]['rotangle']))
        lftlj[6]['linkend'] = np.dot(lftlj[6]['rotmat'], lftlj[6]['linkvec'])+lftlj[6]['linkpos']
        lftlj[6]['rngmin'] = -(360-rngsafemargin)
        lftlj[6]['rngmax'] = +(360-rngsafemargin)

        return lftlj

    def __updatefk(self, armlj):
        """
        Update the structure of hrp5's arm links and joints (single)
        Note that this function should not be called explicitly
        It is called automatically by functions like movexxx

        :param armlj: the rgtlj or lftlj robot structure
        :return: null

        author: weiwei
        date: 20161202
        """

        i = 1
        while i != -1:
            j = armlj[i]['mother']
            armlj[i]['linkpos'] = armlj[j]['linkend']
            armlj[i]['refcs'] = np.dot(armlj[j]['rotmat'], armlj[i]['inherentR'])
            armlj[i]['rotmat'] = np.dot(armlj[i]['refcs'], rm.rodrigues(armlj[i]['rotax'], armlj[i]['rotangle']))
            armlj[i]['linkend'] = np.dot(armlj[i]['rotmat'], armlj[i]['linkvec']) + armlj[i]['linkpos']
            i = armlj[i]['child']
        return armlj

    def numik(self, objpos, objrot, armname="rgt"):
        return ur3dualik.numik(self, objpos, objrot, armname)

    def numikr(self, objpos, objrot, armname="rgt"):
        return ur3dualik.numikr(self, objpos, objrot, armname)

    def numikmsc(self, objpos, objrot, msc, armname="rgt"):
        return ur3dualik.numikmsc(self, objpos, objrot, msc, armname)

    def numikrmsc(self, objpos, objrot, msc, armname="rgt"):
        return ur3dualik.numikrmsc(self, objpos, objrot, msc, armname)

    def getworldpose(self, relpos, relrot, armname="rgt"):
        """
        given a relative pos and relative rot with respective to ee,
        get the world pos and world rot

        :param relpos: nparray 1x3
        :param relrot: nparray 3x3
        :return:

        author: weiwei
        date: 20190312
        """

        armlj = self.rgtarm
        if armname == "lft":
            armlj = self.lftarm
        objpos = armlj[-1]['linkend'] + np.dot(armlj[-1]['rotmat'], relpos)
        objrot = np.dot(armlj[-1]['rotmat'], relrot)
        return [objpos, objrot]

    def getinhandpose(self, worldpos, worldrot, armname="rgt"):
        """
        given a world pos and world rot,
        get a relative pos and relative rot with respective to ee

        :param worldpos: nparray 1x3
        :param worldrot: nparray 3x3
        :return:

        author: weiwei
        date: 20190312
        """

        armlj = self.rgtarm
        if armname == "lft":
            armlj = self.lftarm
        relpos, relrot = rm.relpose(armlj[-1]['linkend'], armlj[-1]['rotmat'], worldpos, worldrot)
        return [relpos, relrot]

if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    from direct.filter.CommonFilters import CommonFilters
    import ur3dualmesh
    import manipulation.grip.robotiq85.rtq85 as rtq85
    import ur3dualball

    base = pandactrl.World(camp = [3000,0,3000], lookatp = [0,0,700])

    ur3dualrobot = Ur3DualRobot()
    ur3dualrobot2 = Ur3DualRobot(position=np.array([1000,0,0]), rotmat=rm.rodrigues([0,0,1],170))
    ur3dualrobot.goinitpose()
    ur3dualrobot2.goinitpose()
    # ur3dualrobot.gozeropose()

    rgthnd = rtq85.Rtq85()
    lfthnd = rtq85.Rtq85()
    ur3mg = ur3dualmesh.Ur3DualMesh(rgthand=rgthnd, lfthand=lfthnd)
    pgg = pandageom.PandaGeomGen()
    # ur3stick = ur3mg.gensnp(ur3dualrobot)
    # ur3stick.reparentTo(base.render)
    #
    # # from manipulation.grip.hrp5three import hrp5threenm
    ur3dualmnp = ur3mg.genmnp(ur3dualrobot, togglejntscoord=False)
    ur3dualmnp2 = ur3mg.genmnp(ur3dualrobot2, togglejntscoord=False)
    ur3dualmnp.reparentTo(base.render)
    ur3dualmnp2.reparentTo(base.render)
    # ur3dualrobot.gozeropose()
    # ur3dualmnp = ur3mg.genmnp(ur3dualrobot, togglejntscoord=True)
    # ur3dualmnp.reparentTo(base.render)
    pgg.plotAxis(base.render, Vec3(0,0,0))
    ur3dualrobotball = ur3dualball.Ur3DualBall()
    bcndict = ur3dualrobotball.genfullbcndict(ur3dualrobot)
    ur3dualrobotball.showbcn(base, bcndict)
    base.run()
    #
    objpos = np.array([400,200,720])
    # objrot = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    objrot = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
    # objrot = np.array([[-1,0,0],[0,1,0],[0,0,-1]])
    # objrotmat4 = pg.npToMat4(objrot)
    # objrotmat4 = objrotmat4*Mat4.rotateMat(30, Vec3(1,0,0))
    # objrot = pg.mat3ToNp(objrotmat4.getUpper3())
    # objpos = np.array([401.67913818,-644.12841797,0])
    # objrot = np.array([[1.93558640e-06,-8.36298645e-01,5.48274219e-01],
    #                     [1.93560686e-06,-5.48274219e-01,-8.36298645e-01],
    #                     [1.00000000e+00,2.67997166e-06,5.57513317e-07]])
    # lfthnd
    # objpos = np.array([180,130,100])
    # objrot = np.array([[0,0,-1],[1,0,0],[0,-1,0]])
    # objpos = np.array([300.21112061, -50.61312485, 1056.34130859])
    # objrot = np.array([[9.46079314e-01, 3.23935062e-01, -9.43876017e-08],
    #                      [3.23935062e-01, -9.46079314e-01, -8.82536355e-09],
    #                      [-1.03894116e-07, -3.17865130e-08, -9.99999940e-01]])
    objpos = np.array([323.03422715, 317.46267233, 1053.93052481])
    objrot = np.array([[9.25048192e-02, 9.95708155e-01, 2.85119776e-03],
                        [9.94953796e-01, -9.25457215e-02, 3.87586538e-02],
                        [3.88561738e-02, -7.48552224e-04, -9.99244533e-01]])
    # armname="rgt"
    armname="lft"
    # armjntsgoal = hrp3robot.numikr(objpos, objrot, armname)
    # if armjntsgoal is not None:
    #     hrp3robot.movearmfkr(armjntsgoal, armname)
    #     hrp3plot.plotstick(base.render, hrp5robot)
    #     hrp3mnp = hrp3plot.genHrp5mnp(hrp5robot)
    #     hrp3mnp.reparentTo(base.render)
    import time
    tic = time.time()
    armjntsgoal = ur3dualrobot.numik(objpos, objrot, armname)
    toc = time.time()
    print(toc-tic)
    base.run()
    if armjntsgoal is not None:
        ur3dualrobot.movearmfk(armjntsgoal, armname)
        # ur3dualplot.plotstick(base.render, ur3dualrobot)
        ur3mnp = ur3mg.genmnp(ur3dualrobot, togglejntscoord=True)
        ur3mnp.reparentTo(base.render)

    # plot tableFalse
    import os
    this_dir, this_filename = os.path.split(__file__)
    table_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "obstacles", "table.egg"))
    table_model  = loader.loadModel(table_filepath)
    table_nodepath = NodePath("table")
    table_model.instanceTo(table_nodepath)
    table_nodepath.setPos(400,0,720)
    table_nodepath.reparentTo(base.render)

    # goal hand
    # from manipulation.grip.robotiq85 import rtq85nm
    # hrp3robotrgthnd = rtq85nm.Rtq85NM()
    # hrp3robotrgthnd.setColor([1,0,0,.3])
    # hrp3robotrgtarmlj9_rotmat = pandageom.cvtMat4(objrot, objpos+objrot[:,0]*130)
    # pg.plotAxisSelf(base.render, objpos, hrp5robotrgtarmlj9_rotmat)
    # hrp3robotrgthnd.setMat(hrp5robotrgtarmlj9_rotmat)
    # hrp3robotrgthnd.reparentTo(base.render)
    #
    # angle = nxtik.eurgtbik(objpos)
    # nxtrobot.movewaist(angle)
    # armjntsgoal=nxtik.numik(nxtrobot, objpos, objrot)
    #
    # # nxtplot.plotstick(base.render, nxtrobot)
    # pandamat4=Mat4()
    # pandamat4.setRow(3,Vec3(0,0,250))
    # # pg.plotAxis(base.render, pandamat4)
    # # nxtplot.plotmesh(base, nxtrobot)
    # # pandageom.plotAxis(base.render, pandageom.cvtMat4(nxtrobot.rgtarm[6]['rotmat'], nxtrobot.rgtarm[6]['linkpos']))
    # pg.plotDumbbell(base.render, objpos, objpos, rgba = [1,0,0,1])
    pgg.plotAxis(base.render, objpos, base.pg.npToMat4(objrot))
    # pg.plotArrow(base.render, hrp5robot.rgtarm[8]['linkpos'], hrp5robot.rgtarm[8]['linkpos']+hrp5robot.rgtarm[8]['rotax']*1000)
    #
    # # nxtrobot.movearmfk6(armjntsgoal)
    # # nxtmnp = nxtplot.genNxtmnp_nm(nxtrobot, plotcolor=[1,0,0,1])
    # # nxtmnp.reparentTo(base.render)

    base.run()