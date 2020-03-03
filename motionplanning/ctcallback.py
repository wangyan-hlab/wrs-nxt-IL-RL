import numpy as np
import copy
import math

class CtCallback(object):
    """
    ctall back means constraint callback
    it accepts both cdchecker (necessary) and ctchecker
    and determines collision by considering both of them
    """

    def __init__(self, base, robot, cdchecker, ctchecker = None, armname = 'rgt', usewaist = False):
        """

        :param usewaist: is waist included
        :param cdchecker:

        author: weiwei
        date: 20180811
        """

        self.__base = base
        self.__robot = robot
        self.__cdchecker = cdchecker
        self.__ctchecker = ctchecker
        self.__usewaist = usewaist
        self.__armname = armname
        self.setarmname(self.__armname, self.__usewaist)

    @property
    def cdchecker(self):
        return self.__cdchecker

    @property
    def ctchecker(self):
        return self.__ctchecker

    @property
    def robot(self):
        return self.__robot

    @property
    def armname(self):
        return self.__armname

    @property
    def jointlimits(self):
        return self.__jointlimits

    def __genjntlimits(self):
        arm = self.__robot.rgtarm
        if self.__armname == 'lft':
            arm = self.__robot.lftarm
        jointlimits = []
        if self.__usewaist:
            jointlimits.append([self.__robot.base['rngmin'], self.__robot.base['rngmax']])
        for jntid in self.__robot.targetjoints:
            jointlimits.append([arm[jntid]['rngmin'], arm[jntid]['rngmax']])
        self.__jointlimits = jointlimits

    def setwaist(self, usewaist = False):
        self.__usewaist = usewaist
        self.setarmname(self.__armname, usewaist = usewaist)

    def setarmname(self, armname = 'rgt', usewaist = False):
        self.__armname = armname
        self.__usewaist = usewaist
        self.__genjntlimits()
        self.__initjnts = self.__robot.initrgtjnts
        if self.__armname == 'lft':
            self.__initjnts = self.__robot.initlftjnts
        self.__armlj = self.__robot.rgtarm
        if self.__armname == 'lft':
            self.__armlj = self.__robot.lftarm

    def iscollided(self, jnts, obstaclecmlist):
        """
        collision used for mp

        :param jnts:
        :param obstaclecmlist:
        :param armname:
        :return:

        author: weiwei
        date: 20180811
        """

        if self.__usewaist:
            self.__robot.movearmfkr([jnts[0], np.array(jnts[1:])], self.__armname)
        else:
            self.__robot.movearmfk(jnts, self.__armname)
        # NOTE, TRICK, avoid the other hand in case it is holding the object
        holdarmname = 'rgt'
        if self.__armname is 'rgt':
            holdarmname = 'lft'
        isrocd = self.__cdchecker.isCollided(self.__robot, obstaclecmlist, holdarmname=holdarmname)
        if self.__usewaist:
            self.__robot.movearmfkr([self.__robot.initjnts[0], np.array(self.__initjnts)], self.__armname)
        else:
            self.__robot.movearmfk(self.__initjnts, self.__armname)

        if isrocd:
            return True
        else:
            if self.__ctchecker is not None:
                isctviolated = self.__ctchecker(self.robot, self.armname, jnts, obstaclecmlist, None, None)
                if isctviolated:
                    return True
                else: return False
            else:
                return False

    def iscollidedHold(self, jnts, objcmlist, relmatlist, obstaclecmlist):
        """
        collision used for mp

        :param jnts:
        :param obstaclecmlist:
        :param armname:
        :return:

        author: weiwei
        date: 20180811
        """

        if self.__usewaist:
            self.__robot.movearmfkr([jnts[0], np.array(jnts[1:])], self.__armname)
        else:
            self.__robot.movearmfk(jnts, self.__armname)
        eepos = self.__armlj[-1]['linkend']
        eerot = self.__armlj[-1]['rotmat']
        objcmlistcopy = copy.deepcopy(objcmlist)
        for id, objcmcopy in enumerate(objcmlistcopy):
            relpos = relmatlist[id][0]
            relrot = relmatlist[id][1]
            objpos = eepos + np.dot(eerot, relpos)
            objrot = np.dot(eerot, relrot)
            objcmcopy.setMat(self.__base.pg.npToMat4(objrot, objpos))
        isrocd = self.__cdchecker.isCollided(self.__robot, obstaclecmlist, holdarmname=None)
        isoocd = self.__cdchecker.isObjectsOthersCollided(objcmlistcopy, self.__robot, self.__armname, obstaclecmlist)
        if self.__usewaist:
            self.__robot.movearmfkr([self.__robot.initjnts[0], np.array(self.__initjnts)], self.__armname)
        else:
            self.__robot.movearmfk(self.__initjnts, self.__armname)

        if isrocd or isoocd:
            return True
        else:
            if self.__ctchecker is not None:
                isctviolated = self.__ctchecker(self.robot, self.armname, jnts, obstaclecmlist, objcm, relmat)
                if isctviolated:
                    return True
                else: return False
            else:
                return False

    def isLMAvailablePOS(self, startpos, goalpos, rotmat, msc, discretedist = 10.0, armname = 'rgt'):
        """
        is linear motion available
        to protect the robot from isSecurityStopped

        :param startpos: 1x3 numpy array
        :param goalpos: 1x3 numpy array
        :param rotmat: 3x3 numpy array
        :param msc: current motion sequence configuration
        :param discretedist: 5.0mm by default
        :return:

        author: weiwei
        date: 20180821
        """

        mscopy = copy.deepcopy(msc)

        if self.__usewaist:
            self.__robot.movewaist(mscopy[0])
            startjnts = self.__robot.numikmsc(startpos, rotmat, mscopy[1], armname)
            if startjnts is None:
                return []
            startjnts = [mscopy[0], startjnts]
        else:
            startjnts = self.__robot.numikmsc(startpos, rotmat, mscopy, armname)
            if startjnts is None:
                return []

        returnjntslist = []
        vecsg = goalpos-startpos
        dist = np.linalg.norm(vecsg)
        if(dist<1e-6):
            return [startjnts]
        distvec = vecsg/dist
        njnts = int(math.ceil(dist/discretedist))
        for i in range(1,njnts):
            pos = startpos + i*distvec*discretedist
            if self.__usewaist:
                self.__robot.movewaist(mscopy[0])
                tmpjnts = self.__robot.numikmsc(pos, rotmat, mscopy[1], armname)
                if tmpjnts is None:
                    mscopy = None
                else:
                    mscopy = [mscopy[0], tmpjnts]
            else:
                mscopy = self.__robot.numikmsc(pos, rotmat, mscopy, armname)
            if mscopy is None:
                return []
            else:
                returnjntslist.append(mscopy)

        # append start and goal
        if self.__usewaist:
            self.__robot.movewaist(mscopy[0])
            goaljnts = self.__robot.numikmsc(goalpos, rotmat, mscopy[1], armname)
            if goaljnts is None:
                return []
            goaljnts = [mscopy[0], goaljnts]
        else:
            goaljnts = self.__robot.numikmsc(goalpos, rotmat, mscopy, armname)
            if goaljnts is None:
                return []
        returnjntslist = [startjnts]+returnjntslist+[goaljnts]
        return returnjntslist

    def isLMAvailableJNT(self, startjnts, goaljnts, discretedist = 10.0, armname = 'rgt'):
        """
        is linear motion available
        to protect the robot from isSecurityStopped

        :param startjnts: 1x6 joint angles in degree
        :param goaljnts: 1x6 joint angles in degree
        :param discretedist: distance to interplate
        :param armname:
        :return:

        author: weiwei
        date: 20180821
        """

        armlj = self.robot.rgtarm
        if armname == "lft":
            armlj = self.robot.lftarm

        movefunc = self.robot.movearmfk
        if self.__usewaist:
            movefunc = self.robot.movearmfkr

        msc = startjnts
        if self.__usewaist:
            savejnts = [self.robot.getwaist(), self.robot.getarmjnts(armname=armname)]
        else:
            savejnts = self.robot.getarmjnts(armname=armname)
        movefunc(startjnts, armname=armname)
        startpos = armlj[-1]['linkend']
        rotmat = armlj[-1]['rotmat']
        movefunc(goaljnts, armname=armname)
        goalpos = armlj[-1]['linkend']
        movefunc(savejnts)

        return self.isLMAvailablePOS(startpos, goalpos, rotmat, msc, discretedist, armname)

    def isLMAvailableJNTwithObj(self, startjnts, goaljnts, startobjpose, discretedist = 10.0, armname = 'rgt', type = "x"):
        """
        is linear motion available
        if available, both robot joints and object poses will be interplated
        in case of of type "c/w" the poses of the object are interplated together with joints
        in case of "o/x", the poses of the object are simply copies of the startobjpose

        :param startjnts: 1x6 joint angles in degree or waist+1x6 jnt angles
        :param goaljnts: 1x6 joint angles in degree
        :param startobjpose: [objpos, objrot]
        :param discretedist: distance to interplate
        :param armname:
        :param type could be "c/w" closed or "o/x" open, objpose will be changed if "c/w"
        :return: jntslist, objposelist = [[pos1x3, rot3x3], [], ...]

        author: weiwei
        date: 20190313
        """

        armlj = self.robot.rgtarm
        if armname == "lft":
            armlj = self.robot.lftarm

        movefunc = self.robot.movearmfk
        if self.__usewaist:
            movefunc = self.robot.movearmfkr

        msc = startjnts
        if self.__usewaist:
            savejnts = [self.robot.getwaist(), self.robot.getarmjnts(armname=armname)]
        else:
            savejnts = self.robot.getarmjnts(armname=armname)
        movefunc(startjnts, armname=armname)
        startpos = armlj[-1]['linkend']
        rotmat = armlj[-1]['rotmat']

        objrelpos, objrelrot = self.robot.getinhandpose(startobjpose[0], startobjpose[1], armname)

        movefunc(goaljnts, armname=armname)
        goalpos = armlj[-1]['linkend']
        returnjntslist = self.isLMAvailablePOS(startpos, goalpos, rotmat, msc, discretedist, armname)
        returnobjposelist = []
        if type is "c" or type is "w":
            for returnjnt in returnjntslist:
                movefunc(returnjnt, armname=armname)
                returnobjposelist.append(self.robot.getworldpose(objrelpos, objrelrot, armname))
        elif type is "x" or type is "o":
            for returnjnt in returnjntslist:
                returnobjposelist.append(startobjpose)
        else:
            print(type)
            raise Exception("Wrong hand type!")
        movefunc(savejnts)
        return [returnjntslist, returnobjposelist]

    # def iscollidedKP(self, jnts, obstaclecmlist):
    #     """
    #     collision at key poses
    #     hand cd is ignored
    #     collision used for keyposes
    #
    #     :param jnts:
    #     :param obstaclecmlist:
    #     :param armname:
    #     :return:
    #
    #     author: weiwei
    #     date: 20180811
    #     """
    #
    #     self.__robot.movearmfk(jnts, self.__armname)
    #     # NOTE: we ignore both arms here for conciseness
    #     # This might be a potential bug
    #     isrocd = self.__cdchecker.isCollided(self.__robot, obstaclecmlist, holdarm="all")
    #     self.__robot.movearmfk(self.__initjnts, self.__armname)
    #
    #     if isrocd:
    #         return True
    #     else:
    #         if self.__ctchecker is not None:
    #             isctviolated = self.__ctchecker(self.robot, self.armname, jnts, obstaclecmlist, None, None)
    #             if isctviolated:
    #                 return True
    #             else: return False
    #         else:
    #             return False
