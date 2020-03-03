import numpy as np
import utiltools.robotmath as rm
import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
from robotsim.hrp5p import hrp5pik
from robotsim.hrp5p import hrp5plegik
import random

class Hrp5PRobot():
    def __init__(self):
        # initjnts has 21 elements where the first three are for the base and head,
        # the remaining 18 are for each of the two 9-dof arms
        self.__name = 'hrp5n'
        # initjnts[0] = waist, 0,0 = head, 0,45,-20,...-30,0,0 = rgt, 0,45,20,...,30,0,0 = lft
        self.__initjnts = np.array([0,0,0,0,45,-50,-45,-150,45,0,0,-90,0,45,50,45,-150,-45,0,0,-90], dtype="float64");
        # self.__initjnts = np.array([0,0,0,51.3,15.7,-15.3,-9.9,-101.1,-41.6,28.7,-37.0,23.9,-15.2,49.3,45.4,26.0,-170.0,-37.5,-6.4,21.5,209.5], dtype="float64");
        self.__initlegjnts = np.array([-5,0,-45,75,-30,0,5,0,-45,75,-30,0], dtype="float64");
        # self.__initjnts = np.array([0,0,0,0,45,-20,0,-75,0,-30,0,0,0,45,20,0,-75,0,30,0,0]);
        # self.__initjnts = np.array([0,0,0,45,-20,0,-64,82,-27,109,-111,0,45,20,0,-75,0,30,0,0]);
        # self.__initjnts = np.array([0,0,0,45,-20,0,-68,77,-32,109,-122,0,45,20,0,-75,0,30,0,0]);
        # self.__initjnts = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);

        self.__robotoriginpos = np.array([0, 0, 0])
        self.__robotoriginrotmat = np.eye(3)
        self.__holdinghand = "rgt"
        self.__objectpos = np.array([0, 0, 0])

        self.__rgtarm = self.__initrgtarmj()
        self.__lftarm = self.__initlftarmj()
        self.__rgtleg = self.__initrgtlegj()
        self.__lftleg = self.__initlftlegj()
        self.__chest = self.__initchest()

        # define the target joints for ik
        # waist 0 should not be in the list
        # self.__targetjoints = [2,3,4,5,6,9]
        self.__targetjoints = [1,2,3,4,5,6,7,8,9]
        self.__targetlegjoints = [1,2,3,4,5,6]
        # self.__sixjoints = [2,4,5,6,7,9]
        self.__base = self.__rgtarm[0]
        self.goinitpose()
        self.goinitlegpose()

    @property
    def name(self):
        # read-only property
        return self.__name

    @property
    def objectpos(self):
        # read-only property
        return self.__objectpos

    @property
    def initjnts(self):
        # read-only property
        return self.__initjnts

    @property
    def initrgtjntsr(self):
        # read-only property
        return np.array([self.__initjnts[i] for i in [0, 3, 4, 5, 6, 7, 8, 9, 10, 11]])

    @property
    def initlftjntsr(self):
        # read-only property
        return np.array([self.__initjnts[i] for i in [0, 12, 13, 14, 15, 16, 17, 18, 19, 20]])

    @property
    def initrgtjnts(self):
        # read-only property
        return np.array([self.__initjnts[i] for i in [3, 4, 5, 6, 7, 8, 9, 10, 11]])

    @property
    def initlftjnts(self):
        # read-only property
        return np.array([self.__initjnts[i] for i in [12, 13, 14, 15, 16, 17, 18, 19, 20]])

    @property
    def initrgtlegjnts(self):
        # read-only property
        return np.array([self.__initlegjnts[i - 1] for i in self.__targetlegjoints])

    @property
    def initlftlegjnts(self):
        # read-only property
        return np.array([self.__initjnts[i - 1 + 6] for i in self.__targetlegjoints])

    @property
    def chest(self):
        # read-only property
        return self.__chest

    @property
    def rgtarm(self):
        # read-only property
        return self.__rgtarm

    @property
    def lftarm(self):
        # read-only property
        return self.__lftarm

    @property
    def rgtleg(self):
        # read-only property
        return self.__rgtleg

    @property
    def lftleg(self):
        # read-only property
        return self.__lftleg

    @property
    def base(self):
        # read-only property
        return self.__base

    @property
    def targetjoints(self):
        # read-only property
        return self.__targetjoints

    @property
    def robotoriginrotmat(self):
        # read-only property
        return self.__robotoriginrotmat

    @property
    def robotoriginpos(self):
        # read-only property
        return self.__robotoriginpos

    @property
    def targetlegjoints(self):
        # read-only property
        return self.__targetlegjoints

    @property
    def holdinghand(self):
        # read-only property
        return self.__holdinghand

    def __initrgtarmj(self):
        """
        Init rgt arm links and joints

        ## note
        x is facing forward, y is facing left, z is facing upward
        each element of rgtlj is a dictionary
        rgtlj[i]['linkpos'] indicates the position of a link
        rgtlj[i]['linkvec'] indicates the vector of a link that points from start to end
        rgtlj[i]['rotmat'] indicates the frame of this link
        rgtlj[i]['rotax'] indicates the rotation axis of the link
        rgtlj[i]['rotangle'] indicates the rotation angle of the link around the rotax
        rgtlj[i]['linkend'] indicates the end position of the link (passively computed)

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
        date: 20161202, tsukuba

        Mass, center of mass (local coordinates), center of mass (global coordinates) added to the links

        author: Daniel
        date: 20180515
        """

        # create a arm with 10 joints
        rgtlj = [dict() for i in range(10)]
        rngsafemargin = 0

        # the 0th link and joint
        rgtlj[0]['name'] = 'link0'
        rgtlj[0]['mother'] = -1
        rgtlj[0]['child'] = 1
        rgtlj[0]['linkpos'] = self.robotoriginpos
        rgtlj[0]['linkvec'] = np.array([87.0, -80.0, 542.0])
        rgtlj[0]['rotax'] = np.array([0, 0, 1])
        rgtlj[0]['rotangle'] = 0
        rgtlj[0]['rotmat'] = np.dot(self.__robotoriginrotmat, np.eye(3))
        rgtlj[0]['linkend'] = np.dot(rgtlj[0]['rotmat'], rgtlj[0]['linkvec']) + rgtlj[0]['linkpos']
        rgtlj[0]['rngmin'] = -(90 - rngsafemargin)
        rgtlj[0]['rngmax'] = +(90 - rngsafemargin)
        rgtlj[0]['mass'] = 0
        rgtlj[0]['lcntrofmass'] = np.array([0, 0, 0])
        rgtlj[0]['gcntrofmass'] = np.dot(rgtlj[0]['rotmat'], rgtlj[0]['lcntrofmass']) + rgtlj[0]['linkpos']
        rgtlj[0]['isleg'] = False

        # the 1st joint and link
        rgtlj[1]['name'] = 'link1'
        rgtlj[1]['mother'] = 0
        rgtlj[1]['child'] = 2
        rgtlj[1]['linkpos'] = rgtlj[0]['linkend']
        rgtlj[1]['linkvec'] = np.array([0, -130, 0])
        rgtlj[1]['rotax'] = np.array([0, 0, 1])
        rgtlj[1]['rotangle'] = 0
        rgtlj[1]['rotmat'] = np.dot(rgtlj[0]['rotmat'], rm.rodrigues(rgtlj[1]['rotax'], rgtlj[1]['rotangle']))
        rgtlj[1]['linkend'] = np.dot(rgtlj[1]['rotmat'], rgtlj[1]['linkvec']) + rgtlj[1]['linkpos']
        rgtlj[1]['rngmin'] = -(20 - rngsafemargin)
        rgtlj[1]['rngmax'] = +(60 - rngsafemargin)
        rgtlj[1]['mass'] = 1.136130
        rgtlj[1]['lcntrofmass'] = 1000 * np.array([0.000000, -0.009400, 0.012520])
        rgtlj[1]['gcntrofmass'] = np.dot(rgtlj[1]['rotmat'], rgtlj[1]['lcntrofmass']) + rgtlj[1]['linkpos']
       # print(rgtlj[1]['linkpos'])

        # print(rgtlj[1]['gcntrofmass'])

        # the 2nd joint and link
        rgtlj[2]['name'] = 'link2'
        rgtlj[2]['mother'] = 1
        rgtlj[2]['child'] = 3
        rgtlj[2]['linkpos'] = rgtlj[1]['linkend']
        rgtlj[2]['linkvec'] = np.array([0, 0, 0])
        rgtlj[2]['rotax'] = np.array([0, 1, 0])
        rgtlj[2]['rotangle'] = 0
        rgtlj[2]['rotmat'] = np.dot(rgtlj[1]['rotmat'], rm.rodrigues(rgtlj[2]['rotax'], rgtlj[2]['rotangle']))
        rgtlj[2]['linkend'] = np.dot(rgtlj[2]['rotmat'], rgtlj[2]['linkvec']) + rgtlj[2]['linkpos']
        rgtlj[2]['rngmin'] = -(200 - rngsafemargin)
        rgtlj[2]['rngmax'] = +(70 - rngsafemargin)
        rgtlj[2]['mass'] = 0.575550
        rgtlj[2]['lcntrofmass'] = 1000 * np.array([-0.011520, 0.003790, 0.008220])
        rgtlj[2]['gcntrofmass'] = np.dot(rgtlj[2]['rotmat'], rgtlj[2]['lcntrofmass']) + rgtlj[2]['linkpos']
        # print(rgtlj[2]['gcntrofmass'])

        # the 3rd joint and link
        rgtlj[3]['name'] = 'link3'
        rgtlj[3]['mother'] = 2
        rgtlj[3]['child'] = 4
        rgtlj[3]['linkpos'] = rgtlj[2]['linkend']
        rgtlj[3]['linkvec'] = np.array([0, -20, 0])
        rgtlj[3]['rotax'] = np.array([1, 0, 0])
        rgtlj[3]['rotangle'] = 0
        rgtlj[3]['rotmat'] = np.dot(rgtlj[2]['rotmat'], rm.rodrigues(rgtlj[3]['rotax'], rgtlj[3]['rotangle']))
        rgtlj[3]['linkend'] = np.dot(rgtlj[3]['rotmat'], rgtlj[3]['linkvec']) + rgtlj[3]['linkpos']
        rgtlj[3]['rngmin'] = -(95 - rngsafemargin)
        rgtlj[3]['rngmax'] = +(25 - rngsafemargin)
        rgtlj[3]['mass'] = 1.668470
        rgtlj[3]['lcntrofmass'] = 1000 * np.array([0.000000, -0.014180, -0.003410])
        rgtlj[3]['gcntrofmass'] = np.dot(rgtlj[3]['rotmat'], rgtlj[3]['lcntrofmass']) + rgtlj[3]['linkpos']
        # print(rgtlj[3]['gcntrofmass'])

        # the 4th joint and link
        rgtlj[4]['name'] = 'link4'
        rgtlj[4]['mother'] = 3
        rgtlj[4]['child'] = 5
        rgtlj[4]['linkpos'] = rgtlj[3]['linkend']
        rgtlj[4]['linkvec'] = np.array([50, 0, -295.804])
        rgtlj[4]['rotax'] = np.array([0, 0, 1])
        rgtlj[4]['rotangle'] = 0
        rgtlj[4]['rotmat'] = np.dot(rgtlj[3]['rotmat'], rm.rodrigues(rgtlj[4]['rotax'], rgtlj[4]['rotangle']))
        rgtlj[4]['linkend'] = np.dot(rgtlj[4]['rotmat'], rgtlj[4]['linkvec']) + rgtlj[4]['linkpos']
        rgtlj[4]['rngmin'] = -(92 - rngsafemargin)
        rgtlj[4]['rngmax'] = +(120 - rngsafemargin)
        rgtlj[4]['mass'] = 2.638680
        rgtlj[4]['lcntrofmass'] = 1000 * np.array([0.007520, -0.006500, -0.190920])
        rgtlj[4]['gcntrofmass'] = np.dot(rgtlj[4]['rotmat'], rgtlj[4]['lcntrofmass']) + rgtlj[4]['linkpos']
        # print(rgtlj[4]['gcntrofmass'])

        # the 5th joint and link
        rgtlj[5]['name'] = 'link5'
        rgtlj[5]['mother'] = 4
        rgtlj[5]['child'] = 6
        rgtlj[5]['linkpos'] = rgtlj[4]['linkend']
        rgtlj[5]['linkvec'] = np.array([-50, 0, -295.804])
        rgtlj[5]['rotax'] = np.array([0, 1, 0])
        rgtlj[5]['rotangle'] = 0
        rgtlj[5]['rotmat'] = np.dot(rgtlj[4]['rotmat'], rm.rodrigues(rgtlj[5]['rotax'], rgtlj[5]['rotangle']))
        rgtlj[5]['linkend'] = np.dot(rgtlj[5]['rotmat'], rgtlj[5]['linkvec']) + rgtlj[5]['linkpos']
        rgtlj[5]['rngmin'] = -(180 - rngsafemargin)
        rgtlj[5]['rngmax'] = -(16.2 - rngsafemargin)
        rgtlj[5]['mass'] = 1.132660
        rgtlj[5]['lcntrofmass'] = 1000 * np.array([-0.031530, 0.011270, -0.033830])
        rgtlj[5]['gcntrofmass'] = np.dot(rgtlj[5]['rotmat'], rgtlj[5]['lcntrofmass']) + rgtlj[5]['linkpos']
        # print(rgtlj[5]['gcntrofmass'])

        # the 6th joint and link
        rgtlj[6]['name'] = 'link6'
        rgtlj[6]['mother'] = 5
        rgtlj[6]['child'] = 7
        rgtlj[6]['linkpos'] = rgtlj[5]['linkend']
        rgtlj[6]['linkvec'] = np.array([0.0, 0.0, 0.0])
        rgtlj[6]['rotax'] = np.array([0, 0, 1])
        rgtlj[6]['rotangle'] = 0
        rgtlj[6]['rotmat'] = np.dot(rgtlj[5]['rotmat'], rm.rodrigues(rgtlj[6]['rotax'], rgtlj[6]['rotangle']))
        rgtlj[6]['linkend'] = np.dot(rgtlj[6]['rotmat'], rgtlj[6]['linkvec']) + rgtlj[6]['linkpos']
        rgtlj[6]['rngmin'] = -(122 - rngsafemargin)
        rgtlj[6]['rngmax'] = +(182 - rngsafemargin)
        rgtlj[6]['mass'] = 1.289340
        rgtlj[6]['lcntrofmass'] = 1000 * np.array([-0.002700, 0.000010, 0.129600])
        rgtlj[6]['gcntrofmass'] = np.dot(rgtlj[6]['rotmat'], rgtlj[6]['lcntrofmass']) + rgtlj[6]['linkpos']
        # print(rgtlj[6]['gcntrofmass'])

        # the 7th joint and link
        rgtlj[7]['name'] = 'link7'
        rgtlj[7]['mother'] = 6
        rgtlj[7]['child'] = 8
        rgtlj[7]['linkpos'] = rgtlj[6]['linkend']
        rgtlj[7]['linkvec'] = np.array([0.0, 0.0, 0.0])
        rgtlj[7]['rotax'] = np.array([1, 0, 0])
        rgtlj[7]['rotangle'] = 0
        rgtlj[7]['rotmat'] = np.dot(rgtlj[6]['rotmat'], rm.rodrigues(rgtlj[7]['rotax'], rgtlj[7]['rotangle']))
        rgtlj[7]['linkend'] = np.dot(rgtlj[7]['rotmat'], rgtlj[7]['linkvec']) + rgtlj[7]['linkpos']
        rgtlj[7]['rngmin'] = -(47 - rngsafemargin)
        rgtlj[7]['rngmax'] = +(70 - rngsafemargin)

        # rgtlj[7]['rngmin'] = -(87-rngsafemargin)
        # rgtlj[7]['rngmax'] = +(120-rngsafemargin)

        rgtlj[7]['mass'] = 0.047270
        rgtlj[7]['lcntrofmass'] = 1000 * np.array([0.000000, 0.000460, 0.000000])
        rgtlj[7]['gcntrofmass'] = np.dot(rgtlj[7]['rotmat'], rgtlj[7]['lcntrofmass']) + rgtlj[7]['linkpos']
        # print(rgtlj[7]['gcntrofmass'])

        # the 8th joint and link
        rgtlj[8]['name'] = 'link8'
        rgtlj[8]['mother'] = 7
        rgtlj[8]['child'] = 9
        rgtlj[8]['linkpos'] = rgtlj[7]['linkend']
        rgtlj[8]['linkvec'] = np.array([0.0, 17.324, -78.842])
        rgtlj[8]['rotax'] = np.array([0, 1, 0])
        rgtlj[8]['rotangle'] = 0
        rgtlj[8]['rotmat'] = np.dot(rgtlj[7]['rotmat'], rm.rodrigues(rgtlj[8]['rotax'], rgtlj[8]['rotangle']))
        rgtlj[8]['linkend'] = np.dot(rgtlj[8]['rotmat'], rgtlj[8]['linkvec']) + rgtlj[8]['linkpos']
        rgtlj[8]['rngmin'] = -(42 - rngsafemargin)
        rgtlj[8]['rngmax'] = +(42 - rngsafemargin)
        # rgtlj[8]['rngmin'] = -(92-rngsafemargin)
        # rgtlj[8]['rngmax'] = +(92-rngsafemargin)

        rgtlj[8]['mass'] = 0.363430
        rgtlj[8]['lcntrofmass'] = 1000 * np.array([0.000000, -0.024950, -0.033700])
        rgtlj[8]['gcntrofmass'] = np.dot(rgtlj[8]['rotmat'], rgtlj[8]['lcntrofmass']) + rgtlj[8]['linkpos']
        # print(rgtlj[8]['gcntrofmass'])

        # the 9th joint and link
        rgtlj[9]['name'] = 'link9'
        rgtlj[9]['mother'] = 8
        rgtlj[9]['child'] = -1
        rgtlj[9]['linkpos'] = rgtlj[8]['linkend']
        rgtlj[9]['linkvec'] = np.array([-187.0, 0.0, 0.0])
        rgtlj[9]['rotax'] = np.array([1, 0, 0])
        rgtlj[9]['rotangle'] = 0
        # make sure x direction faces at ee, z directions faces downward
        # see the definition of the coordinates in rtq85 (execute the file)
        rgtlj[9]['inherentR'] = np.dot(rm.rodrigues([1, 0, 0], 45), rm.rodrigues([0, 1, 0], -90))
        rgtlj[9]['rotmat'] = np.dot(np.dot(rgtlj[8]['rotmat'], rgtlj[9]['inherentR']),rm.rodrigues(rgtlj[9]['rotax'], rgtlj[9]['rotangle']))
        rgtlj[9]['linkend'] = np.dot(rgtlj[9]['rotmat'], rgtlj[9]['linkvec']) + rgtlj[9]['linkpos']
        rgtlj[9]['rngmin'] = -(150 - rngsafemargin)
        rgtlj[9]['rngmax'] = +(150 - rngsafemargin)
        rgtlj[9]['mass'] = 1.140300 #* 10 #For one leg balancing
        rgtlj[9]['lcntrofmass'] = 1000 * np.array([0.000000, -0.000060, 0.042610])
        rgtlj[9]['gcntrofmass'] = np.dot(rgtlj[9]['rotmat'], rgtlj[9]['lcntrofmass']) + rgtlj[9]['linkpos']
        # print(rgtlj[9]['gcntrofmass'])


        return rgtlj

    def __initlftarmj(self):
        """
            Mass, center of mass (local coordinates), center of mass (global coordinates) added to the links
            author: Daniel
            date: 20180515
            """

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

        ## more note:
        lftlj[1]['linkpos'] is the position of the first joint
        lftlj[i]['linkend'] is the same as lftlj[i+1]['linkpos'],
        I am keeping this value for the eef (end-effector)

        ## even more note:
        joint is attached to the linkpos of a link
        for the first link, the joint is fixed and the rotax = 0,0,0

        ## inherentR is only available at the first link

        :return:
        lftlj:
            a list of dictionaries with each dictionary holding name, mother, child,
        linkpos (link position), linkvec (link vector), rotmat (orientation), rotax (rotation axis of a joint),
        rotangle (rotation angle of the joint around rotax)
        linkend (the end position of a link) is computed using rotmat, rotax, linkpos, and linklen
        lj means link and joint, the joint attached to the link is at the linkend

        author: weiwei
        date: 20161202, tsukuba
        """

        # create a arm with 10 joints
        lftlj = [dict() for i in range(10)]
        rngsafemargin = 5

        # the 0th link and joint
        lftlj[0]['name'] = 'link0'
        lftlj[0]['mother'] = -1
        lftlj[0]['child'] = 1
        lftlj[0]['linkpos'] = self.robotoriginpos
        lftlj[0]['linkvec'] = np.array([87, 80, 542])
        lftlj[0]['rotax'] = np.array([0, 0, 1])
        lftlj[0]['rotangle'] = 0
        lftlj[0]['rotmat'] = np.dot(self.__robotoriginrotmat, np.eye(3))
        lftlj[0]['linkend'] = np.dot(lftlj[0]['rotmat'], lftlj[0]['linkvec']) + lftlj[0]['linkpos']
        lftlj[0]['rngmin'] = -(90 - rngsafemargin)
        lftlj[0]['rngmax'] = +(90 - rngsafemargin)
        lftlj[0]['mass'] = 0
        lftlj[0]['lcntrofmass'] = np.array([0, 0, 0])
        lftlj[0]['gcntrofmass'] = np.dot(lftlj[0]['rotmat'], lftlj[0]['lcntrofmass']) + lftlj[0]['linkpos']
        lftlj[0]['isleg'] = False
        # print(lftlj[0]['gcntrofmass'])

        # the 1st joint and link
        lftlj[1]['name'] = 'link1'
        lftlj[1]['mother'] = 0
        lftlj[1]['child'] = 2
        lftlj[1]['linkpos'] = lftlj[0]['linkend']
        lftlj[1]['linkvec'] = np.array([0, 130, 0])
        lftlj[1]['rotax'] = np.array([0, 0, 1])
        lftlj[1]['rotangle'] = 0
        lftlj[1]['rotmat'] = np.dot(lftlj[0]['rotmat'], rm.rodrigues(lftlj[1]['rotax'], lftlj[1]['rotangle']))
        lftlj[1]['linkend'] = np.dot(lftlj[1]['rotmat'], lftlj[1]['linkvec']) + lftlj[1]['linkpos']
        lftlj[1]['rngmin'] = -(60 - rngsafemargin)
        lftlj[1]['rngmax'] = +(20 - rngsafemargin)
        lftlj[1]['mass'] = 1.136130
        lftlj[1]['lcntrofmass'] = 1000 * np.array([0.000000, 0.009400, 0.012520])
        lftlj[1]['gcntrofmass'] = np.dot(lftlj[1]['rotmat'], lftlj[1]['lcntrofmass']) + lftlj[1]['linkpos']
        # print(lftlj[1]['gcntrofmass'])

        # the 2nd joint and link
        lftlj[2]['name'] = 'link2'
        lftlj[2]['mother'] = 1
        lftlj[2]['child'] = 3
        lftlj[2]['linkpos'] = lftlj[1]['linkend']
        lftlj[2]['linkvec'] = np.array([0, 0, 0])
        lftlj[2]['rotax'] = np.array([0, 1, 0])
        lftlj[2]['rotangle'] = 0
        lftlj[2]['rotmat'] = np.dot(lftlj[1]['rotmat'], rm.rodrigues(lftlj[2]['rotax'], lftlj[2]['rotangle']))
        lftlj[2]['linkend'] = np.dot(lftlj[2]['rotmat'], lftlj[2]['linkvec']) + lftlj[2]['linkpos']
        lftlj[2]['rngmin'] = -(200 - rngsafemargin)
        lftlj[2]['rngmax'] = +(70 - rngsafemargin)
        lftlj[2]['mass'] = 0.575550
        lftlj[2]['lcntrofmass'] = 1000 * np.array([-0.011520, -0.003790, 0.008220])
        lftlj[2]['gcntrofmass'] = np.dot(lftlj[2]['rotmat'], lftlj[2]['lcntrofmass']) + lftlj[2]['linkpos']
        # print(lftlj[2]['gcntrofmass'])

        # the 3rd joint and link
        lftlj[3]['name'] = 'link3'
        lftlj[3]['mother'] = 2
        lftlj[3]['child'] = 4
        lftlj[3]['linkpos'] = lftlj[2]['linkend']
        lftlj[3]['linkvec'] = np.array([0, 20, 0])
        lftlj[3]['rotax'] = np.array([1, 0, 0])
        lftlj[3]['rotangle'] = 0
        lftlj[3]['rotmat'] = np.dot(lftlj[2]['rotmat'], rm.rodrigues(lftlj[3]['rotax'], lftlj[3]['rotangle']))
        lftlj[3]['linkend'] = np.dot(lftlj[3]['rotmat'], lftlj[3]['linkvec']) + lftlj[3]['linkpos']
        lftlj[3]['rngmin'] = -(25 - rngsafemargin)
        lftlj[3]['rngmax'] = +(95 - rngsafemargin)
        lftlj[3]['mass'] = 1.668470
        lftlj[3]['lcntrofmass'] = 1000 * np.array([0.000000, 0.014180, -0.003410])
        lftlj[3]['gcntrofmass'] = np.dot(lftlj[3]['rotmat'], lftlj[3]['lcntrofmass']) + lftlj[3]['linkpos']
        # print(lftlj[3]['gcntrofmass'])

        # the 4th joint and link
        lftlj[4]['name'] = 'link4'
        lftlj[4]['mother'] = 3
        lftlj[4]['child'] = 5
        lftlj[4]['linkpos'] = lftlj[3]['linkend']
        lftlj[4]['linkvec'] = np.array([50, 0, -295.804])
        lftlj[4]['rotax'] = np.array([0, 0, 1])
        lftlj[4]['rotangle'] = 0
        lftlj[4]['rotmat'] = np.dot(lftlj[3]['rotmat'], rm.rodrigues(lftlj[4]['rotax'], lftlj[4]['rotangle']))
        lftlj[4]['linkend'] = np.dot(lftlj[4]['rotmat'], lftlj[4]['linkvec']) + lftlj[4]['linkpos']
        lftlj[4]['rngmin'] = -(120 - rngsafemargin)
        lftlj[4]['rngmax'] = +(92 - rngsafemargin)
        lftlj[4]['mass'] = 2.638720
        lftlj[4]['lcntrofmass'] = 1000 * np.array([0.007520, 0.006490, -0.190930])
        lftlj[4]['gcntrofmass'] = np.dot(lftlj[4]['rotmat'], lftlj[4]['lcntrofmass']) + lftlj[4]['linkpos']
        # print(lftlj[4]['gcntrofmass'])

        # the 5th joint and link
        lftlj[5]['name'] = 'link5'
        lftlj[5]['mother'] = 4
        lftlj[5]['child'] = 6
        lftlj[5]['linkpos'] = lftlj[4]['linkend']
        lftlj[5]['linkvec'] = np.array([-50, 0, -295.804])
        lftlj[5]['rotax'] = np.array([0, 1, 0])
        lftlj[5]['rotangle'] = 0
        lftlj[5]['rotmat'] = np.dot(lftlj[4]['rotmat'], rm.rodrigues(lftlj[5]['rotax'], lftlj[5]['rotangle']))
        lftlj[5]['linkend'] = np.dot(lftlj[5]['rotmat'], lftlj[5]['linkvec']) + lftlj[5]['linkpos']
        lftlj[5]['rngmin'] = -(180 - rngsafemargin)
        lftlj[5]['rngmax'] = -(16.2 - rngsafemargin)
        lftlj[5]['mass'] = 1.132380
        lftlj[5]['lcntrofmass'] = 1000 * np.array([-0.031530, -0.011270, -0.033830])
        lftlj[5]['gcntrofmass'] = np.dot(lftlj[5]['rotmat'], lftlj[5]['lcntrofmass']) + lftlj[5]['linkpos']
        # print(lftlj[5]['gcntrofmass'])

        # the 6th joint and link
        lftlj[6]['name'] = 'link6'
        lftlj[6]['mother'] = 5
        lftlj[6]['child'] = 7
        lftlj[6]['linkpos'] = lftlj[5]['linkend']
        lftlj[6]['linkvec'] = np.array([0, 0, 0])
        lftlj[6]['rotax'] = np.array([0, 0, 1])
        lftlj[6]['rotangle'] = 0
        lftlj[6]['rotmat'] = np.dot(lftlj[5]['rotmat'], rm.rodrigues(lftlj[6]['rotax'], lftlj[6]['rotangle']))
        lftlj[6]['linkend'] = np.dot(lftlj[6]['rotmat'], lftlj[6]['linkvec']) + lftlj[6]['linkpos']
        lftlj[6]['rngmin'] = -(182 - rngsafemargin)
        lftlj[6]['rngmax'] = +(122 - rngsafemargin)
        lftlj[6]['mass'] = 1.289340
        lftlj[6]['lcntrofmass'] = 1000 * np.array([-0.002700, -0.000010, 0.129600])
        lftlj[6]['gcntrofmass'] = np.dot(lftlj[6]['rotmat'], lftlj[6]['lcntrofmass']) + lftlj[6]['linkpos']
        # print(lftlj[6]['gcntrofmass'])

        # the 7th joint and link
        lftlj[7]['name'] = 'link7'
        lftlj[7]['mother'] = 6
        lftlj[7]['child'] = 8
        lftlj[7]['linkpos'] = lftlj[6]['linkend']
        lftlj[7]['linkvec'] = np.array([0, 0, 0])
        lftlj[7]['rotax'] = np.array([1, 0, 0])
        lftlj[7]['rotangle'] = 0
        lftlj[7]['rotmat'] = np.dot(lftlj[6]['rotmat'], rm.rodrigues(lftlj[7]['rotax'], lftlj[7]['rotangle']))
        lftlj[7]['linkend'] = np.dot(lftlj[7]['rotmat'], lftlj[7]['linkvec']) + lftlj[7]['linkpos']
        lftlj[7]['rngmin'] = -(70 - rngsafemargin)
        lftlj[7]['rngmax'] = +(47 - rngsafemargin)
        lftlj[7]['mass'] = 0.047270
        lftlj[7]['lcntrofmass'] = 1000 * np.array([0.000000, -0.000460, 0.000000])
        lftlj[7]['gcntrofmass'] = np.dot(lftlj[7]['rotmat'], lftlj[7]['lcntrofmass']) + lftlj[7]['linkpos']
        # print(lftlj[7]['gcntrofmass'])

        # the 8th joint and link
        lftlj[8]['name'] = 'link8'
        lftlj[8]['mother'] = 7
        lftlj[8]['child'] = 9
        lftlj[8]['linkpos'] = lftlj[7]['linkend']
        # lftlj[8]['linkvec'] = np.array([0,-7.004,-67.249])
        lftlj[8]['linkvec'] = np.array([0, -17.324, -78.842])
        lftlj[8]['rotax'] = np.array([0, 1, 0])
        lftlj[8]['rotangle'] = 0
        lftlj[8]['rotmat'] = np.dot(lftlj[7]['rotmat'], rm.rodrigues(lftlj[8]['rotax'], lftlj[8]['rotangle']))
        lftlj[8]['linkend'] = np.dot(lftlj[8]['rotmat'], lftlj[8]['linkvec']) + lftlj[8]['linkpos']
        lftlj[8]['rngmin'] = -(42 - rngsafemargin)
        lftlj[8]['rngmax'] = +(42 - rngsafemargin)
        lftlj[8]['mass'] = 0.363430
        lftlj[8]['lcntrofmass'] = 1000 * np.array([0.000000, 0.024950, -0.033700])
        lftlj[8]['gcntrofmass'] = np.dot(lftlj[8]['rotmat'], lftlj[8]['lcntrofmass']) + lftlj[8]['linkpos']
        # print(lftlj[8]['gcntrofmass'])

        # the 9th joint and link
        lftlj[9]['name'] = 'link9'
        lftlj[9]['mother'] = 8
        lftlj[9]['child'] = -1
        lftlj[9]['linkpos'] = lftlj[8]['linkend']
        lftlj[9]['linkvec'] = np.array([-187.0, 0, 0])
        lftlj[9]['rotax'] = np.array([1, 0, 0])
        lftlj[9]['rotangle'] = 0
        # make sure x direction faces at ee, z directions faces downward
        # see the definition of the coordinates in rtq85 (execute the file)
        lftlj[9]['inherentR'] = np.dot(rm.rodrigues([1, 0, 0], -45), rm.rodrigues([0, 1, 0], -90))
        lftlj[9]['rotmat'] = np.dot(np.dot(lftlj[8]['rotmat'], lftlj[9]['inherentR']), \
                                    rm.rodrigues(lftlj[9]['rotax'], lftlj[9]['rotangle']))
        lftlj[9]['linkend'] = np.dot(lftlj[9]['rotmat'], lftlj[9]['linkvec']) + lftlj[9]['linkpos']
        lftlj[9]['rngmin'] = -(330 - rngsafemargin)
        lftlj[9]['rngmax'] = -(30 + rngsafemargin)
        lftlj[9]['mass'] = 1.140300
        lftlj[9]['lcntrofmass'] = 1000 * np.array([0.000000, 0.000060, 0.042610])
        lftlj[9]['gcntrofmass'] = np.dot(lftlj[9]['rotmat'], lftlj[9]['lcntrofmass']) + lftlj[9]['linkpos']
        # print(lftlj[9]['gcntrofmass'])
        # 0.000000, -0.026100, 0.008800
        # mass 0.018250

        return lftlj

    def __initrgtlegj(self):

        """
            Mass, center of mass (local coordinates), center of mass (global coordinates) added to the links
            author: Daniel
            date: 20180515
            """

        """
        init rgt leg

        :return:
        author: weiwei
        date: 20180203
        """

        # create a arm with 10 joints lj = linkjoint
        rgtlj = [dict() for i in range(7)]
        rngsafemargin = 5

        # the 0th link and joint
        rgtlj[0]['name'] = 'link0'
        rgtlj[0]['mother'] = -1
        rgtlj[0]['child'] = 1
        rgtlj[0]['linkpos'] = self.robotoriginpos
        rgtlj[0]['linkvec'] = np.array([0, -60, 0])
        rgtlj[0]['rotax'] = np.array([0, 0, 1])
        rgtlj[0]['rotangle'] = 0
        rgtlj[0]['rotmat'] = np.dot(self.__robotoriginrotmat, np.eye(3))
        rgtlj[0]['linkend'] = np.dot(rgtlj[0]['rotmat'], rgtlj[0]['linkvec']) + rgtlj[0]['linkpos']
        rgtlj[0]['rngmin'] = -(90 - rngsafemargin)
        rgtlj[0]['rngmax'] = +(90 - rngsafemargin)
        rgtlj[0]['mass'] = 0
        rgtlj[0]['lcntrofmass'] = np.array([0, 0, 0])
        rgtlj[0]['gcntrofmass'] = np.dot(rgtlj[0]['rotmat'], rgtlj[0]['lcntrofmass']) + rgtlj[0]['linkpos']
        rgtlj[0]['isleg'] = True
        # print(rgtlj[0]['gcntrofmass'])

        # the 1st joint and link
        rgtlj[1]['name'] = 'link1'
        rgtlj[1]['mother'] = 0
        rgtlj[1]['child'] = 2
        rgtlj[1]['linkpos'] = rgtlj[0]['linkend']
        rgtlj[1]['linkvec'] = np.array([0, 0, 0])
        rgtlj[1]['rotax'] = np.array([0, 0, 1])
        rgtlj[1]['rotangle'] = 0
        rgtlj[1]['rotmat'] = np.dot(rgtlj[0]['rotmat'], rm.rodrigues(rgtlj[1]['rotax'], rgtlj[1]['rotangle']))
        rgtlj[1]['linkend'] = np.dot(rgtlj[1]['rotmat'], rgtlj[1]['linkvec']) + rgtlj[1]['linkpos']
        rgtlj[1]['rngmin'] = -(45 - rngsafemargin)
        rgtlj[1]['rngmax'] = +(45 - rngsafemargin)
        rgtlj[1]['mass'] = 1.035720
        rgtlj[1]['lcntrofmass'] = 1000 * np.array([0.002870, 0.006590, 0.053390])
        rgtlj[1]['gcntrofmass'] = np.dot(rgtlj[1]['rotmat'], rgtlj[1]['lcntrofmass']) + rgtlj[1]['linkpos']
        # print(rgtlj[1]['gcntrofmass'])

        # the 2nd joint and link
        rgtlj[2]['name'] = 'link2'
        rgtlj[2]['mother'] = 1
        rgtlj[2]['child'] = 3
        rgtlj[2]['linkpos'] = rgtlj[1]['linkend']
        rgtlj[2]['linkvec'] = np.array([0, 0, 0])
        rgtlj[2]['rotax'] = np.array([1, 0, 0])
        rgtlj[2]['rotangle'] = 0
        rgtlj[2]['rotmat'] = np.dot(rgtlj[1]['rotmat'], rm.rodrigues(rgtlj[2]['rotax'], rgtlj[2]['rotangle']))
        rgtlj[2]['linkend'] = np.dot(rgtlj[2]['rotmat'], rgtlj[2]['linkvec']) + rgtlj[2]['linkpos']
        rgtlj[2]['rngmin'] = -(45 - rngsafemargin)
        rgtlj[2]['rngmax'] = +(45 - rngsafemargin)
        rgtlj[2]['mass'] = 0.355380
        rgtlj[2]['lcntrofmass'] = 1000 * np.array([-0.005470, -0.012240, 0.000000])
        rgtlj[2]['gcntrofmass'] = np.dot(rgtlj[2]['rotmat'], rgtlj[2]['lcntrofmass']) + rgtlj[2]['linkpos']
        # print(rgtlj[2]['gcntrofmass'])

        # the 3rd joint and link
        rgtlj[3]['name'] = 'link3'
        rgtlj[3]['mother'] = 2
        rgtlj[3]['child'] = 4
        rgtlj[3]['linkpos'] = rgtlj[2]['linkend']
        rgtlj[3]['linkvec'] = np.array([0, -35, -380])
        rgtlj[3]['rotax'] = np.array([0, 1, 0])
        rgtlj[3]['rotangle'] = 0
        rgtlj[3]['rotmat'] = np.dot(rgtlj[2]['rotmat'], rm.rodrigues(rgtlj[3]['rotax'], rgtlj[3]['rotangle']))
        rgtlj[3]['linkend'] = np.dot(rgtlj[3]['rotmat'], rgtlj[3]['linkvec']) + rgtlj[3]['linkpos']
        rgtlj[3]['rngmin'] = -(125 - rngsafemargin)
        rgtlj[3]['rngmax'] = +(77 - rngsafemargin)
        rgtlj[3]['mass'] = 7.048530
        rgtlj[3]['lcntrofmass'] = 1000 * np.array([0.006300, -0.081590, -0.191920])
        rgtlj[3]['gcntrofmass'] = np.dot(rgtlj[3]['rotmat'], rgtlj[3]['lcntrofmass']) + rgtlj[3]['linkpos']
        # print(rgtlj[3]['gcntrofmass'])

        # the 4th joint and link
        rgtlj[4]['name'] = 'link4'
        rgtlj[4]['mother'] = 3
        rgtlj[4]['child'] = 5
        rgtlj[4]['linkpos'] = rgtlj[3]['linkend']
        rgtlj[4]['linkvec'] = np.array([0, 0, -380])
        rgtlj[4]['rotax'] = np.array([0, 1, 0])
        rgtlj[4]['rotangle'] = 0
        rgtlj[4]['rotmat'] = np.dot(rgtlj[3]['rotmat'], rm.rodrigues(rgtlj[4]['rotax'], rgtlj[4]['rotangle']))
        rgtlj[4]['linkend'] = np.dot(rgtlj[4]['rotmat'], rgtlj[4]['linkvec']) + rgtlj[4]['linkpos']
        rgtlj[4]['rngmin'] = -(0 - rngsafemargin)
        rgtlj[4]['rngmax'] = +(150 - rngsafemargin)
        rgtlj[4]['mass'] = 3.982090
        rgtlj[4]['lcntrofmass'] = 1000 * np.array([0.007370, -0.008450, -0.138770])
        rgtlj[4]['gcntrofmass'] = np.dot(rgtlj[4]['rotmat'], rgtlj[4]['lcntrofmass']) + rgtlj[4]['linkpos']
        # print(rgtlj[4]['gcntrofmass'])

        # the 5th joint and link
        rgtlj[5]['name'] = 'link5'
        rgtlj[5]['mother'] = 4
        rgtlj[5]['child'] = 6
        rgtlj[5]['linkpos'] = rgtlj[4]['linkend']
        rgtlj[5]['linkvec'] = np.array([0, 0, 0])
        rgtlj[5]['rotax'] = np.array([0, 1, 0])
        rgtlj[5]['rotangle'] = 0
        rgtlj[5]['rotmat'] = np.dot(rgtlj[4]['rotmat'], rm.rodrigues(rgtlj[5]['rotax'], rgtlj[5]['rotangle']))
        rgtlj[5]['linkend'] = np.dot(rgtlj[5]['rotmat'], rgtlj[5]['linkvec']) + rgtlj[5]['linkpos']
        rgtlj[5]['rngmin'] = -(85 - rngsafemargin)
        rgtlj[5]['rngmax'] = +(55 - rngsafemargin)
        rgtlj[5]['mass'] = 0.469790
        rgtlj[5]['lcntrofmass'] = 1000 * np.array([-0.011240, -0.001050, 0.011740])
        rgtlj[5]['gcntrofmass'] = np.dot(rgtlj[5]['rotmat'], rgtlj[5]['lcntrofmass']) + rgtlj[5]['linkpos']
        # print(rgtlj[5]['gcntrofmass'])

        # the 6th joint and link
        rgtlj[6]['name'] = 'link6'
        rgtlj[6]['mother'] = 5
        rgtlj[6]['child'] = -1
        rgtlj[6]['linkpos'] = rgtlj[5]['linkend']
        rgtlj[6]['linkvec'] = np.array([0, 0, -102])
        rgtlj[6]['rotax'] = np.array([1, 0, 0])
        rgtlj[6]['rotangle'] = 0
        rgtlj[6]['rotmat'] = np.dot(rgtlj[5]['rotmat'], rm.rodrigues(rgtlj[6]['rotax'], rgtlj[6]['rotangle']))
        rgtlj[6]['linkend'] = np.dot(rgtlj[6]['rotmat'], rgtlj[6]['linkvec']) + rgtlj[6]['linkpos']
        rgtlj[6]['rngmin'] = -(45 - rngsafemargin)
        rgtlj[6]['rngmax'] = +(50 - rngsafemargin)
        rgtlj[6]['mass'] = 1.564760
        rgtlj[6]['lcntrofmass'] = 1000 * np.array([0.000690, -0.007770, -0.070910])
        rgtlj[6]['gcntrofmass'] = np.dot(rgtlj[6]['rotmat'], rgtlj[6]['lcntrofmass']) + rgtlj[6]['linkpos']
        rgtlj[6]['solevertices'] = 1000 * np.array([[-0.10, -0.075, -0.09], [-0.10, 0.055, -0.09],
                                                    [0.13, 0.055, -0.09], [0.13, -0.075, -0.09]])
        rgtlj[6]['gsolevertices'] = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                                              [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        for i in range(4):
            rgtlj[6]['gsolevertices'][i] = np.dot(rgtlj[6]['rotmat'], np.array(rgtlj[6]['solevertices'][i])) + rgtlj[6][
                'linkpos']
        # print(rgtlj[6]['gcntrofmass'])

        rgtlj[6]['footcenterposition'] = np.array([rgtlj[6]['linkpos'][0], rgtlj[6]['linkpos'][1],
                                                   rgtlj[6]['gsolevertices'][0][2]])
        return rgtlj

    def __initlftlegj(self):
        """
        init lft leg

        :return:
        author: weiwei
        date: 20180203
        """
        """
            Mass, center of mass (local coordinates), center of mass (global coordinates) added to the links 
            author: Daniel
            date: 20180515
            """

        # create a arm with 10 joints lj = linkjoint
        lftlj = [dict() for i in range(7)]
        rngsafemargin = 5

        # the 0th link and joint
        lftlj[0]['name'] = 'link0'
        lftlj[0]['mother'] = -1
        lftlj[0]['child'] = 1
        lftlj[0]['linkpos'] = self.robotoriginpos
        lftlj[0]['linkvec'] = np.array([0, 60, 0])
        lftlj[0]['rotax'] = np.array([0, 0, 1])
        lftlj[0]['rotangle'] = 0
        lftlj[0]['rotmat'] = np.dot(self.__robotoriginrotmat, np.eye(3))
        lftlj[0]['linkend'] = np.dot(lftlj[0]['rotmat'], lftlj[0]['linkvec']) + lftlj[0]['linkpos']
        lftlj[0]['rngmin'] = -(90 - rngsafemargin)
        lftlj[0]['rngmax'] = +(90 - rngsafemargin)
        lftlj[0]['mass'] = 0
        lftlj[0]['lcntrofmass'] = np.array([0, 0, 0])
        lftlj[0]['gcntrofmass'] = np.dot(lftlj[0]['rotmat'], lftlj[0]['lcntrofmass']) + lftlj[0]['linkpos']
        lftlj[0]['isleg'] = True
        # print(lftlj[0]['gcntrofmass'])

        # the 1st joint and link
        lftlj[1]['name'] = 'link1'
        lftlj[1]['mother'] = 0
        lftlj[1]['child'] = 2
        lftlj[1]['linkpos'] = lftlj[0]['linkend']
        lftlj[1]['linkvec'] = np.array([0, 0, 0])
        lftlj[1]['rotax'] = np.array([0, 0, 1])
        lftlj[1]['rotangle'] = 0
        lftlj[1]['rotmat'] = np.dot(lftlj[0]['rotmat'], rm.rodrigues(lftlj[1]['rotax'], lftlj[1]['rotangle']))
        lftlj[1]['linkend'] = np.dot(lftlj[1]['rotmat'], lftlj[1]['linkvec']) + lftlj[1]['linkpos']
        lftlj[1]['rngmin'] = -(45 - rngsafemargin)
        lftlj[1]['rngmax'] = +(45 - rngsafemargin)
        lftlj[1]['mass'] = 1.035720
        lftlj[1]['lcntrofmass'] = 1000 * np.array([0.002870, -0.006590, 0.053390])
        lftlj[1]['gcntrofmass'] = np.dot(lftlj[1]['rotmat'], lftlj[1]['lcntrofmass']) + lftlj[1]['linkpos']

        # print(lftlj[1]['gcntrofmass'])

        # the 2nd joint and link
        lftlj[2]['name'] = 'link2'
        lftlj[2]['mother'] = 1
        lftlj[2]['child'] = 3
        lftlj[2]['linkpos'] = lftlj[1]['linkend']
        lftlj[2]['linkvec'] = np.array([0, 0, 0])
        lftlj[2]['rotax'] = np.array([1, 0, 0])
        lftlj[2]['rotangle'] = 0
        lftlj[2]['rotmat'] = np.dot(lftlj[1]['rotmat'], rm.rodrigues(lftlj[2]['rotax'], lftlj[2]['rotangle']))
        lftlj[2]['linkend'] = np.dot(lftlj[2]['rotmat'], lftlj[2]['linkvec']) + lftlj[2]['linkpos']
        lftlj[2]['rngmin'] = -(45 - rngsafemargin)
        lftlj[2]['rngmax'] = +(45 - rngsafemargin)
        lftlj[2]['mass'] = 0.355380
        lftlj[2]['lcntrofmass'] = 1000 * np.array([-0.005470, 0.012240, 0.000000])
        lftlj[2]['gcntrofmass'] = np.dot(lftlj[2]['rotmat'], lftlj[2]['lcntrofmass']) + lftlj[2]['linkpos']

        # print(lftlj[2]['gcntrofmass'])

        # the 3rd joint and link
        lftlj[3]['name'] = 'link3'
        lftlj[3]['mother'] = 2
        lftlj[3]['child'] = 4
        lftlj[3]['linkpos'] = lftlj[2]['linkend']
        lftlj[3]['linkvec'] = np.array([0, 35, -380])
        lftlj[3]['rotax'] = np.array([0, 1, 0])
        lftlj[3]['rotangle'] = 0
        lftlj[3]['rotmat'] = np.dot(lftlj[2]['rotmat'], rm.rodrigues(lftlj[3]['rotax'], lftlj[3]['rotangle']))
        lftlj[3]['linkend'] = np.dot(lftlj[3]['rotmat'], lftlj[3]['linkvec']) + lftlj[3]['linkpos']
        lftlj[3]['rngmin'] = -(125 - rngsafemargin)
        lftlj[3]['rngmax'] = +(77 - rngsafemargin)
        lftlj[3]['mass'] = 7.048530
        lftlj[3]['lcntrofmass'] = 1000 * np.array([0.006300, 0.081590, -0.191920])
        lftlj[3]['gcntrofmass'] = np.dot(lftlj[3]['rotmat'], lftlj[3]['lcntrofmass']) + lftlj[3]['linkpos']

        # print(lftlj[3]['gcntrofmass'])

        # the 4th joint and link
        lftlj[4]['name'] = 'link4'
        lftlj[4]['mother'] = 3
        lftlj[4]['child'] = 5
        lftlj[4]['linkpos'] = lftlj[3]['linkend']
        lftlj[4]['linkvec'] = np.array([0, 0, -380])
        lftlj[4]['rotax'] = np.array([0, 1, 0])
        lftlj[4]['rotangle'] = 0
        lftlj[4]['rotmat'] = np.dot(lftlj[3]['rotmat'], rm.rodrigues(lftlj[4]['rotax'], lftlj[4]['rotangle']))
        lftlj[4]['linkend'] = np.dot(lftlj[4]['rotmat'], lftlj[4]['linkvec']) + lftlj[4]['linkpos']
        lftlj[4]['rngmin'] = -(0 - rngsafemargin)
        lftlj[4]['rngmax'] = +(150 - rngsafemargin)
        lftlj[4]['mass'] = 3.982090
        lftlj[4]['lcntrofmass'] = 1000 * np.array([0.007370, 0.008450, -0.138770])
        lftlj[4]['gcntrofmass'] = np.dot(lftlj[4]['rotmat'], lftlj[4]['lcntrofmass']) + lftlj[4]['linkpos']

        # print(lftlj[4]['gcntrofmass'])

        # the 5th joint and link
        lftlj[5]['name'] = 'link5'
        lftlj[5]['mother'] = 4
        lftlj[5]['child'] = 6
        lftlj[5]['linkpos'] = lftlj[4]['linkend']
        lftlj[5]['linkvec'] = np.array([0, 0, 0])
        lftlj[5]['rotax'] = np.array([0, 1, 0])
        lftlj[5]['rotangle'] = 0
        lftlj[5]['rotmat'] = np.dot(lftlj[4]['rotmat'], rm.rodrigues(lftlj[5]['rotax'], lftlj[5]['rotangle']))
        lftlj[5]['linkend'] = np.dot(lftlj[5]['rotmat'], lftlj[5]['linkvec']) + lftlj[5]['linkpos']
        lftlj[5]['rngmin'] = -(85 - rngsafemargin)
        lftlj[5]['rngmax'] = +(55 - rngsafemargin)
        lftlj[5]['mass'] = 0.469790
        lftlj[5]['lcntrofmass'] = 1000 * np.array([-0.011240, 0.001050, 0.011740])
        lftlj[5]['gcntrofmass'] = np.dot(lftlj[5]['rotmat'], lftlj[5]['lcntrofmass']) + lftlj[5]['linkpos']

        # print(lftlj[5]['gcntrofmass'])

        # the 6th joint and link
        lftlj[6]['name'] = 'link6'
        lftlj[6]['mother'] = 5
        lftlj[6]['child'] = -1
        lftlj[6]['linkpos'] = lftlj[5]['linkend']
        lftlj[6]['linkvec'] = np.array([0, 0, -102])
        lftlj[6]['rotax'] = np.array([1, 0, 0])
        lftlj[6]['rotangle'] = 0
        lftlj[6]['rotmat'] = np.dot(lftlj[5]['rotmat'], rm.rodrigues(lftlj[6]['rotax'], lftlj[6]['rotangle']))
        lftlj[6]['linkend'] = np.dot(lftlj[6]['rotmat'], lftlj[6]['linkvec']) + lftlj[6]['linkpos']
        lftlj[6]['rngmin'] = -(45 - rngsafemargin)
        lftlj[6]['rngmax'] = +(50 - rngsafemargin)
        lftlj[6]['mass'] = 1.564760
        lftlj[6]['lcntrofmass'] = 1000 * np.array([0.000690, 0.007770, -0.070910])
        lftlj[6]['gcntrofmass'] = np.dot(lftlj[6]['rotmat'], lftlj[6]['lcntrofmass']) + lftlj[6]['linkpos']
        lftlj[6]['solevertices'] = 1000 * np.array([[-0.10, -0.055, -0.09], [-0.10, 0.075, -0.09],
                                                    [0.13, 0.075, -0.09], [0.13, -0.055, -0.09]])
        lftlj[6]['gsolevertices'] = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                                              [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        for i in range(4):
            lftlj[6]['gsolevertices'][i] = np.dot(lftlj[6]['rotmat'], np.array(lftlj[6]['solevertices'][i])) + lftlj[6][
                'linkpos']
        # print(lftlj[6]['gcntrofmass'])
        lftlj[6]['footcenterposition'] = np.array(
            [lftlj[6]['linkpos'][0], lftlj[6]['linkpos'][1], lftlj[6]['gsolevertices'][0][2]])

        return lftlj

    def __initchest(self):
        """
        Init rgt arm links and joints

        ## note
        x is facing forward, y is facing left, z is facing upward
        each element of rgtlj is a dictionary
        rgtlj[i]['linkpos'] indicates the position of a link
        rgtlj[i]['linkvec'] indicates the vector of a link that points from start to end
        rgtlj[i]['rotmat'] indicates the frame of this link
        rgtlj[i]['rotax'] indicates the rotation axis of the link
        rgtlj[i]['rotangle'] indicates the rotation angle of the link around the rotax
        rgtlj[i]['linkend'] indicates the end position of the link (passively computed)

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
        date: 20161202, tsukuba
        """
        """
            Mass, center of mass (local coordinates), center of mass (global coordinates) added to the links 
            author: Daniel
            date: 20180515
            """
        # create the chest with 10 joints
        chestj = [dict() for i in range(7)]
        rngsafemargin = 5

        # the 0th link and joint
        chestj[0]['name'] = 'link0'
        chestj[0]['mother'] = -1
        chestj[0]['child'] = 1
        chestj[0]['linkpos'] = self.robotoriginpos
        chestj[0]['linkvec'] = np.array([0.000000, 0.000000, 0.860000])
        chestj[0]['rotax'] = np.array([0, 0, 0])
        chestj[0]['rotangle'] = 0
        chestj[0]['rotmat'] = np.dot(self.__robotoriginrotmat, np.eye(3))
        chestj[0]['linkend'] = np.dot(chestj[0]['rotmat'], chestj[0]['linkvec']) + chestj[0]['linkpos']
        chestj[0]['rngmin'] = -(90 - rngsafemargin)
        chestj[0]['rngmax'] = +(90 - rngsafemargin)
        chestj[0]['mass'] = 0
        chestj[0]['lcntrofmass'] = np.array([0, 0, 0])
        chestj[0]['gcntrofmass'] = np.dot(chestj[0]['rotmat'], chestj[0]['lcntrofmass']) + chestj[0]['linkpos']
        chestj[0]['isleg'] = False

        # the 1st joint and link
        chestj[1]['name'] = 'link1'
        chestj[1]['mother'] = 0
        chestj[1]['child'] = 2
        chestj[1]['linkpos'] = chestj[0]['linkend']
        chestj[1]['linkvec'] = np.array([0.055000, 0.000000, 0.274000])
        chestj[1]['rotax'] = np.array([1, 0, 0])
        chestj[1]['rotangle'] = 0
        chestj[1]['rotmat'] = np.dot(chestj[0]['rotmat'], rm.rodrigues(chestj[1]['rotax'], chestj[1]['rotangle']))
        chestj[1]['linkend'] = np.dot(chestj[1]['rotmat'], chestj[1]['linkvec']) + chestj[1]['linkpos']
        chestj[1]['rngmin'] = -(90 - rngsafemargin)
        chestj[1]['rngmax'] = +(90 - rngsafemargin)
        chestj[1]['mass'] = 7.332630
        chestj[1]['lcntrofmass'] = 1000 * np.array([-0.034470, 0.001170, 0.178920])
        chestj[1]['gcntrofmass'] = np.dot(chestj[1]['rotmat'], chestj[1]['lcntrofmass']) + chestj[1]['linkpos']

        # print(chestj[1]['gcntrofmass'])

        # the 2nd joint and link
        chestj[2]['name'] = 'link2'
        chestj[2]['mother'] = 1
        chestj[2]['child'] = 3
        chestj[2]['linkpos'] = chestj[1]['linkend']
        chestj[2]['linkvec'] = np.array([0, 0, 0])
        chestj[2]['rotax'] = np.array([0, 1, 0])
        chestj[2]['rotangle'] = 0
        chestj[2]['rotmat'] = np.dot(chestj[1]['rotmat'], rm.rodrigues(chestj[2]['rotax'], chestj[2]['rotangle']))
        chestj[2]['linkend'] = np.dot(chestj[2]['rotmat'], chestj[2]['linkvec']) + chestj[2]['linkpos']
        chestj[2]['rngmin'] = -(12 - rngsafemargin)
        chestj[2]['rngmax'] = +(90 - rngsafemargin)
        chestj[2]['mass'] = 0.400950
        chestj[2]['lcntrofmass'] = 1000 * np.array([-0.013850, -0.001440, 0.000000])
        chestj[2]['gcntrofmass'] = np.dot(chestj[2]['rotmat'], chestj[2]['lcntrofmass']) + chestj[2]['linkpos']
        # print(chestj[2]['gcntrofmass'])

        # the 3rd joint and link
        chestj[3]['name'] = 'link3'
        chestj[3]['mother'] = 2
        chestj[3]['child'] = 4
        chestj[3]['linkpos'] = chestj[2]['linkend']
        chestj[3]['linkvec'] = np.array([0, 0, 0])
        chestj[3]['rotax'] = np.array([1, 0, 0])
        chestj[3]['rotangle'] = 0
        chestj[3]['rotmat'] = np.dot(chestj[2]['rotmat'], rm.rodrigues(chestj[3]['rotax'], chestj[3]['rotangle']))
        chestj[3]['linkend'] = np.dot(chestj[3]['rotmat'], chestj[3]['linkvec']) + chestj[3]['linkpos']
        chestj[3]['rngmin'] = -(45 - rngsafemargin)
        chestj[3]['rngmax'] = +(45 - rngsafemargin)
        chestj[3]['mass'] = 0.987050
        chestj[3]['lcntrofmass'] = 1000 * np.array([-0.056080, 0.000000, 0.045910])
        chestj[3]['gcntrofmass'] = np.dot(chestj[3]['rotmat'], chestj[3]['lcntrofmass']) + chestj[3]['linkpos']
        # print(chestj[3]['gcntrofmass'])

        # the 4th joint and link
        chestj[4]['name'] = 'link4'
        chestj[4]['mother'] = 3
        chestj[4]['child'] = 5
        chestj[4]['linkpos'] = chestj[3]['linkend']
        chestj[4]['linkvec'] = np.array([32.000, 0.000000, 521.000])
        chestj[4]['rotax'] = np.array([0, 0, 1])
        chestj[4]['rotangle'] = 0
        chestj[4]['rotmat'] = np.dot(chestj[3]['rotmat'], rm.rodrigues(chestj[4]['rotax'], chestj[4]['rotangle']))
        chestj[4]['linkend'] = np.dot(chestj[4]['rotmat'], chestj[4]['linkvec']) + chestj[4]['linkpos']
        chestj[4]['rngmin'] = -(45 - rngsafemargin)
        chestj[4]['rngmax'] = +(45 - rngsafemargin)
        chestj[4]['mass'] = 23.715460
        chestj[4]['lcntrofmass'] = 1000 * np.array([-0.090070, 0.000000, 0.248430])
        chestj[4]['gcntrofmass'] = np.dot(chestj[4]['rotmat'], chestj[4]['lcntrofmass']) + chestj[4]['linkpos']
        # print(chestj[4]['gcntrofmass'])

        # the 5th joint and link
        chestj[5]['name'] = 'link5'
        chestj[5]['mother'] = 4
        chestj[5]['child'] = 6
        chestj[5]['linkpos'] = chestj[4]['linkend']
        chestj[5]['linkvec'] = np.array([40.00000, 0.000000, 0.000000])
        chestj[5]['rotax'] = np.array([0, 1, 0])
        chestj[5]['rotangle'] = 0
        chestj[5]['rotmat'] = np.dot(chestj[4]['rotmat'], rm.rodrigues(chestj[5]['rotax'], chestj[5]['rotangle']))
        chestj[5]['linkend'] = np.dot(chestj[5]['rotmat'], chestj[5]['linkvec']) + chestj[5]['linkpos']
        chestj[5]['rngmin'] = -(90 - rngsafemargin)
        chestj[5]['rngmax'] = +(90 - rngsafemargin)
        chestj[5]['mass'] = 0.435190
        chestj[5]['lcntrofmass'] = 1000 * np.array([0.010540, 0.000000, -0.092580])
        chestj[5]['gcntrofmass'] = np.dot(chestj[5]['rotmat'], chestj[5]['lcntrofmass']) + chestj[5]['linkpos']
        # print(chestj[5]['gcntrofmass'])

        # the 6th joint and link
        chestj[6]['name'] = 'link6'
        chestj[6]['mother'] = 5
        chestj[6]['child'] = -1
        chestj[6]['linkpos'] = chestj[5]['linkend']
        chestj[6]['linkvec'] = np.array([66.900, -9.000, 98.500])
        chestj[6]['rotax'] = np.array([0, 0, 1])
        chestj[6]['rotangle'] = 0
        chestj[6]['rotmat'] = np.dot(chestj[5]['rotmat'], rm.rodrigues(chestj[6]['rotax'], chestj[6]['rotangle']))
        chestj[6]['linkend'] = np.dot(chestj[6]['rotmat'], chestj[6]['linkvec']) + chestj[6]['linkpos']
        chestj[6]['rngmin'] = -(95 - rngsafemargin)
        chestj[6]['rngmax'] = +(90 - rngsafemargin)
        chestj[6]['mass'] = 3.509700
        chestj[6]['lcntrofmass'] = 1000.0*np.array([-0.22850, -0.000010, 0.018200]) #1000 * np.array([0.012850, -0.000010, 0.018200])
        chestj[6]['gcntrofmass'] = np.dot(chestj[6]['rotmat'], chestj[6]['lcntrofmass']) + chestj[6]['linkpos']
        # print(chestj[6]['gcntrofmass'])

        # # the 7th joint and link HEAD
        # chestj[7]['name'] = 'link6'
        # chestj[7]['mother'] = 6
        # chestj[7]['child'] = 8
        # chestj[7]['linkpos'] = chestj[6]['linkend']
        # chestj[7]['linkvec'] = np.array([47.590, 12.500, 98.500])
        # chestj[7]['rotax'] = np.array([0, 0, 1])
        # chestj[7]['rotangle'] = 0
        # chestj[7]['rotmat'] = np.dot(chestj[6]['rotmat'], rm.rodrigues(chestj[7]['rotax'], chestj[7]['rotangle']))
        # chestj[7]['linkend'] = np.dot(chestj[7]['rotmat'], chestj[7]['linkvec']) + chestj[7]['linkpos']
        # chestj[7]['rngmin'] = -(122 - rngsafemargin)
        # chestj[7]['rngmax'] = +(182 - rngsafemargin)
        # chestj[7]['mass'] = 0
        # chestj[7]['lcntrofmass'] = 1000 * np.array([0.0, 0.0, 0.0])
        # chestj[7]['gcntrofmass'] = np.dot(chestj[7]['rotmat'], chestj[7]['lcntrofmass']) + chestj[7]['linkpos']
        # # print(chestj[7]['gcntrofmass'])
        #
        # # the 8th joint and link
        # chestj[8]['name'] = 'link6'
        # chestj[8]['mother'] = 7
        # chestj[8]['child'] = -1
        # chestj[8]['linkpos'] = chestj[6]['linkend']
        # chestj[8]['linkvec'] = np.array([0.0, 0.0, 0.0])
        # chestj[8]['rotax'] = np.array([0, 0, 1])
        # chestj[8]['rotangle'] = 0
        # chestj[8]['rotmat'] = np.dot(chestj[7]['rotmat'], rm.rodrigues(chestj[8]['rotax'], chestj[8]['rotangle']))
        # chestj[8]['linkend'] = np.dot(chestj[8]['rotmat'], chestj[8]['linkvec']) + chestj[8]['linkpos']
        # chestj[8]['rngmin'] = -(122 - rngsafemargin)
        # chestj[8]['rngmax'] = +(182 - rngsafemargin)
        # chestj[8]['mass'] = 0
        # chestj[8]['lcntrofmass'] = 1000 * np.array([0.0, 0.0, 0.0])
        # chestj[8]['gcntrofmass'] = np.dot(chestj[8]['rotmat'], chestj[8]['lcntrofmass']) + chestj[8]['linkpos']
        # # print(chestj[8]['gcntrofmass'])

        return chestj

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
            if i == 9:
                armlj[i]['rotmat'] = np.dot(np.dot(armlj[j]['rotmat'], armlj[i]['inherentR']),
                                            rm.rodrigues(armlj[i]['rotax'], armlj[i]['rotangle']))
            else:
                armlj[i]['rotmat'] = np.dot(armlj[j]['rotmat'], rm.rodrigues(armlj[i]['rotax'], armlj[i]['rotangle']))
            armlj[i]['linkend'] = np.squeeze(
                np.dot(armlj[i]['rotmat'], armlj[i]['linkvec'].reshape((-1, 1))).reshape((1, -1))) + armlj[i]['linkpos']

            """
                  An update of the center of mass location is also performed

                  author: Daniel
                  date: 20180515
                  """

            armlj[i]['gcntrofmass'] = np.squeeze(
                np.dot(armlj[i]['rotmat'], armlj[i]['lcntrofmass'].reshape((-1, 1))).reshape((1, -1))) + armlj[i][
                                          'linkpos']

            i = armlj[i]['child']

        if armlj[0]['isleg']:
            for i in range(4):
                armlj[6]['gsolevertices'][i] = np.squeeze(
                    np.dot(armlj[6]['rotmat'], armlj[6]['solevertices'][i].reshape((-1, 1))).reshape((1, -1))) + \
                                               armlj[6][
                                                   'linkpos']

        return armlj

    def goinitpose(self):
        """
        move the robot to initial pose

        :return: null

        author: weiwei
        date: 20161202, tsukuba
        """

        self.movealljnts(self.initjnts)

    def getinitarmjnts(self, armid="rgt"):
        """
        get init arm jnts by specifying armid

        :param armid:
        :return:

        date: 20180616 for ik udpate
        """

        if armid!="rgt" and armid!="lft":
            raise ValueError

        armjnts = self.initrgtjnts
        if armid == "lft":
            armjnts = self.initlftjnts

        return armjnts

    def movewaist(self, rotangle=0):
        """
        rotate the base of the robot

        :param rotangle: in degree
        :return: null

        author: weiwei
        date: 20170410
        """

        # right arm
        self.rgtarm[0]['rotangle'] = rotangle
        self.rgtarm[0]['rotmat'] = np.dot(self.__robotoriginrotmat,
                                          rm.rodrigues(self.rgtarm[0]['rotax'], self.rgtarm[0]['rotangle']))
        self.rgtarm[0]['linkend'] = np.squeeze(
            np.dot(self.rgtarm[0]['rotmat'], self.rgtarm[0]['linkvec'].reshape((-1,)))) + self.rgtarm[0]['linkpos']

        # left arm
        self.lftarm[0]['rotangle'] = rotangle
        self.lftarm[0]['rotmat'] = np.dot(self.__robotoriginrotmat,
                                          rm.rodrigues(self.lftarm[0]['rotax'], self.lftarm[0]['rotangle']))
        self.lftarm[0]['linkend'] = np.squeeze(
            np.dot(self.lftarm[0]['rotmat'], self.lftarm[0]['linkvec'].reshape((-1,)))) + self.lftarm[0]['linkpos']

        self.__updatefk(self.rgtarm)
        self.__updatefk(self.lftarm)
        self.__updatefk(self.chest)


    def movewholerobot(self, rotationmatrix=None, vectorposition=None):

        """
                move the whole robot

                :param rotangle: in degree
                :return: null

                author: Daniel
                date: 20170410
                """

        self.__robotoriginrotmat = np.dot(rotationmatrix, self.__robotoriginrotmat)
        self.robotoriginpos = self.robotoriginpos + np.dot(self.robotoriginrotmat, vectorposition)
        self.chest[0]['linkpos'] = self.robotoriginpos
        self.rgtarm[0]['linkpos'] = self.robotoriginpos
        self.lftarm[0]['linkpos'] = self.robotoriginpos
        self.rgtleg[0]['linkpos'] = self.robotoriginpos
        self.lftleg[0]['linkpos'] = self.robotoriginpos
        self.chest[0]['rotmat'] = self.__robotoriginrotmat
        self.rgtleg[0]['rotmat'] = self.__robotoriginrotmat
        self.rgtarm[0]['rotmat'] = self.__robotoriginrotmat
        self.lftleg[0]['rotmat'] = self.__robotoriginrotmat
        self.lftarm[0]['rotmat'] = self.__robotoriginrotmat

        # self.rgtleg[0]['rotmat'] = np.dot(self.__robotoriginrotmat, self.rgtleg[0]['rotmat'])
        self.rgtleg[0]['linkend'] = np.dot(self.rgtleg[0]['rotmat'], self.rgtleg[0]['linkvec']) + self.rgtleg[0][
            'linkpos'] #+ np.dot(self.robotoriginrotmat, vectorposition)
        self.rgtleg[1]['linkpos'] = self.rgtleg[0]['linkend']

        # self.rgtarm[0]['rotmat'] = np.dot(self.__robotoriginrotmat, self.rgtarm[0]['rotmat'])
        self.rgtarm[0]['linkend'] = np.dot(self.rgtarm[0]['rotmat'], self.rgtarm[0]['linkvec']) + self.rgtarm[0][
            'linkpos']# + np.dot(self.robotoriginrotmat, vectorposition)
        self.rgtarm[1]['linkpos'] = self.rgtarm[0]['linkend']

        # self.lftleg[0]['rotmat'] = np.dot(self.__robotoriginrotmat, self.lftleg[0]['rotmat'])
        self.lftleg[0]['linkend'] = np.dot(self.lftleg[0]['rotmat'], self.lftleg[0]['linkvec']) + self.lftleg[0][
            'linkpos'] #+ np.dot(self.robotoriginrotmat, vectorposition)
        self.lftleg[1]['linkpos'] = self.lftleg[0]['linkend']

        # self.lftarm[0]['rotmat'] = np.dot(self.__robotoriginrotmat, self.lftarm[0]['rotmat'])
        self.lftarm[0]['linkend'] = np.dot(self.lftarm[0]['rotmat'], self.lftarm[0]['linkvec']) + self.lftarm[0][
            'linkpos']# + np.dot(self.robotoriginrotmat, vectorposition)
        self.lftarm[1]['linkpos'] = self.lftarm[0]['linkend']

        # self.chest[0]['rotmat'] = np.dot(self.__robotoriginrotmat, self.chest[0]['rotmat'])
        self.chest[0]['linkend'] = np.dot(self.chest[0]['rotmat'], self.chest[0]['linkvec']) + self.chest[0][
            'linkpos'] #+ np.dot(self.robotoriginrotmat, vectorposition)
        self.chest[1]['linkpos'] = self.chest[0]['linkend']

        self.__updatefk(self.rgtarm)
        self.__updatefk(self.lftarm)
        self.__updatefk(self.chest)
        self.__updatefk(self.lftleg)
        self.__updatefk(self.rgtleg)


    def movearmfk(self, armjnts, armid="rgt"):
        """
        move the joints of armlj specified by targetjoints using forward kinematics, waist is not included

        :param armjnts: a 1-by-n ndarray where each element indicates the angle of a joint (in degree)
        :param armid: a string indicating the rgtlj or lftlj robot structure, could "lft" or "rgt"
        :return: null

        author: weiwei
        date: 20161205
        """

        if armid != "rgt" and armid != "lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            armlj[i]['rotangle'] = armjnts[counter]
            counter += 1
        self.__updatefk(armlj)

    def movearmfkr(self, armjnts, armid ="rgt"):
        """
        move the redundant joints of armlj using forward kinematics

        :param armjnts: [waistrot, armjnts] all in angle
        :param armid: a string indicating the rgtlj or lftlj robot structure, could "lft" or "rgt"
        :return: null

        author: weiwei
        date: 20170112
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            armlj[i]['rotangle'] = armjnts[1][counter]
            counter += 1

        self.movewaist(armjnts[0])

    def getarmjnts(self, armid="rgt"):
        """
        get the target joints of the specified armid

        :param armid:
        :return: armjnts: a 1-by-x numpy ndarray

        author: weiwei
        date: 20161205, tsukuba
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        armjnts = np.zeros(len(self.__targetjoints))
        counter = 0
        for i in self.__targetjoints:
            armjnts[counter] = armlj[i]['rotangle']
            counter += 1

        return armjnts

    def getjntwaist(self):
        """
        get the rot angle of robot waist

        :return: waistangle in degree

         author: weiwei
         date: 20170112
        """

        return self.base['rotangle']

    def chkrng(self, armjnts, armid="rgt"):
        """
        check if the given armjnts is inside the oeprating range of the speificed armid
        this function doesn't check the waist

        :param armjnts: a 1-by-6 numpy ndarray indicating the targejoints of a manipulator
        :param armid: a string "rgt" or "lft"
        :return: True or False indicating inside the range or not

        author: weiwei
        date: 20161205
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            if armjnts[counter] < armlj[i]["rngmin"] or armjnts[counter] > armlj[i]["rngmax"]:
                print( "Joint "+ str(i) + " of the " + armid + " arm is out of range")
                print( "Angle is " + str(armjnts[counter]))
                print( "Range is (" + str(armlj[i]["rngmin"]) + ", " + str(armlj[i]["rngmax"]) + ")")
                return False
            counter += 1

        return True

    def chkrngdrag(self, armjnts, armid="rgt"):
        """
        check if the given armjnts is inside the oeprating range of the speificed armid
        this function doesn't check the waist
        The joint angles out of range will be pulled back to their maxima

        :param armjnts: a 1-by-6 numpy ndarray indicating the targejoints of a manipulator
        :param armid: a string "rgt" or "lft"
        :return: Two parameters, one is true or false indicating if the joint angles are inside the range or not
                The other is the joint angles after draggin.
                If the joints were not dragged, the same joint angles will be returned

        author: weiwei
        date: 20161205
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        bdragged = True
        jntanglesdrag = []
        for i in self.__targetjoints:
            if armjnts[counter] < armlj[i]["rngmin"]:
                # print("Joint "+ str(i) + " of the " + armid + " arm is out of range")
                # print("Angle is " + str(armjnts[counter]))
                # print("Range is (" + str(armlj[i]["rngmin"]) + ", " + str(armlj[i]["rngmax"]) + ")")
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

    def numik(self, objpos, objrot, armid="rgt"):
        return hrp5pik.numik(self, objpos, objrot, armid)

    def numikr(self, objpos, objrot, armid="rgt"):
        return hrp5pik.numikr(self, objpos, objrot, armid)


    def __updatelegfk(self, leglj):
        """
        Update the structure of hrp5's leg links and joints (single)
        Note that this function should not be called explicitly
        It is called automatically by functions like movexxx

        :param leglj: the rgtlj or lftlj robot structure
        :return: null

        author: weiwei
        date: 20180616
        """

        i = 1
        while i != -1:
            j = leglj[i]['mother']
            leglj[i]['linkpos'] = leglj[j]['linkend']
            if i == 9:
                leglj[i]['rotmat'] = np.dot(np.dot(leglj[j]['rotmat'], leglj[i]['inherentR']),
                                            rm.rodrigues(leglj[i]['rotax'], leglj[i]['rotangle']))
            else:
                leglj[i]['rotmat'] = np.dot(leglj[j]['rotmat'], rm.rodrigues(leglj[i]['rotax'], leglj[i]['rotangle']))
            leglj[i]['linkend'] = np.squeeze(
                np.dot(leglj[i]['rotmat'], leglj[i]['linkvec'].reshape((-1, 1))).reshape((1, -1))) + leglj[i]['linkpos']
            i = leglj[i]['child']
        return leglj

    def goinitlegpose(self):
        """
        move the robot to initial pose

        :return: null

        author: weiwei
        date: 20180203
        """

        self.movelegfk(self.__initlegjnts[0:6], legid='rgt')
        self.movelegfk(self.__initlegjnts[6:12], legid='lft')

    def getinitlegjnts(self, legid="rgt"):
        """
        get init leg jnts by specifying legid

        :param legid:
        :return:

        date: 20180616 for ik udpate
        """

        if legid!="rgt" and legid!="lft":
            raise ep.ValueError

        legjnts = self.initrgtlegjnts
        if legid == "lft":
            legjnts = self.initlftlegjnts

        return legjnts

    def movelegfk(self, legjnts, legid="rgt"):
        """
        move the joints of leglj specified by targetjoints using forward kinematics, waist is not included

        :param legjnts: a 1-by-n ndarray where each element indicates the angle of a joint (in degree)
        :param legid: a string indicating the rgtlj or lftlj robot structure, could "lft" or "rgt"
        :return: null

        author: weiwei
        date: 20180616
        """

        if legid != "rgt" and legid != "lft":
            raise ep.ValueError

        leglj = self.rgtleg
        if legid == "lft":
            leglj = self.lftleg

        counter = 0
        for i in self.__targetlegjoints:
            leglj[i]['rotangle'] = legjnts[counter]
            counter += 1
        self.__updatefk(leglj)

    def movelegfkreturn(self, legjnts, legid="rgt"):
        """
        move the joints of leglj using forward kinematics, waist is not included

        :param legjnts: a 1-by-n ndarray where each element indicates the angle of a joint (in degree)
        :param legid: a string indicating the rgtlj or lftlj robot structure, could "lft" or "rgt"
        :return: null

        author: weiwei
        date: 20161205
        """

        if legid != "rgt" and legid != "lft":
            raise ep.ValueError

        leglj = self.rgtleg
        if legid == "lft":
            leglj = self.lftleg

        counter = 0
       # print(legjnts)
        for i in self.__targetlegjoints:
            leglj[i]['rotangle'] = legjnts[counter]
            counter += 1
        leg = self.__updatefk(leglj)

        return leg

    def movealljnts(self, robotjnts):
        """
        move all joints of the  robo

        :param robotjnts: the definition as self.initjntss
        :return: null

        author: weiwei
        date: 20161202
        """

        narmjoints = len(self.__targetjoints)
        # right arm
        i = 1
        while i != -1:
            self.rgtarm[i]['rotangle'] = robotjnts[i+2]
            i = self.rgtarm[i]['child']
        # left arm
        i = 1
        while i != -1:
            self.lftarm[i]['rotangle'] = robotjnts[i+2+narmjoints]
            i = self.lftarm[i]['child']

        # self.__updatefk(self.rgtarm)
        # self.__updatefk(self.lftarm)
        self.movewaist(robotjnts[0])



    def getlegjnts(self, legid="rgt"):
        """
        get the target joints of the specified legid

        :param legid:
        :return: legjnts: a 1-by-x numpy ndarray

        author: weiwei
        date: 20180616, osaka
        """

        if legid!="rgt" and legid!="lft":
            raise ep.ValueError

        leglj = self.rgtleg
        if legid == "lft":
            leglj = self.lftleg

        legjnts = np.zeros(len(self.__targetlegjoints))
        counter = 0
        for i in self.__targetlegjoints:
            legjnts[counter] = leglj[i]['rotangle']
            counter += 1

        return legjnts

    def chklegrng(self, legjnts, legid="rgt"):
        """
        check if the given legjnts is inside the oeprating range of the speificed legid
        this function doesn't check the waist

        :param legjnts: a 1-by-6 numpy ndarray indicating the targejoints of a manipulator
        :param legid: a string "rgt" or "lft"
        :return: True or False indicating inside the range or not

        author: weiwei
        date: 20180616
        """

        if legid!="rgt" and legid!="lft":
            raise ep.ValueError

        leglj = self.rgtleg
        if legid == "lft":
            leglj = self.lftleg

        counter = 0
        for i in self.__targetlegjoints:
            if legjnts[counter] < leglj[i]["rngmin"] or legjnts[counter] > leglj[i]["rngmax"]:
                print( "Joint "+ str(i) + " of the " + legid + " leg is out of range")
                print( "Angle is " + str(legjnts[counter]))
                print( "Range is (" + str(leglj[i]["rngmin"]) + ", " + str(leglj[i]["rngmax"]) + ")")
                return False
            counter += 1

        return True

    def chklegrngdrag(self, legjnts, legid="rgt"):
        """
        check if the given legjnts is inside the oeprating range of the speificed legid
        this function doesn't check the waist
        The joint angles out of range will be pulled back to their maxima

        :param legjnts: a 1-by-6 numpy ndarray indicating the targejoints of a manipulator
        :param legid: a string "rgt" or "lft"
        :return: Two parameters, one is true or false indicating if the joint angles are inside the range or not
                The other is the joint angles after draggin.
                If the joints were not dragged, the same joint angles will be returned

        author: weiwei
        date: 20161205
        """

        if legid!="rgt" and legid!="lft":
            raise ep.ValueError

        leglj = self.rgtleg
        if legid == "lft":
            leglj = self.lftleg

        counter = 0
        bdragged = True
        jntanglesdrag = []
        for i in self.__targetlegjoints:
            if legjnts[counter] < leglj[i]["rngmin"]:
                # print("Joint "+ str(i) + " of the " + legid + " leg is out of range")
                # print("Angle is " + str(legjnts[counter]))
                # print("Range is (" + str(leglj[i]["rngmin"]) + ", " + str(leglj[i]["rngmax"]) + ")")
                bdragged = True
                jntanglesdrag.append(leglj[i]["rngmin"])
            elif legjnts[counter] > leglj[i]["rngmax"]:
                # print("Joint "+ str(i) + " of the " + legid + " leg is out of range")
                # print("Angle is " + str(legjnts[counter]))
                # print("Range is (" + str(leglj[i]["rngmin"]) + ", " + str(leglj[i]["rngmax"]) + ")")
                bdragged = True
                jntanglesdrag.append(leglj[i]["rngmax"])
            else:
                bdragged = False
                jntanglesdrag.append(legjnts[counter])

            counter += 1

        return bdragged, jntanglesdrag

    def numlegik(self, tgtpos, tgtrot=np.eye(3), legid="rgt"):
        return hrp5plegik.numik(self, tgtpos, tgtrot, legid)

    def numik(self, objpos, objrot, armid="rgt"):
        return hrp5pik.numik(self, objpos, objrot, armid)

    def numikr(self, objpos, objrot, armid="rgt"):
        return hrp5pik.numikr(self, objpos, objrot, armid)

    def numikmsc(self, objpos, objrot, msc, armid="rgt"):
        return hrp5pik.numikmsc(self, objpos, objrot,msc, armid)

    def numikrmsc(self, objpos, objrot,msc, armid="rgt"):
        return hrp5pik.numikrmsc(self, objpos, objrot, msc, armid)

    def gencollisionmodel(self):
        """
        generate capsule based collision model for quick collision detection

        :return: capcdnode a dictionary with 'rgt' and 'lft'
        author: weiwei
        date: 20170615
        """

        # bodycapcdnode = CollisionNode("body")
        # rgtarmcapcdnode = CollisionNode("rgtarm")
        # lftarmcapcdnode = CollisionNode("lftarm")
        # capcdnode['rgt'] = []
        # capcdnode['lft'] = []
        # # rgt arm
        # i = 0
        # while i != -1:
        #     spos=self.rgtarm[i]['linkpos']
        #     epos=self.rgtarm[i]['linkend']
        #     radius = np.linalg.norm(epos-spos)/3.0
        #     capcdnode['rgt'].append(CollisionTube(spos[0], spos[1], spos[2], epos[0], epos[1], epos[2], radius))
        #     i = self.rgtarm[i]['child']
        # # lft arm
        # i = 0
        # while i != -1:
        #     spos=self.lftarm[i]['linkpos']
        #     epos=self.lftarm[i]['linkend']
        #     radius = np.linalg.norm(epos-spos)/3.0
        #     capcdnode['lft'].append(CollisionTube(spos[0], spos[1], spos[2], epos[0], epos[1], epos[2], radius))
        #     i = self.lftarm[i]['child']

        # return capcdnode
        pass



    def robotwalk(self, movingleg="rgt", newlegposition=np.array([-5, 0, -45, 75, -30, 0, 5, 0, -45, 75, -30, 0])):

        if movingleg != "rgt" and movingleg != "lft":
            print( "No valid leg chosen")
            return

        if movingleg == "rgt":
            originallegposition = copy.deepcopy(self.rgtleg)
            movedleg = self.movelegfkreturn(newlegposition, legid='rgt')


            # pggen = pg.PandaGeomGen()
            # pggen.plotAxis(base.render, spos=movedleg[6]['linkend'], pandamat4 = pg.cvtMat4(movedleg[6]['rotmat'], movedleg[6]['linkend']))
            #
            # pggen = pg.PandaGeomGen()
            pggen.plotAxis(base.render, spos=originallegposition[6]['linkend'], pandamat4 = pg.cvtMat4(originallegposition[6]['rotmat'], originallegposition[6]['linkend']), thickness = 50)


            rotationmatrix = np.dot(originallegposition[6]['rotmat'], movedleg[6]['rotmat'].T)
            # print("Difference in fixed foot", rotationmatrix)
            feetdifferencevector = originallegposition[6]['linkpos'] - movedleg[6]['linkpos']
            # print(feetdifferencevector)

            self.movewholerobot(rotationmatrix=rotationmatrix, vectorposition=feetdifferencevector)

        elif movingleg == "lft":
            originallegposition = copy.deepcopy(self.lftleg)
            movedleg = self.movelegfkreturn(newlegposition, legid='lft')

            # pggen = pg.PandaGeomGen()
            # pggen.plotAxis(base.render, spos=movedleg[6]['linkend'],
            #                pandamat4=pg.cvtMat4(movedleg[6]['rotmat'], movedleg[6]['linkend']))
            #
            # pggen = pg.PandaGeomGen()
            # pggen.plotAxis(base.render, spos=originallegposition[6]['linkend'],
            #                pandamat4=pg.cvtMat4(originallegposition[6]['rotmat'], originallegposition[6]['linkend']),
            #                thickness=50)

            rotationmatrix = np.dot(originallegposition[6]['rotmat'], movedleg[6]['rotmat'].T)
            print( "Difference in fixed foot", rotationmatrix)
            feetdifferencevector = originallegposition[6]['linkpos'] - movedleg[6]['linkpos']
            # feetdifferencevector = np.array([0,0,0])

            self.movewholerobot(rotationmatrix=rotationmatrix, vectorposition=feetdifferencevector)

        return

if __name__=="__main__":
    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    from direct.filter.CommonFilters import CommonFilters
    from manipulation.grip.hrp5pf3 import hrp5pf3
    import hrp5pmesh
    import hrp5pball as robotball
    from motionplanning import collisioncheckerball


    base = pandactrl.World()

    pggen = pg.PandaGeomGen()

    lfthnd = hrp5pf3.Hrp5pf3(hndid='lft')
    rgthnd = hrp5pf3.Hrp5pf3(hndid='rgt')

    hrp5probot = Hrp5PRobot()


    import hrp5pdynamics

    hrp5pmgen = hrp5pmesh.Hrp5PMesh(lfthand=lfthnd, rgthand=rgthnd)
    hrp5pmnp = hrp5pmgen.genmnp(hrp5probot, togglejntscoord=False)
    # hrp5pmnp.reparentTo(base.render)
    # hrp5pmnp.setH(hrp5pmnp,50)
    # hrp5psnp = hrp5pmgen.gensnp(hrp5probot)
    # hrp5psnp.reparentTo(base.render)

    legid = "rgt"
    z = -650
    tgtpos = np.array([240, -60, z])
    tgtrot = np.eye(3)
    # tgtpos = hrp5probot.rgtleg[-1]['linkend']+np.array([300,0,200])
    # tgtrot = hrp5probot.rgtleg[-1]['rotmat']
    # pggen.plotAxis(nodepath=base.render, pandamat4=pg.cvtMat4(tgtrot, tgtpos))
    rgtlegjntsgoal = hrp5probot.numlegik(tgtpos, tgtrot, legid)
    legid = "lft"
    tgtpos = np.array([-240, 60, z])
    tgtrot = np.eye(3)
    lftlegjntsgoal = hrp5probot.numlegik(tgtpos, tgtrot, legid)

    if lftlegjntsgoal is not None and rgtlegjntsgoal is not None:
        hrp5probot.movelegfk(legjnts=rgtlegjntsgoal, legid="rgt")
        hrp5probot.movelegfk(legjnts=lftlegjntsgoal, legid="lft")
    else:
        print( "NO LEG IK SOLUTION")
    print( rgtlegjntsgoal, "rgtleg jntsssss")
    HRP5PDynamics = hrp5pdynamics.Hrp5PDynamics()
    hullvertices = HRP5PDynamics.getfeetconvexhull(robot=hrp5probot)



    #
    hrp5probot.movearmfk(np.array([10.575675731105564, 9.518076865782815, -37.86754961624106, -21.32097082283939, -114.36043303735323, 16.621647349153612, -16.510458397924463, -31.07960471304081, -25.50135054150485]),armid="rgt")
    two =  hrp5probot.lftarm[-1]['linkpos']
    hrp5pikmnp2 = hrp5pmgen.genmnp(hrp5probot, togglejntscoord=False)
    hrp5pikmnp2.reparentTo(base.render)

    hrp5pikmnp = hrp5pmgen.genmnp(hrp5probot, togglejntscoord=False)
    hrp5pikmnp.reparentTo(base.render)


    base.run()
