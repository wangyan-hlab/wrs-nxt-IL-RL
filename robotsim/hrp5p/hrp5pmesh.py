import os

import numpy as np
from manipulation.grip.hrp5three import hrp5three
from panda3d.core import *
import copy

import pandaplotutils.pandageom as pg

class Hrp5PMesh(object):
    """
    generate hrp5pmesh

    NOTE: it is unnecessary to attach a mnp to render repeatedly
    once attached, it is always there. update the joint angles will change the attached model directly
    """

    def __init__(self, rgthand = None, lfthand = None):
        """
        load models

        :param handpkg:
        author: weiwei
        date: 20180203
        """

        ##########
        ### load the model files of the robots
        ##########
        this_dir, this_filename = os.path.split(__file__)

        # chest0-2, head1 (neck is not plotted)
        hrp5pbody_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "base.egg"))
        hrp5pchest0_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "chest0.egg"))
        hrp5pchest1_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "chest1.egg"))
        hrp5pchest2_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "chest2.egg"))
        hrp5phead0_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "head0.egg"))
        hrp5phead1_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "head1.egg"))

        hrp5pbody_model = loader.loadModel(hrp5pbody_filepath)
        hrp5pchest0_model = loader.loadModel(hrp5pchest0_filepath)
        hrp5pchest1_model = loader.loadModel(hrp5pchest1_filepath)
        hrp5pchest2_model = loader.loadModel(hrp5pchest2_filepath)
        hrp5phead0_model = loader.loadModel(hrp5phead0_filepath)
        hrp5phead1_model = loader.loadModel(hrp5phead1_filepath)

        self.__hrp5pbody_nodepath = NodePath("hrp5pbody")
        self.__hrp5pchest0_nodepath = NodePath("hrp5pchest0")
        self.__hrp5pchest1_nodepath = NodePath("hrp5pchest1")
        self.__hrp5pchest2_nodepath = NodePath("hrp5pchest2")
        self.__hrp5phead0_nodepath = NodePath("hrp5phead0")
        self.__hrp5phead1_nodepath = NodePath("hrp5phead1")

        hrp5pbody_model.instanceTo(self.__hrp5pbody_nodepath)
        hrp5pchest0_model.instanceTo(self.__hrp5pchest0_nodepath)
        hrp5pchest1_model.instanceTo(self.__hrp5pchest1_nodepath)
        hrp5pchest2_model.instanceTo(self.__hrp5pchest2_nodepath)
        hrp5phead0_model.instanceTo(self.__hrp5phead0_nodepath)
        hrp5phead1_model.instanceTo(self.__hrp5phead1_nodepath)

        self.__hrp5pchest0_nodepath.reparentTo(self.__hrp5pbody_nodepath)
        self.__hrp5pchest0_nodepath.setPos(55.0, 0.0, 274.0)
        self.__hrp5pchest1_nodepath.reparentTo(self.__hrp5pchest0_nodepath)
        self.__hrp5pchest2_nodepath.reparentTo(self.__hrp5pchest1_nodepath)
        self.__hrp5phead0_nodepath.reparentTo(self.__hrp5pchest2_nodepath)
        self.__hrp5phead0_nodepath.setPos(32.0, 0.0, 549.0)
        self.__hrp5phead1_nodepath.reparentTo(self.__hrp5phead0_nodepath)

        # rgtarm
        hrp5prgtarmlj0_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rscap.egg"))
        hrp5prgtarmlj0_model = loader.loadModel(hrp5prgtarmlj0_filepath)
        self.__hrp5prgtarmlj0_nodepath = NodePath("rscap")
        hrp5prgtarmlj0_model.instanceTo(self.__hrp5prgtarmlj0_nodepath)
        #
        hrp5prgtarmlj1_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rlink0.egg"))
        hrp5prgtarmlj1_model = loader.loadModel(hrp5prgtarmlj1_filepath)
        self.__hrp5prgtarmlj1_nodepath = NodePath("rlink0")
        hrp5prgtarmlj1_model.instanceTo(self.__hrp5prgtarmlj1_nodepath)
        #
        hrp5prgtarmlj2_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rlink1.egg"))
        hrp5prgtarmlj2_model = loader.loadModel(hrp5prgtarmlj2_filepath)
        self.__hrp5prgtarmlj2_nodepath = NodePath("rlink1")
        hrp5prgtarmlj2_model.instanceTo(self.__hrp5prgtarmlj2_nodepath)
        #
        hrp5prgtarmlj3_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rlink2.egg"))
        hrp5prgtarmlj3_model = loader.loadModel(hrp5prgtarmlj3_filepath)
        self.__hrp5prgtarmlj3_nodepath = NodePath("rlink2")
        hrp5prgtarmlj3_model.instanceTo(self.__hrp5prgtarmlj3_nodepath)
        #
        hrp5prgtarmlj4_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rlink3.egg"))
        hrp5prgtarmlj4_model = loader.loadModel(hrp5prgtarmlj4_filepath)
        self.__hrp5prgtarmlj4_nodepath = NodePath("rlink3")
        hrp5prgtarmlj4_model.instanceTo(self.__hrp5prgtarmlj4_nodepath)
        #
        hrp5prgtarmlj5_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rlink4.egg"))
        hrp5prgtarmlj5_model = loader.loadModel(hrp5prgtarmlj5_filepath)
        self.__hrp5prgtarmlj5_nodepath = NodePath("rlink4")
        hrp5prgtarmlj5_model.instanceTo(self.__hrp5prgtarmlj5_nodepath)
        #
        hrp5prgtarmlj6_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rlink5.egg"))
        hrp5prgtarmlj6_model = loader.loadModel(hrp5prgtarmlj6_filepath)
        self.__hrp5prgtarmlj6_nodepath = NodePath("rlink5")
        hrp5prgtarmlj6_model.instanceTo(self.__hrp5prgtarmlj6_nodepath)
        #
        hrp5prgtarmlj7_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rlink6.egg"))
        hrp5prgtarmlj7_model = loader.loadModel(hrp5prgtarmlj7_filepath)
        self.__hrp5prgtarmlj7_nodepath = NodePath("rlink6")
        hrp5prgtarmlj7_model.instanceTo(self.__hrp5prgtarmlj7_nodepath)

        # lftarm
        hrp5plftarmlj0_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "lscap.egg"))
        hrp5plftarmlj0_model = loader.loadModel(hrp5plftarmlj0_filepath)
        self.__hrp5plftarmlj0_nodepath = NodePath("lscap")
        hrp5plftarmlj0_model.instanceTo(self.__hrp5plftarmlj0_nodepath)
        #
        hrp5plftarmlj1_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "llink0.egg"))
        hrp5plftarmlj1_model = loader.loadModel(hrp5plftarmlj1_filepath)
        self.__hrp5plftarmlj1_nodepath = NodePath("llink0")
        hrp5plftarmlj1_model.instanceTo(self.__hrp5plftarmlj1_nodepath)
        #
        hrp5plftarmlj2_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "llink1.egg"))
        hrp5plftarmlj2_model = loader.loadModel(hrp5plftarmlj2_filepath)
        self.__hrp5plftarmlj2_nodepath = NodePath("llink1")
        hrp5plftarmlj2_model.instanceTo(self.__hrp5plftarmlj2_nodepath)
        #
        hrp5plftarmlj3_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "llink2.egg"))
        hrp5plftarmlj3_model = loader.loadModel(hrp5plftarmlj3_filepath)
        self.__hrp5plftarmlj3_nodepath = NodePath("llink2")
        hrp5plftarmlj3_model.instanceTo(self.__hrp5plftarmlj3_nodepath)
        #
        hrp5plftarmlj4_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "llink3.egg"))
        hrp5plftarmlj4_model = loader.loadModel(hrp5plftarmlj4_filepath)
        self.__hrp5plftarmlj4_nodepath = NodePath("llink3")
        hrp5plftarmlj4_model.instanceTo(self.__hrp5plftarmlj4_nodepath)
        #
        hrp5plftarmlj5_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "llink4.egg"))
        hrp5plftarmlj5_model = loader.loadModel(hrp5plftarmlj5_filepath)
        self.__hrp5plftarmlj5_nodepath = NodePath("llink4")
        hrp5plftarmlj5_model.instanceTo(self.__hrp5plftarmlj5_nodepath)
        #
        hrp5plftarmlj6_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "llink5.egg"))
        hrp5plftarmlj6_model = loader.loadModel(hrp5plftarmlj6_filepath)
        self.__hrp5plftarmlj6_nodepath = NodePath("llink5")
        hrp5plftarmlj6_model.instanceTo(self.__hrp5plftarmlj6_nodepath)
        #
        hrp5plftarmlj7_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "llink6.egg"))
        hrp5plftarmlj7_model = loader.loadModel(hrp5plftarmlj7_filepath)
        self.__hrp5plftarmlj7_nodepath = NodePath("llink6")
        hrp5plftarmlj7_model.instanceTo(self.__hrp5plftarmlj7_nodepath)

        # hand
        self.rgthnd = rgthand
        self.lfthnd = lfthand

        # rgtleg
        hrp5prgtleglj0_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rleg0.egg"))
        hrp5prgtleglj0_model = loader.loadModel(hrp5prgtleglj0_filepath)
        self.__hrp5prgtleglj0_nodepath = NodePath("rleg0")
        hrp5prgtleglj0_model.instanceTo(self.__hrp5prgtleglj0_nodepath)
        #
        hrp5prgtleglj1_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rleg1.egg"))
        hrp5prgtleglj1_model = loader.loadModel(hrp5prgtleglj1_filepath)
        self.__hrp5prgtleglj1_nodepath = NodePath("rleg1")
        hrp5prgtleglj1_model.instanceTo(self.__hrp5prgtleglj1_nodepath)
        #
        hrp5prgtleglj2_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rleg2.egg"))
        hrp5prgtleglj2_model = loader.loadModel(hrp5prgtleglj2_filepath)
        self.__hrp5prgtleglj2_nodepath = NodePath("rleg2")
        hrp5prgtleglj2_model.instanceTo(self.__hrp5prgtleglj2_nodepath)
        #
        hrp5prgtleglj3_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rleg3.egg"))
        hrp5prgtleglj3_model = loader.loadModel(hrp5prgtleglj3_filepath)
        self.__hrp5prgtleglj3_nodepath = NodePath("rleg3")
        hrp5prgtleglj3_model.instanceTo(self.__hrp5prgtleglj3_nodepath)
        #
        hrp5prgtleglj4_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rleg4.egg"))
        hrp5prgtleglj4_model = loader.loadModel(hrp5prgtleglj4_filepath)
        self.__hrp5prgtleglj4_nodepath = NodePath("rleg4")
        hrp5prgtleglj4_model.instanceTo(self.__hrp5prgtleglj4_nodepath)
        #
        hrp5prgtleglj5_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "rleg5.egg"))
        hrp5prgtleglj5_model = loader.loadModel(hrp5prgtleglj5_filepath)
        self.__hrp5prgtleglj5_nodepath = NodePath("rleg5")
        hrp5prgtleglj5_model.instanceTo(self.__hrp5prgtleglj5_nodepath)

        # lftleg
        hrp5plftleglj0_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "lleg0.egg"))
        hrp5plftleglj0_model = loader.loadModel(hrp5plftleglj0_filepath)
        self.__hrp5plftleglj0_nodepath = NodePath("lleg0")
        hrp5plftleglj0_model.instanceTo(self.__hrp5plftleglj0_nodepath)
        #
        hrp5plftleglj1_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "lleg1.egg"))
        hrp5plftleglj1_model = loader.loadModel(hrp5plftleglj1_filepath)
        self.__hrp5plftleglj1_nodepath = NodePath("lleg1")
        hrp5plftleglj1_model.instanceTo(self.__hrp5plftleglj1_nodepath)
        #
        hrp5plftleglj2_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "lleg2.egg"))
        hrp5plftleglj2_model = loader.loadModel(hrp5plftleglj2_filepath)
        self.__hrp5plftleglj2_nodepath = NodePath("lleg2")
        hrp5plftleglj2_model.instanceTo(self.__hrp5plftleglj2_nodepath)
        #
        hrp5plftleglj3_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "lleg3.egg"))
        hrp5plftleglj3_model = loader.loadModel(hrp5plftleglj3_filepath)
        self.__hrp5plftleglj3_nodepath = NodePath("lleg3")
        hrp5plftleglj3_model.instanceTo(self.__hrp5plftleglj3_nodepath)
        #
        hrp5plftleglj4_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "lleg4.egg"))
        hrp5plftleglj4_model = loader.loadModel(hrp5plftleglj4_filepath)
        self.__hrp5plftleglj4_nodepath = NodePath("lleg4")
        hrp5plftleglj4_model.instanceTo(self.__hrp5plftleglj4_nodepath)
        #
        hrp5plftleglj5_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5pegg", "lleg5.egg"))
        hrp5plftleglj5_model = loader.loadModel(hrp5plftleglj5_filepath)
        self.__hrp5plftleglj5_nodepath = NodePath("lleg5")
        hrp5plftleglj5_model.instanceTo(self.__hrp5plftleglj5_nodepath)
        

        ##########
        ### load the model files of sticks
        ##########
        self.pggen = pg.PandaGeomGen()

    def gensnp(self, robot, rgtrbga=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0]), name = 'robotstick'):
        """
        generate the stick model of the nextage robot in panda3d
        snp means stick nodepath

        :param robot: the Hrp5Robot object, see Hrp5robot.py
        :param rgtrbga: color of right arm
        :param lftrgba: color of left arm
        :return: null

        author: weiwei
        date: 20180203
        """

        hrp5pstick = NodePath(name)
        i = 0
        while i != -1:
            sticknp = self.pggen.genDumbbell(spos=robot.rgtarm[i]['linkpos'],
                                             epos=robot.rgtarm[i]['linkend'],
                                             thickness=20, rgba=rgtrbga)
            i = robot.rgtarm[i]['child']
            sticknp.reparentTo(hrp5pstick)
        i = 0
        while i != -1:
            sticknp = self.pggen.genDumbbell(spos=robot.lftarm[i]['linkpos'],
                                             epos=robot.lftarm[i]['linkend'],
                                             thickness=20, rgba=lftrgba)
            i = robot.lftarm[i]['child']
            sticknp.reparentTo(hrp5pstick)

        hrp5plegstick = self.genlegsnp(robot)
        hrp5plegstick.reparentTo(hrp5pstick)

        return hrp5pstick

    def genmnp(self, robot, jawwidthrgt = 0, jawwidthlft = 0,
               toggleendcoord = False, togglejntscoord = False, name = 'robotmesh'):
        """
        generate a panda3d nodepath for the hrp5robot
        mnp indicates this function generates a mesh nodepath

        :param robot: the Hrp5Robot object, see hrp5.py
        :param toggleendcoord: whether to plot end coordinate systems
        :param togglejntscoord: whether to plot joints coordinate systems
        :return: a nodepath which is ready to be plotted using plotmesh

        author: weiwei
        date: 20180203
        """

        robotmesh_nodepath = NodePath(name)
        # body
        hrp5pbody_rotmat = pg.cvtMat4(robot.rgtarm[0]['rotmat'], robot.rgtarm[0]['linkpos'] )
        self.__hrp5pbody_nodepath.reparentTo(robotmesh_nodepath)

        #ADDED
        # self.__hrp5pchest0_nodepath.setMat(hrp5pbody_rotmat)
        # self.__hrp5pchest1_nodepath.setMat(hrp5pbody_rotmat)
        # ADDED
        self.__hrp5pchest2_nodepath.setMat(hrp5pbody_rotmat)

        # self.__hrp5phead0_nodepath.setMat(hrp5pbody_rotmat)
        # self.__hrp5phead1_nodepath.setMat(hrp5pbody_rotmat)


        self.__hrp5pbody_nodepath.setColor(0.2, 0.2, 0.2, 1)



        # rgtarm 0
        hrp5prgtarmlj0_rotmat = pg.cvtMat4(robot.rgtarm[1]['rotmat'], robot.rgtarm[1]['linkpos'])
        self.__hrp5prgtarmlj0_nodepath.setMat(hrp5prgtarmlj0_rotmat)
        self.__hrp5prgtarmlj0_nodepath.setColor(0.2, 0.2, 0.2, 1)
        # rgtarm 1
        hrp5prgtarmlj1_rotmat = pg.cvtMat4(robot.rgtarm[2]['rotmat'], robot.rgtarm[2]['linkpos'])
        self.__hrp5prgtarmlj1_nodepath.setMat(hrp5prgtarmlj1_rotmat)
        self.__hrp5prgtarmlj1_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 2
        hrp5prgtarmlj2_rotmat = pg.cvtMat4(robot.rgtarm[3]['rotmat'], robot.rgtarm[3]['linkpos'])
        self.__hrp5prgtarmlj2_nodepath.setMat(hrp5prgtarmlj2_rotmat)
        self.__hrp5prgtarmlj2_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 3
        hrp5prgtarmlj3_rotmat = pg.cvtMat4(robot.rgtarm[4]['rotmat'], robot.rgtarm[4]['linkpos'])
        self.__hrp5prgtarmlj3_nodepath.setMat(hrp5prgtarmlj3_rotmat)
        self.__hrp5prgtarmlj3_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 4
        hrp5prgtarmlj4_rotmat = pg.cvtMat4(robot.rgtarm[5]['rotmat'], robot.rgtarm[5]['linkpos'])
        self.__hrp5prgtarmlj4_nodepath.setMat(hrp5prgtarmlj4_rotmat)
        self.__hrp5prgtarmlj4_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 5
        hrp5prgtarmlj5_rotmat = pg.cvtMat4(robot.rgtarm[6]['rotmat'], robot.rgtarm[6]['linkpos'])
        self.__hrp5prgtarmlj5_nodepath.setMat(hrp5prgtarmlj5_rotmat)
        self.__hrp5prgtarmlj5_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 6
        hrp5prgtarmlj6_rotmat = pg.cvtMat4(robot.rgtarm[7]['rotmat'], robot.rgtarm[7]['linkpos'])
        self.__hrp5prgtarmlj6_nodepath.setMat(hrp5prgtarmlj6_rotmat)
        self.__hrp5prgtarmlj6_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 7
        hrp5prgtarmlj7_rotmat = pg.cvtMat4(robot.rgtarm[8]['rotmat'], robot.rgtarm[8]['linkpos'])
        self.__hrp5prgtarmlj7_nodepath.setMat(hrp5prgtarmlj7_rotmat)
        self.__hrp5prgtarmlj7_nodepath.setColor(.5, .5, .5, 1)

        self.__hrp5prgtarmlj0_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5prgtarmlj1_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5prgtarmlj2_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5prgtarmlj3_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5prgtarmlj4_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5prgtarmlj5_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5prgtarmlj6_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5prgtarmlj7_nodepath.reparentTo(robotmesh_nodepath)

        # lftarm 0
        hrp5plftarmlj0_rotmat = pg.cvtMat4(robot.lftarm[1]['rotmat'], robot.lftarm[1]['linkpos'])
        self.__hrp5plftarmlj0_nodepath.setMat(hrp5plftarmlj0_rotmat)
        self.__hrp5plftarmlj0_nodepath.setColor(0.2, 0.2, 0.2, 1)
        # lftarm 1
        hrp5plftarmlj1_rotmat = pg.cvtMat4(robot.lftarm[2]['rotmat'], robot.lftarm[2]['linkpos'])
        self.__hrp5plftarmlj1_nodepath.setMat(hrp5plftarmlj1_rotmat)
        self.__hrp5plftarmlj1_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 2
        hrp5plftarmlj2_rotmat = pg.cvtMat4(robot.lftarm[3]['rotmat'], robot.lftarm[3]['linkpos'])
        self.__hrp5plftarmlj2_nodepath.setMat(hrp5plftarmlj2_rotmat)
        self.__hrp5plftarmlj2_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 3
        hrp5plftarmlj3_rotmat = pg.cvtMat4(robot.lftarm[4]['rotmat'], robot.lftarm[4]['linkpos'])
        self.__hrp5plftarmlj3_nodepath.setMat(hrp5plftarmlj3_rotmat)
        self.__hrp5plftarmlj3_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 4
        hrp5plftarmlj4_rotmat = pg.cvtMat4(robot.lftarm[5]['rotmat'], robot.lftarm[5]['linkpos'])
        self.__hrp5plftarmlj4_nodepath.setMat(hrp5plftarmlj4_rotmat)
        self.__hrp5plftarmlj4_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 5
        hrp5plftarmlj5_rotmat = pg.cvtMat4(robot.lftarm[6]['rotmat'], robot.lftarm[6]['linkpos'])
        self.__hrp5plftarmlj5_nodepath.setMat(hrp5plftarmlj5_rotmat)
        self.__hrp5plftarmlj5_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 6
        hrp5plftarmlj6_rotmat = pg.cvtMat4(robot.lftarm[7]['rotmat'], robot.lftarm[7]['linkpos'])
        self.__hrp5plftarmlj6_nodepath.setMat(hrp5plftarmlj6_rotmat)
        self.__hrp5plftarmlj6_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 7
        hrp5plftarmlj7_rotmat = pg.cvtMat4(robot.lftarm[8]['rotmat'], robot.lftarm[8]['linkpos'])
        self.__hrp5plftarmlj7_nodepath.setMat(hrp5plftarmlj7_rotmat)
        self.__hrp5plftarmlj7_nodepath.setColor(.5, .5, .5, 1)

        self.__hrp5plftarmlj0_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5plftarmlj1_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5plftarmlj2_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5plftarmlj3_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5plftarmlj4_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5plftarmlj5_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5plftarmlj6_nodepath.reparentTo(robotmesh_nodepath)
        self.__hrp5plftarmlj7_nodepath.reparentTo(robotmesh_nodepath)

        # endcoord
        if toggleendcoord:
            self.pggen.plotAxis(robotmesh_nodepath,
                            pandamat4 = pg.cvtMat4(robot.rgtarm[-1]['rotmat'], robot.rgtarm[-1]['linkend']))
            self.pggen.plotAxis(robotmesh_nodepath,
                            pandamat4 = pg.cvtMat4(robot.lftarm[-1]['rotmat'], robot.lftarm[-1]['linkend']))
        # toggle all coord
        if togglejntscoord:
            for i in range(1, 8):
                self.pggen.plotAxis(robotmesh_nodepath, pandamat4=pg.cvtMat4(robot.rgtarm[i - 1]['rotmat'],
                                                     robot.rgtarm[i]['linkpos']))
                self.pggen.plotAxis(robotmesh_nodepath, pandamat4=pg.cvtMat4(robot.lftarm[i - 1]['rotmat'],
                                                     robot.lftarm[i]['linkpos']))

        if self.rgthnd is not None:
            self.rgthnd.setMat(pg.cvtMat4(robot.rgtarm[-1]['rotmat'], robot.rgtarm[-1]['linkpos']))
            self.rgthnd.setJawwidth(jawwidthrgt)
            self.rgthnd.reparentTo(robotmesh_nodepath)
        if self.lfthnd is not None:
            self.lfthnd.setMat(pg.cvtMat4(robot.lftarm[-1]['rotmat'], robot.lftarm[-1]['linkpos']))
            self.lfthnd.setJawwidth(jawwidthlft)
            self.lfthnd.reparentTo(robotmesh_nodepath)

        robotlegmesh_nodepath = self.genlegmnp(robot, toggleendcoord = toggleendcoord, togglejntscoord = togglejntscoord)
        robotlegmesh_nodepath.reparentTo(robotmesh_nodepath)

        return copy.deepcopy(robotmesh_nodepath)

    def genlegmnp(self, robot, toggleendcoord = True, togglejntscoord = False, name = 'legmnp'):
        """
        this function is additional to genmnp
        it is an additional function to include legs

        :param robot:
        :return: leg mesh nodepath

        author: weiwei
        date: 20180203
        """

        robotlegmesh_nodepath = NodePath(name)
        # rgtleg 0
        hrp5prgtleglj0_rotmat = pg.cvtMat4(robot.rgtleg[1]['rotmat'], robot.rgtleg[1]['linkpos'])
        self.__hrp5prgtleglj0_nodepath.setMat(hrp5prgtleglj0_rotmat)
        self.__hrp5prgtleglj0_nodepath.setColor(0.2, 0.2, 0.2, 1)
        # rgtleg 1
        hrp5prgtleglj1_rotmat = pg.cvtMat4(robot.rgtleg[2]['rotmat'], robot.rgtleg[2]['linkpos'])
        self.__hrp5prgtleglj1_nodepath.setMat(hrp5prgtleglj1_rotmat)
        self.__hrp5prgtleglj1_nodepath.setColor(.5, .5, .5, 1)
        # rgtleg 2
        hrp5prgtleglj2_rotmat = pg.cvtMat4(robot.rgtleg[3]['rotmat'], robot.rgtleg[3]['linkpos'])
        self.__hrp5prgtleglj2_nodepath.setMat(hrp5prgtleglj2_rotmat)
        self.__hrp5prgtleglj2_nodepath.setColor(.5, .5, .5, 1)
        # rgtleg 3
        hrp5prgtleglj3_rotmat = pg.cvtMat4(robot.rgtleg[4]['rotmat'], robot.rgtleg[4]['linkpos'])
        self.__hrp5prgtleglj3_nodepath.setMat(hrp5prgtleglj3_rotmat)
        self.__hrp5prgtleglj3_nodepath.setColor(.5, .5, .5, 1)
        # rgtleg 4
        hrp5prgtleglj4_rotmat = pg.cvtMat4(robot.rgtleg[5]['rotmat'], robot.rgtleg[5]['linkpos'])
        self.__hrp5prgtleglj4_nodepath.setMat(hrp5prgtleglj4_rotmat)
        self.__hrp5prgtleglj4_nodepath.setColor(.5, .5, .5, 1)
        # rgtleg 5
        hrp5prgtleglj5_rotmat = pg.cvtMat4(robot.rgtleg[6]['rotmat'], robot.rgtleg[6]['linkpos'])
        self.__hrp5prgtleglj5_nodepath.setMat(hrp5prgtleglj5_rotmat)
        self.__hrp5prgtleglj5_nodepath.setColor(.5, .5, .5, 1)

        self.__hrp5prgtleglj0_nodepath.reparentTo(robotlegmesh_nodepath)
        self.__hrp5prgtleglj1_nodepath.reparentTo(robotlegmesh_nodepath)
        self.__hrp5prgtleglj2_nodepath.reparentTo(robotlegmesh_nodepath)
        self.__hrp5prgtleglj3_nodepath.reparentTo(robotlegmesh_nodepath)
        self.__hrp5prgtleglj4_nodepath.reparentTo(robotlegmesh_nodepath)
        self.__hrp5prgtleglj5_nodepath.reparentTo(robotlegmesh_nodepath)
        
        # lftleg 0
        hrp5plftleglj0_rotmat = pg.cvtMat4(robot.lftleg[1]['rotmat'], robot.lftleg[1]['linkpos'])
        self.__hrp5plftleglj0_nodepath.setMat(hrp5plftleglj0_rotmat)
        self.__hrp5plftleglj0_nodepath.setColor(0.2, 0.2, 0.2, 1)
        # lftleg 1
        hrp5plftleglj1_rotmat = pg.cvtMat4(robot.lftleg[2]['rotmat'], robot.lftleg[2]['linkpos'])
        self.__hrp5plftleglj1_nodepath.setMat(hrp5plftleglj1_rotmat)
        self.__hrp5plftleglj1_nodepath.setColor(.5, .5, .5, 1)
        # lftleg 2
        hrp5plftleglj2_rotmat = pg.cvtMat4(robot.lftleg[3]['rotmat'], robot.lftleg[3]['linkpos'])
        self.__hrp5plftleglj2_nodepath.setMat(hrp5plftleglj2_rotmat)
        self.__hrp5plftleglj2_nodepath.setColor(.5, .5, .5, 1)
        # lftleg 3
        hrp5plftleglj3_rotmat = pg.cvtMat4(robot.lftleg[4]['rotmat'], robot.lftleg[4]['linkpos'])
        self.__hrp5plftleglj3_nodepath.setMat(hrp5plftleglj3_rotmat)
        self.__hrp5plftleglj3_nodepath.setColor(.5, .5, .5, 1)
        # lftleg 4
        hrp5plftleglj4_rotmat = pg.cvtMat4(robot.lftleg[5]['rotmat'], robot.lftleg[5]['linkpos'])
        self.__hrp5plftleglj4_nodepath.setMat(hrp5plftleglj4_rotmat)
        self.__hrp5plftleglj4_nodepath.setColor(.5, .5, .5, 1)
        # lftleg 5
        hrp5plftleglj5_rotmat = pg.cvtMat4(robot.lftleg[6]['rotmat'], robot.lftleg[6]['linkpos'])
        self.__hrp5plftleglj5_nodepath.setMat(hrp5plftleglj5_rotmat)
        self.__hrp5plftleglj5_nodepath.setColor(.5, .5, .5, 1)

        self.__hrp5plftleglj0_nodepath.reparentTo(robotlegmesh_nodepath)
        self.__hrp5plftleglj1_nodepath.reparentTo(robotlegmesh_nodepath)
        self.__hrp5plftleglj2_nodepath.reparentTo(robotlegmesh_nodepath)
        self.__hrp5plftleglj3_nodepath.reparentTo(robotlegmesh_nodepath)
        self.__hrp5plftleglj4_nodepath.reparentTo(robotlegmesh_nodepath)
        self.__hrp5plftleglj5_nodepath.reparentTo(robotlegmesh_nodepath)

        # endcoord
        if toggleendcoord:
            self.pggen.plotAxis(robotlegmesh_nodepath,
                            pandamat4 = pg.cvtMat4(robot.rgtleg[-1]['rotmat'], robot.rgtleg[-1]['linkend']))
            self.pggen.plotAxis(robotlegmesh_nodepath,
                            pandamat4 = pg.cvtMat4(robot.lftleg[-1]['rotmat'], robot.lftleg[-1]['linkend']))
        # toggle all coord
        if togglejntscoord:
            for i in range(1, 6):
                self.pggen.plotAxis(robotlegmesh_nodepath,
                                    pandamat4=pg.cvtMat4(robot.rgtleg[i-1]['rotmat'], robot.rgtleg[i]['linkpos']))
                self.pggen.plotAxis(robotlegmesh_nodepath,
                                    pandamat4=pg.cvtMat4(robot.lftleg[i-1]['rotmat'], robot.lftleg[i]['linkpos']))

        return robotlegmesh_nodepath

    def genlegsnp(self, robot, rgtrbga=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0]), name = 'rlegstick'):
        """
        this function is additional to genmnp
        it is an additional function to include legs

        :param robot:
        :return: leg mesh nodepath

        author: weiwei
        date: 20180203
        """

        hrp5plegstick = NodePath(name)
        i = 0
        while i != -1:
            sticknp = self.pggen.genDumbbell(spos=robot.rgtleg[i]['linkpos'],
                                             epos=robot.rgtleg[i]['linkend'],
                                             thickness=20, rgba=rgtrbga)
            i = robot.rgtleg[i]['child']
            sticknp.reparentTo(hrp5plegstick)
        i = 0
        while i != -1:
            sticknp = self.pggen.genDumbbell(spos=robot.lftleg[i]['linkpos'],
                                             epos=robot.lftleg[i]['linkend'],
                                             thickness=20, rgba=lftrgba)
            i = robot.lftleg[i]['child']
            sticknp.reparentTo(hrp5plegstick)

        return hrp5plegstick
