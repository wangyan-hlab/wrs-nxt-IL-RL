import os

import numpy as np
from panda3d.core import *
import copy

import pandaplotutils.pandageom as pg

class Ur3DualMesh(object):
    """
    generate ur3dualmesh

    NOTE: it is unnecessary to attach a mnp to render repeatedly
    once attached, it is always there. update the joint angles
    will change the attached model directly
    """

    def __init__(self, rgthand = None, lfthand = None):
        """
        load models

        :param rgthand: hand object
        :param lfthand: hand object
               the two hands could be different
        author: weiwei
        date: 20171213
        """

        ##########
        ### load the model files of the robots
        ##########
        this_dir, this_filename = os.path.split(__file__)

        robotwaist_filepath = os.path.join(this_dir, "ur3stl", "ur3dualbody.stl")

        ur3base_filepath = os.path.join(this_dir, "ur3stl", "base.stl")
        ur3upperarm_filepath = os.path.join(this_dir, "ur3stl", "upperarm.stl")
        ur3shoulder_filepath = os.path.join(this_dir, "ur3stl", "shoulder.stl")
        ur3forearm_filepath = os.path.join(this_dir, "ur3stl", "forearm.stl")
        ur3wrist1_filepath = os.path.join(this_dir, "ur3stl", "wrist1.stl")
        ur3wrist2_filepath = os.path.join(this_dir, "ur3stl", "wrist2.stl")
        ur3wrist3_filepath = os.path.join(this_dir, "ur3stl", "wrist3.stl")

        robotwaist_model = pg.loadstlaspandanp_fn(robotwaist_filepath)

        ur3base_model  = pg.loadstlaspandanp_fn(ur3base_filepath)
        ur3upperarm_model  = pg.loadstlaspandanp_fn(ur3upperarm_filepath)
        ur3shoulder_model = pg.loadstlaspandanp_fn(ur3shoulder_filepath)
        ur3forearm_model = pg.loadstlaspandanp_fn(ur3forearm_filepath)
        ur3wrist1_model = pg.loadstlaspandanp_fn(ur3wrist1_filepath)
        ur3wrist2_model = pg.loadstlaspandanp_fn(ur3wrist2_filepath)
        ur3wrist3_model = pg.loadstlaspandanp_fn(ur3wrist3_filepath)

        self.__robotwaist_nodepath = NodePath("ur3dualbody")
        robotwaist_model.instanceTo(self.__robotwaist_nodepath)
        self.__robotwaist_nodepath.setColor(.3, .3, .3, 1)

        # rgt
        self.__ur3rgtbase_nodepath = NodePath("ur3rgtbase")
        self.__ur3rgtshoulder_nodepath = NodePath("ur3rgtshoulder")
        self.__ur3rgtupperarm_nodepath = NodePath("ur3rgtupperarm")
        self.__ur3rgtforearm_nodepath = NodePath("ur3rgtforearm")
        self.__ur3rgtwrist1_nodepath = NodePath("ur3rgtwrist1")
        self.__ur3rgtwrist2_nodepath = NodePath("ur3rgtwrist2")
        self.__ur3rgtwrist3_nodepath = NodePath("ur3rgtwrist3")
        # base egg coordinates is retracted 89.2 along z to fit parameters
        ur3base_model.instanceTo(self.__ur3rgtbase_nodepath)
        self.__ur3rgtbase_nodepath.setColor(.5, .5, .5, 1)
        ur3shoulder_model.instanceTo(self.__ur3rgtshoulder_nodepath)
        self.__ur3rgtshoulder_nodepath.setColor(.5, .7, .3, 1)
        ur3upperarm_model.instanceTo(self.__ur3rgtupperarm_nodepath)
        self.__ur3rgtupperarm_nodepath.setColor(.5, .5, .5, 1)
        ur3forearm_model.instanceTo(self.__ur3rgtforearm_nodepath)
        self.__ur3rgtforearm_nodepath.setColor(.5, .5, .5, 1)
        ur3wrist1_model.instanceTo(self.__ur3rgtwrist1_nodepath)
        self.__ur3rgtwrist1_nodepath.setColor(.5, .7, .3, 1)
        ur3wrist2_model.instanceTo(self.__ur3rgtwrist2_nodepath)
        self.__ur3rgtwrist2_nodepath.setColor(.5, .5, .5, 1)
        ur3wrist3_model.instanceTo(self.__ur3rgtwrist3_nodepath)
        self.__ur3rgtwrist3_nodepath.setColor(.5, .5, .5, 1)
        # lft
        self.__ur3lftbase_nodepath = NodePath("ur3lftbase")
        self.__ur3lftshoulder_nodepath = NodePath("ur3lftshoulder")
        self.__ur3lftupperarm_nodepath = NodePath("ur3lftupperarm")
        self.__ur3lftforearm_nodepath = NodePath("ur3lftforearm")
        self.__ur3lftwrist1_nodepath = NodePath("ur3lftwrist1")
        self.__ur3lftwrist2_nodepath = NodePath("ur3lftwrist2")
        self.__ur3lftwrist3_nodepath = NodePath("ur3lftwrist3")
        # base egg coordinates is retracted 89.2 along z to fit parameters
        ur3base_model.instanceTo(self.__ur3lftbase_nodepath)
        self.__ur3lftbase_nodepath.setColor(.5, .5, .5, 1)
        ur3shoulder_model.instanceTo(self.__ur3lftshoulder_nodepath)
        self.__ur3lftshoulder_nodepath.setColor(.5, .7, .3, 1)
        ur3upperarm_model.instanceTo(self.__ur3lftupperarm_nodepath)
        self.__ur3lftupperarm_nodepath.setColor(.5, .5, .5, 1)
        ur3forearm_model.instanceTo(self.__ur3lftforearm_nodepath)
        self.__ur3lftforearm_nodepath.setColor(.5, .5, .5, 1)
        ur3wrist1_model.instanceTo(self.__ur3lftwrist1_nodepath)
        self.__ur3lftwrist1_nodepath.setColor(.5, .7, .3, 1)
        ur3wrist2_model.instanceTo(self.__ur3lftwrist2_nodepath)
        self.__ur3lftwrist2_nodepath.setColor(.5, .5, .5, 1)
        ur3wrist3_model.instanceTo(self.__ur3lftwrist3_nodepath)
        self.__ur3lftwrist3_nodepath.setColor(.5, .5, .5, 1)

        ##########
        ### load the model files of sticks
        ##########

        self.pggen = pg.PandaGeomGen()

        # hand
        self.rgthnd = rgthand
        self.lfthnd = lfthand

    def gensnp(self, robot, rgtrgba=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0]), name = 'robotstick'):
        """
        generate the stick model of ur3
        snp means stick nodepath

        :param robot: the ur3dualrobot
        :param rgba: color of the arm
        :return: null

        author: weiwei
        date: 20171213
        """

        ur3dualstick = NodePath(name)

        i = 0
        while i != -1:
            self.pggen.plotDumbbell(ur3dualstick, spos=robot.rgtarm[i]['linkpos'], epos=robot.rgtarm[i]['linkend'],
                            thickness=20.0, rgba=rgtrgba)
            i = robot.rgtarm[i]['child']
        i = 0
        while i != -1:
            self.pggen.plotDumbbell(ur3dualstick, spos=robot.lftarm[i]['linkpos'], epos=robot.lftarm[i]['linkend'],
                            thickness=20.0, rgba=lftrgba)
            i = robot.lftarm[i]['child']

        return ur3dualstick

    def genesnp(self, robot, rgtrgba=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0]), name = 'robotstick'):
        """
        generate the stick model of ur3
        only the end sphere (es) is drawn to show the trajectory of the end effector

        :param robot: the ur3dualrobot
        :param rbga: color of the arm
        :return: null

        author: weiwei
        date: 20181003, madrid
        """

        ur3dualstick = NodePath(name)
        self.pggen.plotSphere(ur3dualstick, pos=robot.rgtarm[-1]['linkend'], radius=25.0, rgba=rgtrgba)
        self.pggen.plotSphere(ur3dualstick, pos=robot.lftarm[-1]['linkend'], radius=25.0, rgba=lftrgba)

        return ur3dualstick

    def genmnp(self, robot, jawwidthrgt = 0, jawwidthlft = 0,
               toggleendcoord = True, togglejntscoord = False, name = 'robotmesh'):
        """
        generate the mesh model of ur5
        mnp means mesh nodepath

        :param robotjoints: the joint positions of ur5
        :param toggleendcoord: whether to plot end coordinate systems
        :param togglejntscoord: whether to plot joints coordinate systems
        :return: a nodepath which is ready to be plotted using plotmesh

        author: weiwei
        date: 20180130
        """

        robotmesh_nodepath = NodePath(name)
        # robotmesh_nodepath.setMat(pg.npToMat4(robot.base['rotmat'], robot.base['linkpos']))

        robotwaist_rotmat = pg.npToMat4(robot.base['rotmat'], npvec3=robot.base['linkpos'])
        self.__robotwaist_nodepath.setMat(robotwaist_rotmat)

        # rgt
        # base
        ur3rgtbase_rotmat = pg.npToMat4(robot.rgtarm[1]['rotmat'], robot.rgtarm[1]['linkpos'])
        self.__ur3rgtbase_nodepath.setMat(ur3rgtbase_rotmat)
        self.__ur3rgtbase_nodepath.setColor(.5,.5,.5,1)
        # shoulder
        ur3rgtshoulder_rotmat = pg.npToMat4(robot.rgtarm[1]['rotmat'], robot.rgtarm[1]['linkpos'])
        self.__ur3rgtshoulder_nodepath.setMat(ur3rgtshoulder_rotmat)
        self.__ur3rgtshoulder_nodepath.setColor(.1,.3,.5,1)
        # upperarm
        ur3rgtupperarm_rotmat = pg.npToMat4(robot.rgtarm[2]['rotmat'], robot.rgtarm[2]['linkpos'])
        self.__ur3rgtupperarm_nodepath.setMat(ur3rgtupperarm_rotmat)
        self.__ur3rgtupperarm_nodepath.setColor(.7,.7,.7,1)
        # forearm
        ur3rgtforearm_rotmat = pg.npToMat4(robot.rgtarm[3]['rotmat'], robot.rgtarm[3]['linkpos'])
        self.__ur3rgtforearm_nodepath.setMat(ur3rgtforearm_rotmat)
        self.__ur3rgtforearm_nodepath.setColor(.35,.35,.35,1)
        # wrist1
        ur3rgtwrist1_rotmat = pg.npToMat4(robot.rgtarm[4]['rotmat'], robot.rgtarm[4]['linkpos'])
        self.__ur3rgtwrist1_nodepath.setMat(ur3rgtwrist1_rotmat)
        self.__ur3rgtwrist1_nodepath.setColor(.7,.7,.7,1)
        # wrist2
        ur3rgtwrist2_rotmat = pg.npToMat4(robot.rgtarm[5]['rotmat'], robot.rgtarm[5]['linkpos'])
        self.__ur3rgtwrist2_nodepath.setMat(ur3rgtwrist2_rotmat)
        self.__ur3rgtwrist2_nodepath.setColor(.1,.3,.5,1)
        # wrist3
        ur3rgtwrist3_rotmat = pg.npToMat4(robot.rgtarm[6]['rotmat'], robot.rgtarm[6]['linkpos'])
        self.__ur3rgtwrist3_nodepath.setMat(ur3rgtwrist3_rotmat)
        self.__ur3rgtwrist3_nodepath.setColor(.5,.5,.5,1)

        self.__robotwaist_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3rgtbase_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3rgtshoulder_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3rgtupperarm_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3rgtforearm_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3rgtwrist1_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3rgtwrist2_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3rgtwrist3_nodepath.reparentTo(robotmesh_nodepath)

        # endcoord
        if toggleendcoord:
            self.pggen.plotAxis(robotmesh_nodepath,
                            spos = robot.rgtarm[-1]['linkend'],
                            pandamat3 = pg.npToMat3(robot.rgtarm[-1]['rotmat']))
        # toggle all coord
        if togglejntscoord:
            for i in range(1, 7):
                self.pggen.plotAxis(robotmesh_nodepath, spos=robot.rgtarm[i]['linkpos'],
                                pandamat3=pg.npToMat3(robot.rgtarm[i]['refcs']))

        # hand
        if self.rgthnd is not None:
            self.rgthnd.setMat(pg.npToMat4(robot.rgtarm[6]['rotmat'], robot.rgtarm[6]['linkpos']))
            self.rgthnd.setJawwidth(jawwidthrgt)
            self.rgthnd.reparentTo(robotmesh_nodepath)
 
        #lft
        # base
        ur3lftbase_rotmat = pg.npToMat4(robot.lftarm[1]['rotmat'], robot.lftarm[1]['linkpos'])
        self.__ur3lftbase_nodepath.setMat(ur3lftbase_rotmat)
        self.__ur3lftbase_nodepath.setColor(.5,.5,.5,1)
        # shoulder
        ur3lftshoulder_rotmat = pg.npToMat4(robot.lftarm[1]['rotmat'], robot.lftarm[1]['linkpos'])
        self.__ur3lftshoulder_nodepath.setMat(ur3lftshoulder_rotmat)
        self.__ur3lftshoulder_nodepath.setColor(.1,.3,.5,1)
        # upperarm
        ur3lftupperarm_rotmat = pg.npToMat4(robot.lftarm[2]['rotmat'], robot.lftarm[2]['linkpos'])
        self.__ur3lftupperarm_nodepath.setMat(ur3lftupperarm_rotmat)
        self.__ur3lftupperarm_nodepath.setColor(.7,.7,.7,1)
        # forearm
        ur3lftforearm_rotmat = pg.npToMat4(robot.lftarm[3]['rotmat'], robot.lftarm[3]['linkpos'])
        self.__ur3lftforearm_nodepath.setMat(ur3lftforearm_rotmat)
        self.__ur3lftforearm_nodepath.setColor(.35,.35,.35,1)
        # wrist1
        ur3lftwrist1_rotmat = pg.npToMat4(robot.lftarm[4]['rotmat'], robot.lftarm[4]['linkpos'])
        self.__ur3lftwrist1_nodepath.setMat(ur3lftwrist1_rotmat)
        self.__ur3lftwrist1_nodepath.setColor(.7,.7,.7,1)
        # wrist2
        ur3lftwrist2_rotmat = pg.npToMat4(robot.lftarm[5]['rotmat'], robot.lftarm[5]['linkpos'])
        self.__ur3lftwrist2_nodepath.setMat(ur3lftwrist2_rotmat)
        self.__ur3lftwrist2_nodepath.setColor(.1,.3,.5,1)
        # wrist3
        ur3lftwrist3_rotmat = pg.npToMat4(robot.lftarm[6]['rotmat'], robot.lftarm[6]['linkpos'])
        self.__ur3lftwrist3_nodepath.setMat(ur3lftwrist3_rotmat)
        self.__ur3lftwrist3_nodepath.setColor(.5,.5,.5,1)

        self.__ur3lftbase_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3lftshoulder_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3lftupperarm_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3lftforearm_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3lftwrist1_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3lftwrist2_nodepath.reparentTo(robotmesh_nodepath)
        self.__ur3lftwrist3_nodepath.reparentTo(robotmesh_nodepath)

        # endcoord
        if toggleendcoord:
            self.pggen.plotAxis(robotmesh_nodepath,
                            spos = robot.lftarm[-1]['linkend'],
                            pandamat3 = pg.npToMat3(robot.lftarm[-1]['rotmat']))
        # toggle all coord
        if togglejntscoord:
            for i in range(1, 7):
                self.pggen.plotAxis(robotmesh_nodepath, spos = robot.lftarm[i]['linkpos'],
                                pandamat3 = pg.npToMat3(robot.lftarm[i]['refcs']))

        # hand
        if self.lfthnd is not None:
            self.lfthnd.setMat(pg.npToMat4(robot.lftarm[6]['rotmat'], robot.lftarm[6]['linkpos']))
            self.lfthnd.setJawwidth(jawwidthlft)
            self.lfthnd.reparentTo(robotmesh_nodepath)
        
        return copy.deepcopy(robotmesh_nodepath)


if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    from direct.filter.CommonFilters import CommonFilters
    import ur3dual

    base = pandactrl.World(camp = [3000,0,3000], lookatp = [0,0,700])

    ur3dualrobot = ur3dual.Ur3DualRobot()
    ur3dualrobot.goinitpose()
    # ur3dualrobot.gozeropose()
    ur3dualrobotmesh = Ur3DualMesh()
    ur3meshnm = ur3dualrobotmesh.genmnp(ur3dualrobot, togglejntscoord=True)
    ur3meshnm.reparentTo(base.render)
    ur3snp = ur3dualrobotmesh.gensnp(ur3dualrobot)
    ur3snp.reparentTo(base.render)

    base.run()
