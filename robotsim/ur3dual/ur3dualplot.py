import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from manipulation.grip.hrp5three import hrp5three
from panda3d.core import *

import pandaplotutils.pandageom as pg

def plotstick(pandanp, ur3robot, rgtrbga=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0])):
    """
    plot the stick model of the ur3robot robot in panda3d

    :param pandanp: a panda3d nodepath
    :param ur3robot:
    :param rgtrbga: color of right arm
    :param lftrgba: color of left arm
    :return: null

    author: weiwei
    date: 20161202
    """

    i = 0
    while i != -1:
        pg.plotDumbbell(pandanp, spos=ur3robot.rgtarm[i]['linkpos'], epos=ur3robot.rgtarm[i]['linkend'],
                               thickness=20, rgba=rgtrbga)
        i = ur3robot.rgtarm[i]['child']
    i = 0
    while i != -1:
        pg.plotDumbbell(pandanp, spos=ur3robot.lftarm[i]['linkpos'], epos=ur3robot.lftarm[i]['linkend'],
                               thickness=20, rgba=lftrgba)
        i = ur3robot.lftarm[i]['child']

def genmnp(ur3dual, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a panda3d nodepath for the ur3dual
    mnp indicates this function generates a mesh nodepath

    :param ur3dual:
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20161202
    """

    ur3mnp = NodePath("ur3dualmnp")

    this_dir, this_filename = os.path.split(__file__)

    # chest0-2, head1 (neck is not plotted)
    ur3base_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "base.egg"))
    ur3upperarm_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "upperarm.egg"))
    ur3shoulder_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "shoulder.egg"))
    ur3forearm_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "forearm.egg"))
    ur3wrist1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "wrist1.egg"))
    ur3wrist2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "wrist2.egg"))
    ur3wrist3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "wrist3.egg"))

    ur3base_model  = loader.loadModel(ur3base_filepath)
    ur3upperarm_model  = loader.loadModel(ur3upperarm_filepath)
    ur3shoulder_model = loader.loadModel(ur3shoulder_filepath)
    ur3forearm_model = loader.loadModel(ur3forearm_filepath)
    ur3wrist1_model = loader.loadModel(ur3wrist1_filepath)
    ur3wrist2_model = loader.loadModel(ur3wrist2_filepath)
    ur3wrist3_model = loader.loadModel(ur3wrist3_filepath)

    # rgt
    ur3rgtbase_nodepath = NodePath("ur3rgtbase")
    ur3rgtshoulder_nodepath = NodePath("ur3rgtshoulder")
    ur3rgtupperarm_nodepath = NodePath("ur3rgtupperarm")
    ur3rgtforearm_nodepath = NodePath("ur3rgtforearm")
    ur3rgtwrist1_nodepath = NodePath("ur3rgtwrist1")
    ur3rgtwrist2_nodepath = NodePath("ur3rgtwrist2")
    ur3rgtwrist3_nodepath = NodePath("ur3rgtwrist3")
    #
    # base egg coordinates is retracted 89.2 along z to fit parameters
    ur3base_model.instanceTo(ur3rgtbase_nodepath)
    ur3base_rotmat = pg.cvtMat4(ur3dual.rgtarm[1]['inherentR'], ur3dual.rgtarm[1]['linkpos'])
    ur3rgtbase_nodepath.setMat(ur3base_rotmat)
    ur3rgtbase_nodepath.setColor(.5,.5,.5,1)
    ur3rgtbase_nodepath.reparentTo(ur3mnp)
    #
    ur3shoulder_model.instanceTo(ur3rgtshoulder_nodepath)
    ur3shoulder_rotmat = pg.cvtMat4(ur3dual.rgtarm[1]['rotmat'], ur3dual.rgtarm[1]['linkpos'])
    ur3rgtshoulder_nodepath.setMat(ur3shoulder_rotmat)
    ur3rgtshoulder_nodepath.setColor(.5,.7,.3,1)
    ur3rgtshoulder_nodepath.reparentTo(ur3mnp)
    #
    ur3upperarm_model.instanceTo(ur3rgtupperarm_nodepath)
    ur3upperarm_rotmat = pg.cvtMat4(ur3dual.rgtarm[2]['rotmat'], ur3dual.rgtarm[2]['linkpos'])
    ur3rgtupperarm_nodepath.setMat(ur3upperarm_rotmat)
    ur3rgtupperarm_nodepath.setColor(.5,.5,.5,1)
    ur3rgtupperarm_nodepath.reparentTo(ur3mnp)
    #
    ur3forearm_model.instanceTo(ur3rgtforearm_nodepath)
    ur3forearm_rotmat = pg.cvtMat4(ur3dual.rgtarm[3]['rotmat'], ur3dual.rgtarm[3]['linkpos'])
    ur3rgtforearm_nodepath.setMat(ur3forearm_rotmat)
    ur3rgtforearm_nodepath.setColor(.5,.5,.5,1)
    ur3rgtforearm_nodepath.reparentTo(ur3mnp)
    #
    ur3wrist1_model.instanceTo(ur3rgtwrist1_nodepath)
    ur3wrist1_rotmat = pg.cvtMat4(ur3dual.rgtarm[4]['rotmat'], ur3dual.rgtarm[4]['linkpos'])
    ur3rgtwrist1_nodepath.setMat(ur3wrist1_rotmat)
    ur3rgtwrist1_nodepath.setColor(.5,.7,.3,1)
    ur3rgtwrist1_nodepath.reparentTo(ur3mnp)
    #
    ur3wrist2_model.instanceTo(ur3rgtwrist2_nodepath)
    ur3wrist2_rotmat = pg.cvtMat4(ur3dual.rgtarm[5]['rotmat'], ur3dual.rgtarm[5]['linkpos'])
    ur3rgtwrist2_nodepath.setMat(ur3wrist2_rotmat)
    ur3rgtwrist2_nodepath.setColor(.5,.5,.5,1)
    ur3rgtwrist2_nodepath.reparentTo(ur3mnp)
    #
    # wrist3 egg coordinates is rotated 90 around z retracted 72.33752-81.82489 to fit parameters
    ur3wrist3_model.instanceTo(ur3rgtwrist3_nodepath)
    ur3wrist3_rotmat = pg.cvtMat4(ur3dual.rgtarm[6]['rotmat'], ur3dual.rgtarm[6]['linkpos'])
    ur3rgtwrist3_nodepath.setMat(ur3wrist3_rotmat)
    ur3rgtwrist3_nodepath.setColor(.5,.5,.5,1)
    ur3rgtwrist3_nodepath.reparentTo(ur3mnp)

    # lft
    ur3lftbase_nodepath = NodePath("ur3lftbase")
    ur3lftupperarm_nodepath = NodePath("ur3lftupperarm")
    ur3lftshoulder_nodepath = NodePath("ur3lftshoulder")
    ur3lftforearm_nodepath = NodePath("ur3lftforearm")
    ur3lftwrist1_nodepath = NodePath("ur3lftwrist1")
    ur3lftwrist2_nodepath = NodePath("ur3lftwrist2")
    ur3lftwrist3_nodepath = NodePath("ur3lftwrist3")
    #
    # base egg coordinates is retracted 89.2 along z to fit parameters
    ur3base_model.instanceTo(ur3lftbase_nodepath)
    ur3base_rotmat = pg.cvtMat4(ur3dual.lftarm[1]['inherentR'], ur3dual.lftarm[1]['linkpos'])
    ur3lftbase_nodepath.setMat(ur3base_rotmat)
    ur3lftbase_nodepath.setColor(.5,.5,.5,1)
    ur3lftbase_nodepath.reparentTo(ur3mnp)
    #
    ur3shoulder_model.instanceTo(ur3lftshoulder_nodepath)
    ur3shoulder_rotmat = pg.cvtMat4(ur3dual.lftarm[1]['rotmat'], ur3dual.lftarm[1]['linkpos'])
    ur3lftshoulder_nodepath.setMat(ur3shoulder_rotmat)
    ur3lftshoulder_nodepath.setColor(.5,.7,.3,1)
    ur3lftshoulder_nodepath.reparentTo(ur3mnp)
    #
    ur3upperarm_model.instanceTo(ur3lftupperarm_nodepath)
    ur3upperarm_rotmat = pg.cvtMat4(ur3dual.lftarm[2]['rotmat'], ur3dual.lftarm[2]['linkpos'])
    ur3lftupperarm_nodepath.setMat(ur3upperarm_rotmat)
    ur3lftupperarm_nodepath.setColor(.5,.5,.5,1)
    ur3lftupperarm_nodepath.reparentTo(ur3mnp)
    #
    ur3forearm_model.instanceTo(ur3lftforearm_nodepath)
    ur3forearm_rotmat = pg.cvtMat4(ur3dual.lftarm[3]['rotmat'], ur3dual.lftarm[3]['linkpos'])
    ur3lftforearm_nodepath.setMat(ur3forearm_rotmat)
    ur3lftforearm_nodepath.setColor(.5,.5,.5,1)
    ur3lftforearm_nodepath.reparentTo(ur3mnp)
    #
    ur3wrist1_model.instanceTo(ur3lftwrist1_nodepath)
    ur3wrist1_rotmat = pg.cvtMat4(ur3dual.lftarm[4]['rotmat'], ur3dual.lftarm[4]['linkpos'])
    ur3lftwrist1_nodepath.setMat(ur3wrist1_rotmat)
    ur3lftwrist1_nodepath.setColor(.5,.7,.3,1)
    ur3lftwrist1_nodepath.reparentTo(ur3mnp)
    #
    ur3wrist2_model.instanceTo(ur3lftwrist2_nodepath)
    ur3wrist2_rotmat = pg.cvtMat4(ur3dual.lftarm[5]['rotmat'], ur3dual.lftarm[5]['linkpos'])
    ur3lftwrist2_nodepath.setMat(ur3wrist2_rotmat)
    ur3lftwrist2_nodepath.setColor(.5,.5,.5,1)
    ur3lftwrist2_nodepath.reparentTo(ur3mnp)
    #
    ur3wrist3_model.instanceTo(ur3lftwrist3_nodepath)
    ur3wrist3_rotmat = pg.cvtMat4(ur3dual.lftarm[6]['rotmat'], ur3dual.lftarm[6]['linkpos'])
    ur3lftwrist3_nodepath.setMat(ur3wrist3_rotmat)
    ur3lftwrist3_nodepath.setColor(.5,.5,.5,1)
    ur3lftwrist3_nodepath.reparentTo(ur3mnp)

    # rgthnd
    ur3robotrgthnd = handpkg.newHand('rgt', ftsensoroffset = -50)
    ur3robotrgtarmljend_rotmat = pg.cvtMat4(ur3dual.rgtarm[6]['rotmat'], ur3dual.rgtarm[6]['linkpos'])
    # pg.plotAxisSelf(ur3mnp, ur3dual.rgtarm[6]['linkend'], ur3robotrgtarmljend_rotmat)
    ur3robotrgthnd.setMat(ur3robotrgtarmljend_rotmat)
    ur3robotrgthnd.reparentTo(ur3mnp)
    if jawwidthrgt is not None:
        ur3robotrgthnd.setJawwidth(jawwidthrgt)

    # lfthnd
    ur3robotlfthnd = handpkg.newHand('lft', ftsensoroffset = -50)
    ur3robotlftarmljend_rotmat = pg.cvtMat4(ur3dual.lftarm[6]['rotmat'], ur3dual.lftarm[6]['linkpos'])
    # pg.plotAxisSelf(ur3mnp, ur3dual.lftarm[6]['linkend'], ur3robotlftarmljend_rotmat)
    ur3robotlfthnd.setMat(ur3robotlftarmljend_rotmat)
    ur3robotlfthnd.reparentTo(ur3mnp)
    if jawwidthlft is not None:
        ur3robotlfthnd.setJawwidth(jawwidthlft)

    return ur3mnp


def genmnplist(ur3dual, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a list of panda3d nodepath for the ur3dual
    mnp indicates this function generates a mesh nodepath

    the return value is in the following format:
    [[rightarm mnp list], [leftarm mnp list], [body]]
    # Right goes first!
    # The order of an arm mnp list is from base to end-effector

    :param ur3dual:
    :return: a list of mesh nodepaths

    author: weiwei
    date: 20170608
    """

    ur3mnp = NodePath("ur3dualmnp")

    this_dir, this_filename = os.path.split(__file__)

    # chest0-2, head1 (neck is not plotted)
    ur3base_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "base.egg"))
    ur3upperarm_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "upperarm.egg"))
    ur3shoulder_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "shoulder.egg"))
    ur3forearm_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "forearm.egg"))
    ur3wrist1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "wrist1.egg"))
    ur3wrist2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "wrist2.egg"))
    ur3wrist3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "wrist3.egg"))

    ur3base_model  = loader.loadModel(ur3base_filepath)
    ur3upperarm_model  = loader.loadModel(ur3upperarm_filepath)
    ur3shoulder_model = loader.loadModel(ur3shoulder_filepath)
    ur3forearm_model = loader.loadModel(ur3forearm_filepath)
    ur3wrist1_model = loader.loadModel(ur3wrist1_filepath)
    ur3wrist2_model = loader.loadModel(ur3wrist2_filepath)
    ur3wrist3_model = loader.loadModel(ur3wrist3_filepath)

    # rgt
    ur3rgtbase_nodepath = NodePath("ur3rgtbase")
    ur3rgtshoulder_nodepath = NodePath("ur3rgtshoulder")
    ur3rgtupperarm_nodepath = NodePath("ur3rgtupperarm")
    ur3rgtforearm_nodepath = NodePath("ur3rgtforearm")
    ur3rgtwrist1_nodepath = NodePath("ur3rgtwrist1")
    ur3rgtwrist2_nodepath = NodePath("ur3rgtwrist2")
    ur3rgtwrist3_nodepath = NodePath("ur3rgtwrist3")
    #
    # base egg coordinates is retracted 89.2 along z to fit parameters
    ur3base_model.instanceTo(ur3rgtbase_nodepath)
    ur3base_rotmat = pg.cvtMat4(ur3dual.rgtarm[1]['inherentR'], ur3dual.rgtarm[1]['linkpos'])
    ur3rgtbase_nodepath.setMat(ur3base_rotmat)
    ur3rgtbase_nodepath.setColor(.5,.5,.5,1)
    ur3rgtbase_nodepath.reparentTo(ur3mnp)
    #
    ur3shoulder_model.instanceTo(ur3rgtshoulder_nodepath)
    ur3shoulder_rotmat = pg.cvtMat4(ur3dual.rgtarm[1]['rotmat'], ur3dual.rgtarm[1]['linkpos'])
    ur3rgtshoulder_nodepath.setMat(ur3shoulder_rotmat)
    ur3rgtshoulder_nodepath.setColor(.5,.7,.3,1)
    ur3rgtshoulder_nodepath.reparentTo(ur3mnp)
    #
    ur3upperarm_model.instanceTo(ur3rgtupperarm_nodepath)
    ur3upperarm_rotmat = pg.cvtMat4(ur3dual.rgtarm[2]['rotmat'], ur3dual.rgtarm[2]['linkpos'])
    ur3rgtupperarm_nodepath.setMat(ur3upperarm_rotmat)
    ur3rgtupperarm_nodepath.setColor(.5,.5,.5,1)
    ur3rgtupperarm_nodepath.reparentTo(ur3mnp)
    #
    ur3forearm_model.instanceTo(ur3rgtforearm_nodepath)
    ur3forearm_rotmat = pg.cvtMat4(ur3dual.rgtarm[3]['rotmat'], ur3dual.rgtarm[3]['linkpos'])
    ur3rgtforearm_nodepath.setMat(ur3forearm_rotmat)
    ur3rgtforearm_nodepath.setColor(.5,.5,.5,1)
    ur3rgtforearm_nodepath.reparentTo(ur3mnp)
    #
    ur3wrist1_model.instanceTo(ur3rgtwrist1_nodepath)
    ur3wrist1_rotmat = pg.cvtMat4(ur3dual.rgtarm[4]['rotmat'], ur3dual.rgtarm[4]['linkpos'])
    ur3rgtwrist1_nodepath.setMat(ur3wrist1_rotmat)
    ur3rgtwrist1_nodepath.setColor(.5,.7,.3,1)
    ur3rgtwrist1_nodepath.reparentTo(ur3mnp)
    #
    ur3wrist2_model.instanceTo(ur3rgtwrist2_nodepath)
    ur3wrist2_rotmat = pg.cvtMat4(ur3dual.rgtarm[5]['rotmat'], ur3dual.rgtarm[5]['linkpos'])
    ur3rgtwrist2_nodepath.setMat(ur3wrist2_rotmat)
    ur3rgtwrist2_nodepath.setColor(.5,.5,.5,1)
    ur3rgtwrist2_nodepath.reparentTo(ur3mnp)
    #
    # wrist3 egg coordinates is rotated 90 around z retracted 72.33752-81.82489 to fit parameters
    ur3wrist3_model.instanceTo(ur3rgtwrist3_nodepath)
    ur3wrist3_rotmat = pg.cvtMat4(ur3dual.rgtarm[6]['rotmat'], ur3dual.rgtarm[6]['linkpos'])
    ur3rgtwrist3_nodepath.setMat(ur3wrist3_rotmat)
    ur3rgtwrist3_nodepath.setColor(.5,.5,.5,1)
    ur3rgtwrist3_nodepath.reparentTo(ur3mnp)

    # lft
    ur3lftbase_nodepath = NodePath("ur3lftbase")
    ur3lftupperarm_nodepath = NodePath("ur3lftupperarm")
    ur3lftshoulder_nodepath = NodePath("ur3lftshoulder")
    ur3lftforearm_nodepath = NodePath("ur3lftforearm")
    ur3lftwrist1_nodepath = NodePath("ur3lftwrist1")
    ur3lftwrist2_nodepath = NodePath("ur3lftwrist2")
    ur3lftwrist3_nodepath = NodePath("ur3lftwrist3")
    #
    # base egg coordinates is retracted 89.2 along z to fit parameters
    ur3base_model.instanceTo(ur3lftbase_nodepath)
    ur3base_rotmat = pg.cvtMat4(ur3dual.lftarm[1]['inherentR'], ur3dual.lftarm[1]['linkpos'])
    ur3lftbase_nodepath.setMat(ur3base_rotmat)
    ur3lftbase_nodepath.setColor(.5,.5,.5,1)
    ur3lftbase_nodepath.reparentTo(ur3mnp)
    #
    ur3shoulder_model.instanceTo(ur3lftshoulder_nodepath)
    ur3shoulder_rotmat = pg.cvtMat4(ur3dual.lftarm[1]['rotmat'], ur3dual.lftarm[1]['linkpos'])
    ur3lftshoulder_nodepath.setMat(ur3shoulder_rotmat)
    ur3lftshoulder_nodepath.setColor(.5,.7,.3,1)
    ur3lftshoulder_nodepath.reparentTo(ur3mnp)
    #
    ur3upperarm_model.instanceTo(ur3lftupperarm_nodepath)
    ur3upperarm_rotmat = pg.cvtMat4(ur3dual.lftarm[2]['rotmat'], ur3dual.lftarm[2]['linkpos'])
    ur3lftupperarm_nodepath.setMat(ur3upperarm_rotmat)
    ur3lftupperarm_nodepath.setColor(.5,.5,.5,1)
    ur3lftupperarm_nodepath.reparentTo(ur3mnp)
    #
    ur3forearm_model.instanceTo(ur3lftforearm_nodepath)
    ur3forearm_rotmat = pg.cvtMat4(ur3dual.lftarm[3]['rotmat'], ur3dual.lftarm[3]['linkpos'])
    ur3lftforearm_nodepath.setMat(ur3forearm_rotmat)
    ur3lftforearm_nodepath.setColor(.5,.5,.5,1)
    ur3lftforearm_nodepath.reparentTo(ur3mnp)
    #
    ur3wrist1_model.instanceTo(ur3lftwrist1_nodepath)
    ur3wrist1_rotmat = pg.cvtMat4(ur3dual.lftarm[4]['rotmat'], ur3dual.lftarm[4]['linkpos'])
    ur3lftwrist1_nodepath.setMat(ur3wrist1_rotmat)
    ur3lftwrist1_nodepath.setColor(.5,.7,.3,1)
    ur3lftwrist1_nodepath.reparentTo(ur3mnp)
    #
    ur3wrist2_model.instanceTo(ur3lftwrist2_nodepath)
    ur3wrist2_rotmat = pg.cvtMat4(ur3dual.lftarm[5]['rotmat'], ur3dual.lftarm[5]['linkpos'])
    ur3lftwrist2_nodepath.setMat(ur3wrist2_rotmat)
    ur3lftwrist2_nodepath.setColor(.5,.5,.5,1)
    ur3lftwrist2_nodepath.reparentTo(ur3mnp)
    #
    ur3wrist3_model.instanceTo(ur3lftwrist3_nodepath)
    ur3wrist3_rotmat = pg.cvtMat4(ur3dual.lftarm[6]['rotmat'], ur3dual.lftarm[6]['linkpos'])
    ur3lftwrist3_nodepath.setMat(ur3wrist3_rotmat)
    ur3lftwrist3_nodepath.setColor(.5,.5,.5,1)
    ur3lftwrist3_nodepath.reparentTo(ur3mnp)

    # rgthnd
    ur3robotrgthnd = handpkg.newHand('rgt', ftsensoroffset = -50)
    ur3robotrgtarmljend_rotmat = pg.cvtMat4(ur3dual.rgtarm[6]['rotmat'], ur3dual.rgtarm[6]['linkpos'])
    pg.plotAxisSelf(ur3mnp, ur3dual.rgtarm[6]['linkend'], ur3robotrgtarmljend_rotmat)
    ur3robotrgthnd.setMat(ur3robotrgtarmljend_rotmat)
    ur3robotrgthnd.reparentTo(ur3mnp)
    if jawwidthrgt is not None:
        ur3robotrgthnd.setJawwidth(jawwidthrgt)

    # lfthnd
    ur3robotlfthnd = handpkg.newHand('lft', ftsensoroffset = -50)
    ur3robotlftarmljend_rotmat = pg.cvtMat4(ur3dual.lftarm[6]['rotmat'], ur3dual.lftarm[6]['linkpos'])
    pg.plotAxisSelf(ur3mnp, ur3dual.lftarm[6]['linkend'], ur3robotlftarmljend_rotmat)
    ur3robotlfthnd.setMat(ur3robotlftarmljend_rotmat)
    ur3robotlfthnd.reparentTo(ur3mnp)
    if jawwidthlft is not None:
        ur3robotlfthnd.setJawwidth(jawwidthlft)

    return [[ur3rgtbase_nodepath, ur3rgtshoulder_nodepath, ur3rgtupperarm_nodepath,
             ur3rgtforearm_nodepath, ur3rgtwrist1_nodepath, ur3rgtwrist2_nodepath,
             ur3rgtwrist3_nodepath, ur3robotrgthnd.handnp],
            [ur3lftbase_nodepath, ur3lftshoulder_nodepath, ur3lftupperarm_nodepath,
             ur3lftforearm_nodepath, ur3lftwrist1_nodepath, ur3lftwrist2_nodepath,
             ur3lftwrist3_nodepath, ur3robotlfthnd.handnp],
            []]

def genmnp_nm(ur3dual, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a panda3d nodepath for the ur3dual
    mnp indicates this function generates a mesh nodepath

    :param ur3dual:
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20161202
    """

    return genmnp(ur3dual, handpkg, jawwidthrgt, jawwidthlft)