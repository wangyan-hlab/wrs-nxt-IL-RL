from panda3d.core import *
from direct.filter.CommonFilters import CommonFilters
from direct.showbase.ShowBase import ShowBase
import pandaplotutils.inputmanager as im
import pandaplotutils.pandageom as pg
import os
import math

class World(ShowBase, object):

    def __init__(self, camp=[2000,500,2000], lookatp=[0,0,250], up = [0,0,1], fov = 40, w = 2000, h = 1500):
        """

        :param camp:
        :param lookatp:
        :param fov:
        :param w: width of window
        :param h: height of window
        """

        super(World, self).__init__()
        self.setBackgroundColor(1, 1, 1)

        # set up lens
        lens = PerspectiveLens()
        lens.setFov(fov)
        lens.setNearFar(1, 50000)
        self.disableMouse()
        self.cam.setPos(camp[0], camp[1], camp[2])
        self.cam.lookAt(Point3(lookatp[0], lookatp[1], lookatp[2]), Vec3(up[0], up[1], up[2]))
        self.cam.node().setLens(lens)

        # set up slight
        ablight = AmbientLight("ambientlight")
        ablight.setColor(Vec4(0.2, 0.2, 0.2, 1))
        self.__ablightnode = self.cam.attachNewNode(ablight)
        self.render.setLight(self.__ablightnode)

        ptlight0 = PointLight("pointlight1")
        ptlight0.setColor(VBase4(1, 1, 1, 1))
        self.__ptlightnode0 = self.cam.attachNewNode(ptlight0)
        self.__ptlightnode0.setPos(0, 0, 0)
        self.render.setLight(self.__ptlightnode0)

        ptlight1 = PointLight("pointlight1")
        ptlight1.setColor(VBase4(.4, .4, .4, 1))
        self.__ptlightnode1 = self.cam.attachNewNode(ptlight1)
        self.__ptlightnode1.setPos(self.cam.getPos().length(), 0, self.cam.getPos().length())
        self.render.setLight(self.__ptlightnode1)

        ptlight2 = PointLight("pointlight2")
        ptlight2.setColor(VBase4(.3, .3, .3, 1))
        self.__ptlightnode2 = self.cam.attachNewNode(ptlight2)
        self.__ptlightnode2.setPos(-self.cam.getPos().length(), 0, base.cam.getPos().length())
        self.render.setLight(self.__ptlightnode2)

        self.pg = pg
        self.pggen = pg.PandaGeomGen()

        # set up inputmanager
        self.inputmgr = im.InputManager(self, lookatp, self.pggen)
        thepot = []
        taskMgr.add(self.cycleUpdate, "cycle update", extraArgs=[thepot], appendTask=True)

        # set up rotational cam
        self.lookatp = lookatp
        # taskMgr.doMethodLater(.1, self.rotateCam, "rotate cam")

        # set window size
        props = WindowProperties()
        props.setSize(w, h)
        self.win.requestProperties(props)

        # set up cartoon effect
        self.__separation = 1
        self.filters = CommonFilters(self.win, self.cam)
        self.filters.setCartoonInk(separation=self.__separation)
        # self.setCartoonShader(True)

    def cycleUpdate(self, thepot, task):
        # reset aspect ratio
        aspectRatio = self.getAspectRatio()
        self.cam.node().getLens().setAspectRatio(aspectRatio)
        self.inputmgr.checkMouse1Drag()
        self.inputmgr.checkMouse2Drag()
        self.inputmgr.checkMouseWheel()
        # if len(thepot) > 0:
        #     for x in thepot:
        #         x.removeNode()
        # center = self.inputmgr.aimSphereCN.getSolid(0).getCenter()
        # radius = self.inputmgr.aimSphereCN.getSolid(0).getRadius()
        # thepot.append(self.pggen.plotSphere(self.render, center, radius, rgba = (1,0,0,0.2)))
        # normal = self.inputmgr.aimPlaneCN.getSolid(0).getPlane(4).getNormal()
        # thepot.append(self.pggen.plotArrow(self.render, spos=(0,0,0), epos=normal*100, thickness = 20, rgba = (1,0,0,0.2)))
        return task.cont

    def rotateCam(self, task):
        campos = self.cam.getPos()
        camangle = math.atan2(campos[1], campos[0])
        # print camangle
        if camangle < 0:
            camangle += math.pi*2
        if camangle >= math.pi*2:
            camangle = 0
        else:
            camangle += math.pi/180
        camradius = math.sqrt(campos[0]*campos[0]+campos[1]*campos[1])
        camx = camradius*math.cos(camangle)
        camy= camradius*math.sin(camangle)
        self.cam.setPos(camx, camy, campos[2])
        self.cam.lookAt(self.lookatp[0], self.lookatp[1], self.lookatp[2])
        return task.cont

    def changeLookAt(self, lookatp):
        """
        This function is questionable
        as lookat changes the rotation of the camera

        :param lookatp:
        :return:

        author: weiwei
        date: 20180606
        """

        self.cam.lookAt(lookatp[0], lookatp[1], lookatp[2])
        self.inputmgr = im.InputManager(base, lookatp, self.pggen)

    def setCartoonShader(self, switchtoon = False):
        """
        set cartoon shader, the following program is a reference
        https://github.com/panda3d/panda3d/blob/master/samples/cartoon-shader/advanced.py

        :return:

        author: weiwei
        date: 20180601
        """

        this_dir, this_filename = os.path.split(__file__)
        if switchtoon:
            lightinggen = Filename.fromOsSpecific(os.path.join(this_dir, "shaders", "lightingGen.sha"))
            tempnode = NodePath(PandaNode("temp node"))
            tempnode.setShader(loader.loadShader(lightinggen))
            self.cam.node().setInitialState(tempnode.getState())
            # self.render.setShaderInput("light", self.cam)
            self.render.setShaderInput("light", self.__ptlightnode0)
        #
        normalsBuffer = self.win.makeTextureBuffer("normalsBuffer", 0, 0)
        normalsBuffer.setClearColor(LVecBase4(0.5, 0.5, 0.5, 1))
        normalsCamera = self.makeCamera(
            normalsBuffer, lens=self.cam.node().getLens(), scene = self.render)
        normalsCamera.reparentTo(self.cam)
        normalgen = Filename.fromOsSpecific(os.path.join(this_dir, "shaders", "normalGen.sha"))
        tempnode = NodePath(PandaNode("temp node"))
        tempnode.setShader(loader.loadShader(normalgen))
        normalsCamera.node().setInitialState(tempnode.getState())

        drawnScene = normalsBuffer.getTextureCard()
        drawnScene.setTransparency(1)
        drawnScene.setColor(1, 1, 1, 0)
        drawnScene.reparentTo(render2d)
        self.drawnScene = drawnScene
        self.separation = 0.0007
        self.cutoff = 0.05
        normalgen = Filename.fromOsSpecific(os.path.join(this_dir, "shaders", "inkGen.sha"))
        drawnScene.setShader(loader.loadShader(normalgen))
        drawnScene.setShaderInput("separation", LVecBase4(self.separation, 0, self.separation, 0))
        drawnScene.setShaderInput("cutoff", LVecBase4(self.cutoff))