import grpc
import time
import yaml
import math
from concurrent import futures
import robotcon.rpc.nxtrobot.nxtrobot_pb2 as nxtmsg
import robotcon.rpc.nxtrobot.nxtrobot_pb2_grpc as nxtrpc
import robotcon.rpc.nxtrobot.nxtlib.predefinition.predefinition as predef

class NxtServer(nxtrpc.NxtServicer):
    """
    NOTE: All joint angle parameters are in degrees
    """

    __groups = [['torso', ['CHEST_JOINT0']],
              ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']],
              ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2',
                        'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']],
              ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2',
                        'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']]]
    __initpose = [0,0,0,-15,0,-143,0,0,0,15,0,-143,0,0,0]
    __offpose = OffPose = [0,0,0,25,-140,-150,45,0,0,-25,-140,-150,-45,0,0]

    def __deg2rad(self, degreelist):
        return list(map(math.radians, degreelist))

    def __rad2deg(self, radianlist):
        return list(map(math.degrees, radianlist))

    def initialize(self, robot):
        """
        MUST configure the robot in the very beginning

        :param robot: defined in ../predefinition by yan
        :return:

        author: weiwei
        date: 20190417
        """
        self.__robot = robot
        self.__oldyaml = True
        if int(yaml.__version__[0]) >= 5:
            self.__oldyaml = False

    def checkEncoders(self, request, context):
        try:
            self.__robot.checkEncoders()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def servoOn(self, request, context):
        try:
            self.__robot.servoOn()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def servoOff(self, request, context):
        try:
            self.__robot.servoOff()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def goInitial(self, request, context):
        try:
            self.__robot.goInitial()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def goOffPose(self, request, context):
        try:
            self.__robot.goOffPose()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def getJointAngles(self, request, context):
        jntangles = self.__robot.getJointAngles()
        return nxtmsg.ReturnValue(data = yaml.dump(jntangles))

    def setJointAngles(self, request, context):
        """

        :param request: request.data is in degree
        :param context:
        :return:

        author: weiwei
        date: 20190419
        """

        try:
            if self.__oldyaml:
                angles, tm = yaml.load(request.data)
            else:
                angles, tm = yaml.load(request.data, Loader = yaml.UnsafeLoader)
            if tm is None:
                tm = 10.0
            self.__robot.playPattern([self.__deg2rad(angles), [tm]])
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def playPattern(self, request, context):
        try:
            if self.__oldyaml:
                angleslist, tmlist = yaml.load(request.data)
            else:
                angleslist, tmlist = yaml.load(request.data, Loader = yaml.UnsafeLoader)
            self.__robot.playPattern(angleslist, [], [], tmlist)
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def closeHandToolRgt(self, request, context):
        try:
            self.__robot.gripper_r_close()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def closeHandToolLft(self, request, context):
        try:
            self.__robot.gripper_l_close()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def openHandToolRgt(self, request, context):
        try:
            self.__robot.gripper_r_open()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def openHandToolLft(self, request, context):
        try:
            self.__robot.gripper_l_open()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def attachHandToolRgt(self, request, context):
        try:
            self.__robot.handtool_r_attach()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def attachHandToolLft(self, request, context):
        try:
            self.__robot.handtool_l_attach()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def ejectHandToolRgt(self, request, context):
        try:
            self.__robot.handtool_r_eject()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

    def ejectHandToolLft(self, request, context):
        try:
            self.__robot.handtool_l_eject()
            return nxtmsg.Status(value = nxtmsg.Status.DONE)
        except Exception as e:
            print(e, type(e))
            return nxtmsg.Status(value = nxtmsg.Status.ERROR)

def serve():
    robot = predef.pred()

    _ONE_DAY_IN_SECONDS = 60 * 60 * 24

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    nxtserver = NxtServer()
    nxtserver.initialize(robot)
    nxtrpc.add_NxtServicer_to_server(nxtserver, server)
    server.add_insecure_port('[::]:18300')
    server.start()
    print("The Nextage Robot server is started!")
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)

if __name__ == '__main__':
    serve()