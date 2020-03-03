import grpc
import yaml
import robotcon.rpc.nxtrobot.nxtrobot_pb2 as nxtmsg
import robotcon.rpc.nxtrobot.nxtrobot_pb2_grpc as nxtrpc

class NxtRobot(object):

    def __init__(self, host = "localhost:18300"):
        channel = grpc.insecure_channel(host)
        self.stub = nxtrpc.NxtStub(channel)
        self.__oldyaml = True
        if int(yaml.__version__[0]) >= 5:
            self.__oldyaml = False

    def checkEncoders(self):
        returnvalue = self.stub.checkEncoders(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("Encoders succesfully checked.")

    def servoOn(self):
        returnvalue = self.stub.servoOn(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("Servos are turned on.")

    def servoOff(self):
        returnvalue = self.stub.servoOff(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("Servos are turned off.")

    def goInitial(self):
        returnvalue = self.stub.goInitial(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot is moved to its initial pose.")

    def goOffPose(self):
        returnvalue = self.stub.goOffPose(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot is moved to the off pose.")

    def getJointAngles(self):
        if self.__oldyaml:
            jntangles = yaml.load(self.stub.getJointAngles(nxtmsg.Empty()).data)
        else:
            jntangles = yaml.load(self.stub.getJointAngles(nxtmsg.Empty()).data, Loader=yaml.UnsafeLoader)
        return jntangles

    def setJointAngles(self, angles, tm = None):
        """
        All angles are in degree
        The tm is in second

        :param angles: [degree]
        :param tm: None by default
        :return:

        author: weiwei
        date: 20190417
        """

        returnvalue = self.stub.setJointAngles(nxtmsg.SendValue(data = yaml.dump([angles, tm]))).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot is moved to the given pose.")

    def playPattern(self, angleslist, tmlist = None):
        """

        :param angleslist: [[degree]]
        :param tm: [second]
        :return:

        author: weiwei
        date: 20190417
        """

        if tmlist is None:
            tmlist = [.3]*len(angleslist)
        returnvalue = self.stub.playPattern(nxtmsg.SendValue(data = yaml.dump([angleslist, tmlist]))).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot has finished the given motion.")

    def closeHandToolRgt(self):
        returnvalue = self.stub.closeHandToolRgt(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot has closed its right handtool.")

    def closeHandToolLft(self):
        returnvalue = self.stub.closeHandToolLft(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot has closed its left handtool.")

    def openHandToolRgt(self):
        returnvalue = self.stub.openHandToolRgt(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot has opened its right handtool.")

    def openHandToolLft(self):
        returnvalue = self.stub.openHandToolLft(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot has opened its left handtool.")

    def attachHandToolRgt(self):
        returnvalue = self.stub.attachHandToolRgt(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot has attached its right handtool.")

    def attachHandToolLft(self):
        returnvalue = self.stub.attachHandToolLft(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot has attached its left handtool.")

    def ejectHandToolRgt(self):
        returnvalue = self.stub.ejectHandToolRgt(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot has ejected its right handtool.")

    def ejectHandToolLft(self):
        returnvalue = self.stub.ejectHandToolLft(nxtmsg.Empty()).value
        if returnvalue == nxtmsg.Status.ERROR:
            print("Something went wrong with the server!! Try again!")
            raise Exception()
        else:
            print("The robot has ejected its left handtool.")


if __name__ == "__main__":
    nxt = NxtRobot(host = "10.0.1.114:15005")
    nxt.servoOff()
    # nxt.servoOn()
    # nxt.checkEncoders()
    # nxt.goInitial()
    # angles=[0,0,0,-15,0,-143,0,0,0,15,0,-143,0,0,0]
    # import math
    # anglesrad=[]
    # for angle in angles:
    #     anglesrad.append(math.radians(angle))
    # print(anglesrad)
    # nxt.playPattern([anglesrad], [5.0])
    # nxt.closeHandToolRgt()
    # nxt.openHandToolRgt()
    # nxt.ejectHandToolRgt()
    # nxt.goOffPose()
