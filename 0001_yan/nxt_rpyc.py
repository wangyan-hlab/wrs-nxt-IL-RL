import rpyc
import pickle

class NxtCon():
    def __init__(self):
        conn = rpyc.connect("10.0.1.102", port=15010)
        self.__clt = conn.root

    def checkEncoders(self):
        self.__clt.checkEncoders()

    def servoOn(self):
        self.__clt.servoOn()

    def goInitial(self):
        self.__clt.goInitial()

    def goOffPose(self):
        self.__clt.goOffPose()

    def getJointAngles(self):
        jntangles = self.__clt.getJointAngles()
        return pickle.loads(jntangles)

    def setJointAngles(self, angles, tm):
        angles = pickle.dumps(angles)
        self.__clt.setJointAngles(angles, tm)

    def setJointAngle(self, jname, angle, tm):
        angle = pickle.dumps(angle)
        self.__clt.setJointAngle(jname, angle, tm)

    def setJointAnglesOfGroup(self, gname, angles, tm):
        angles = pickle.dumps(angles)
        self.__clt.setJointAnglesOfGroup(gname, angles, tm)

    def getCurrentPosition(self, lname):
        pos = self.__clt.getCurrentPosition(lname)
        return pickle.loads(pos)

    def getCurrentRPY(self, lname):
        rpy = self.__clt.getCurrentRPY(lname)
        return pickle.loads(rpy)

    def setTargetPose(self, gname, pos, rpy, tm):
        pos = pickle.dumps(pos)
        rpy = pickle.dumps(rpy)
        self.__clt.setTargetPose(gname, pos, rpy, tm)

    def playPatternOfGroup(self, para):
        para = pickle.dumps(para)
        self.__clt.playPatternOfGroup(para)

    def attachHandtoollft(self):
        self.__clt.attachHandtoollft()

    def attachHandtoolrgt(self):
        self.__clt.attachHandtoolrgt()

    def ejectHandtoollft(self):
        self.__clt.ejectHandtoollft()

    def ejectHandtoolrgt(self):
        self.__clt.ejectHandtoolrgt()

    def closeGripperlft(self):
        self.__clt.closeGripperlft()

    def closeGripperrgt(self):
        self.__clt. closeGripperrgt()

    def openGripperlft(self):
        self.__clt.openGripperlft()

    def openGripperrgt(self):
        self.__clt.openGripperrgt()

    def stopAirlft_attached(self):
        indices = pickle.dumps([23])
        assignments = pickle.dumps([18, 23])
        self.__clt.dioWriter(indices, assignments)

    def stopAirrgt_attached(self):
        indices = pickle.dumps([18])
        assignments = pickle.dumps([18, 23])
        self.__clt.dioWriter(indices, assignments)
