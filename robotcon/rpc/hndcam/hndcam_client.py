import grpc
import yaml
import robotcon.rpc.hndcam.hndcam_pb2 as hcmsg
import robotcon.rpc.hndcam.hndcam_pb2_grpc as hcrpc

import cv2

class HndCam(object):

    def __init__(self, host = "localhost:18300"):
        self.__oldyaml = True
        if int(yaml.__version__[0]) >= 5:
            self.__oldyaml = False
        channel = grpc.insecure_channel(host)
        self.stub = hcrpc.CamStub(channel)

    def getrc0img(self):
        if self.__oldyaml:
            return yaml.load(self.stub.getrc0img(hcmsg.Empty()).data)
        else:
            return yaml.load(self.stub.getrc0img(hcmsg.Empty()).data, Loader=yaml.UnsafeLoader)

    def getrc1img(self):
        if self.__oldyaml:
            return yaml.load(self.stub.getrc1img(hcmsg.Empty()).data)
        else:
            return yaml.load(self.stub.getrc1img(hcmsg.Empty()).data, Loader=yaml.UnsafeLoader)

    def getlc0img(self):
        if self.__oldyaml:
            return yaml.load(self.stub.getlc0img(hcmsg.Empty()).data)
        else:
            return yaml.load(self.stub.getlc0img(hcmsg.Empty()).data, Loader=yaml.UnsafeLoader)

    def getlc1img(self):
        if self.__oldyaml:
            return yaml.load(self.stub.getlc1img(hcmsg.Empty()).data)
        else:
            return yaml.load(self.stub.getlc1img(hcmsg.Empty()).data, Loader=yaml.UnsafeLoader)

if __name__=="__main__":
    hcc = HndCam(host = "localhost:18300")
    imgx = hcc.getrc0img()
    cv2.imshow("name", imgx)
    cv2.waitKey(0)