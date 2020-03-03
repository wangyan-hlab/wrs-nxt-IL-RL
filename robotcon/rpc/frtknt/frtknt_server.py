import grpc
import time
import yaml
import numpy as np
import robotcon.rpc.frtknt.kntv2 as kntv2
from concurrent import futures
import robotcon.rpc.frtknt.frtknt_pb2 as fkmsg
import robotcon.rpc.frtknt.frtknt_pb2_grpc as fkrpc
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime


class FrtKntServer(fkrpc.KntServicer):

    def initialize(self, kinect):
        self.__kinect = kinect
        self.__oldyaml = True
        if int(yaml.__version__[0]) >= 5:
            self.__oldyaml = False

    def getrgbarray(self, request, context):
        """
        get color image as an array

        :return: a colorHeight*colorWidth*4 np array, the second and third channels are repeated
        author: weiwei
        date: 20180207
        """

        clframe = self.__kinect.getColorFrame()
        clb = np.flip(np.array(clframe[0::4]).reshape((self.__kinect.colorHeight, self.__kinect.colorWidth)),1)
        clg = np.flip(np.array(clframe[1::4]).reshape((self.__kinect.colorHeight, self.__kinect.colorWidth)),1)
        clr = np.flip(np.array(clframe[2::4]).reshape((self.__kinect.colorHeight, self.__kinect.colorWidth)),1)
        clframe8bit = np.dstack((clb, clg, clr)).reshape((self.__kinect.colorHeight, self.__kinect.colorWidth, 3))
        return fkmsg.ReturnValue(data=yaml.dump(clframe8bit))

    def getdeptharray(self, request, context):
        """
        get depth image as an array

        :return: a depthHeight*depthWidth*3 np array, the second and third channels are repeated
        author: weiwei
        date: 20180207
        """

        dframe = self.__kinect.getDepthFrame()
        df8 = np.uint8(dframe.clip(1, 4000) / 16.)
        dframe8bit = np.dstack((df8, df8, df8)).reshape((self.__kinect.depthHeight, self.__kinect.depthWidth, 3))
        return fkmsg.ReturnValue(data=yaml.dump(dframe8bit))

    def getdepthraw(self, request, context):
        """
        get raw ddsaasaepth image

        :return: a raw depth image
        author: weiwei
        date: 20181121
        """

        dframe = self.__kinect.getDepthFrame()
        return fkmsg.ReturnValue(data=yaml.dump(dframe))

    def getpcdarray(self, request, context):
        """
        get the full poind cloud of a new frame as an array

        :param mat_tw yaml string storing mat_tw
        :return: np.array point cloud n-by-3
        author: weiwei
        date: 20181121
        """

        dframe = self.__kinect.getDepthFrame()
        if self.__oldyaml:
            mat_kw = yaml.load(request.data)
        else:
            mat_kw = yaml.load(request.data, Loader=yaml.UnsafeLoader)
        pcdarray = self.__kinect.getPointCloud(dframe, mat_tw=mat_kw)
        return fkmsg.ReturnValue(data=yaml.dump(pcdarray))

    def getpartialpcdarray(self, request, context):
        """
        get partial poind cloud using the given picklerawdframe, width, height in a depth img

        :param rawdframe yaml string storing raw dframe
        :param width, height
        :param picklemat_tw pickle string storing mat_tw
``````````````        author: weiwei
        date: 20181121
        """

        if self.__oldyaml:
            dframe = yaml.load(request.data)
        else:
            dframe = yaml.load(request.data, Loader=yaml.UnsafeLoader)
        width = [request.width.data0, request.width.data1]
        height = [request.height.data0, request.width.data1]
        if self.__oldyaml:
            mat_kw = yaml.load(request.matkw.data)
        else:
            mat_kw = yaml.load(request.matkw.data, Loader=yaml.UnsafeLoader)
        pcdarray = self.__kinect.getPointCloud(dframe, width, height, mat_kw=mat_kw)
        return fkmsg.ReturnValue(data=yaml.dump(pcdarray))

    def mapColorPointToCameraSpace(self, request, context):
        """
        convert color space  , to depth space point

        :param pt:
        :return:
        author: weiwei
        date: 20181121
        """

        return fkmsg.ReturnValue(data = self.__kinect.mapColorPointToCameraSpace([request.data0, request.data1]))

def serve():
    kinect = kntv2.KinectV2(PyKinectV2.FrameSourceTypes_Color |
                            PyKinectV2.FrameSourceTypes_Depth |
                            PyKinectRuntime.FrameSourceTypes_Infrared)
    threadKinectCam = kntv2.ThreadKinectCam(2, time.time(), kinect)
    threadKinectCam.start()
    while True:
        if kinect.getInfraredFrame() is None:
            print("initializing infrared...")
            continue
        if kinect.getColorFrame() is None:
            print("initializing color...")
            continue
        if kinect.getDepthFrame() is None:
            print("initializing depth...")
            continue
        break
    print("Kinect Initialized!")

    _ONE_DAY_IN_SECONDS = 60 * 60 * 24
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    frtserver = FrtKntServer()
    frtserver.initialize(kinect)
    fkrpc.add_KntServicer_to_server(frtserver, server)
    server.add_insecure_port('[::]:18300')
    server.start()
    print("The Front Kinect server is started!")
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)

if __name__ == "__main__":
    serve()