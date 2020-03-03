import rpyc
from rpyc.utils.server import ThreadedServer
import time
import vision.kntv2 as kntv2
import pickle
import numpy as np
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime

class MyService(rpyc.Service):
    def on_connect(self, conn):
        print("onconnect")
        try:
            if self.__connected is True:
                return
        except:
            self.__connected = False

        if self.__connected is False:
            self.__kinect = kntv2.KinectV2(PyKinectV2.FrameSourceTypes_Color |
                                    PyKinectV2.FrameSourceTypes_Depth |
                                    PyKinectRuntime.FrameSourceTypes_Infrared)
            self.__threadKinectCam = kntv2.ThreadKinectCam(2, time.time(), self.__kinect)
            self.__threadKinectCam.start()
            while True:
                # if self.__kinect.getInfraredFrame() is None:
                #     print "initializing infrared..."
                #     continue
                if self.__kinect.getColorFrame() is None:
                    print("initializing color...")
                    continue
                if self.__kinect.getDepthFrame() is None:
                    print("initializing depth...")
                    continue
                break
            print("Initialized!")
            self.__connected = True

    def exposed_iskinectstarted(self):
        try:
            return self.__connected
        except:
            return False

    # def exposed_getifraredarray(self):
    #     """
    #     get infrared image as an array
    #
    #     :return: a infraredHeight*infraredWidth*3 np array
    #     author: weiwei
    #     date: 20180207
    #     """
    #
    #     ifframe = self.__kinect.getInfraredFrame()
    #     iff8 = np.uint8(ifframe.clip(1, 4000) / 16.)
    #     ifframe8bit = np.dstack((iff8, iff8, iff8)).reshape((self.__kinect.infraredHeight, self.__kinect.infraredWidth, 3))
    #     return pickle.dumps(ifframe8bit)

    def exposed_getrgbarray(self):
        """
        get color image as an array

        :return: a colorHeight*colorWidth*4 np array, the second and third channels are repeated
        author: weiwei
        date: 20180207
        """

        clframe = self.__kinect.getColorFrame()
        clb = np.array(clframe[0::4])
        clg = np.array(clframe[1::4])
        clr = np.array(clframe[2::4])
        cla = np.array(clframe[3::4])
        clframe8bit = np.dstack((clb, clg, clr, cla)).reshape((self.__kinect.colorHeight, self.__kinect.colorWidth, 4))
        return pickle.dumps(clframe8bit)

    def exposed_getdeptharray(self):
        """
        get depth image as an array

        :return: a depthHeight*depthWidth*3 np array, the second and third channels are repeated
        author: weiwei
        date: 20180207
        """

        dframe = self.__kinect.getDepthFrame()
        df8 = np.uint8(dframe.clip(1, 4000) / 16.)
        dframe8bit = np.dstack((df8, df8, df8)).reshape((self.__kinect.depthHeight, self.__kinect.depthWidth, 3))
        return pickle.dumps(dframe8bit)

    def exposed_getdepthraw(self):
        """
        get raw depth image

        :return: a raw depth image
        author: weiwei
        date: 20181121
        """

        dframe = self.__kinect.getDepthFrame()
        return pickle.dumps(dframe)

    def exposed_getpcdarray(self, picklemat_tw=None):
        """
        get the full poind cloud of a new frame as an array

        :param picklemat_tw pickle string storing mat_tw
        :return: np.array point cloud n-by-3
        author: weiwei
        date: 20181121
        """

        dframe = self.__kinect.getDepthFrame()
        mat_tw = None
        if picklemat_tw is not None:
            mat_tw = pickle.loads(picklemat_tw)
        pcdarray = self.__kinect.getPointCloud(dframe, mat_tw=mat_tw)
        return pickle.dumps(pcdarray)

    def exposed_getpartialpcdarray(self, picklerawdframe, width=(0, 512), height=(0, 424), picklemat_tw=None):
        """
        get partial poind cloud using the given picklerawdframe, width, height in a depth img

        :param picklerawdframe pickle string storing raw dframe
        :param width, height
        :param picklemat_tw pickle string storing mat_tw
        :return: np.array point cloud n-by-3
        author: weiwei
        date: 20181121
        """

        dframe = pickle.loads(picklerawdframe)
        mat_tw = None
        if picklemat_tw is not None:
            mat_tw = pickle.loads(picklemat_tw)
        pcdarray = self.__kinect.getPointCloud(dframe, width, height, mat_tw=mat_tw)
        return pickle.dumps(pcdarray)

    def exposed_mapColorPointToDepthSpace(self, pt):
        """
        convert color space point to depth space point

        :param pt:
        :return:
        author: weiwei
        date: 20181121
        """

        return self.__kinect.mapColorPointToDepthSpace(pt)

if __name__ == "__main__":
    server = ThreadedServer(MyService, hostname="10.2.0.60", port = 18300)
    server.start()