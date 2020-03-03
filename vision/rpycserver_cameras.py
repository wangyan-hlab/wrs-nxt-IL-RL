import rpyc
from rpyc.utils.server import ThreadedServer
import time
import vision.hndcam as hndcam
import pickle
import threading
import numpy as np
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime

rpyc.core.protocol.DEFAULT_CONFIG['allow_pickle'] = True
lock = threading.Lock()

class MyService(rpyc.Service):
    def on_connect(self, conn):
        print("onconnect")
        try:
            if self.__connected is True:
                return
        except:
            self.__connected = False


        if self.__connected is False:
            self.__hndcam = hndcam.HndCam(rgtcamid = [0,1], lftcamid = [2,3])
            print("Initialized!")

    def exposed_iscameraavailable(self):
        try:
            return self.__connected
        except:
            return False

    def exposed_getrc0img(self):
        """
        get an image from right handcam

        :return:
        author: weiwei
        date: 20181121
        """
        #upper camera rgt hnd
        lock.acquire()
        returnimg = []
        try:
            returnimg = self.__hndcam.getrc0img()
        finally:
            lock.release()
            return pickle.dumps(returnimg)

    def exposed_getlc0img(self):
        """
        get an image from left handcam

        :return:cxsZsAXz
        author: weiwei
        date: 20181121
        """
        # upper camera lft hnd

        lock.acquire()
        returnimg = []
        try:
            returnimg = self.__hndcam.getlc0img()
        finally:
            lock.release()
            return pickle.dumps(returnimg)

    def exposed_getrc1img(self):
        """
        get an image from right handcam

        :return:
        author: joshua
        date: 20181129
        """
        # lower camera rgt hnd
        lock.acquire()
        returnimg = []
        try:
            returnimg = self.__hndcam.getrc1img()
        finally:
            lock.release()
            return pickle.dumps(returnimg)

    def exposed_getlc1img(self):
        """
        get an image from left handcam

        :return:
        author: joshua
        date: 20181129
        """
        # lower camera lft hnd
        lock.acquire()
        returnimg = []
        try:
            returnimg = self.__hndcam.getlc1img()
        finally:
            lock.release()
            return pickle.dumps(returnimg)

if __name__ == "__main__":
    server = ThreadedServer(MyService, hostname="10.2.0.60", port = 18300)
    server.start()