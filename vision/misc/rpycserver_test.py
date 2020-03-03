import rpyc
from rpyc.utils.server import ThreadedServer
import time
import kntv2
import hndcam
import pickle
import threading
import numpy as np
import serial
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime

lock = threading.Lock()

class MyService(rpyc.Service):
    def on_connect(self, conn):
        print "onconnect"
        try:
            if self.__connected is True:
                return
        except:
            self.__connected = False

        if self.__connected is False:
            #test 1
            print "********"
            self.serid = 68



    def exposed_tst_fun(self):
        # test 2
        print "&&&&&&&"
        b = 233
        return b

    def exposed_testfun(self):
        print "#\n"
        b = 223
        return b



if __name__ == "__main__":
    server = ThreadedServer(MyService, hostname="10.2.0.60", port=18404)
    print "aaaaaaaa"
    a = 10
    server.start()
    # test 3
    print "@@@@@@@@"
