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
            self.ser = serial.Serial("COM3", 9600, timeout=1)
            self.channeldict = {1: 1, 2: 2, 3: 4, 4: 8}
            self.__connected = True

    def exposed_switchon(self, channel=[0, 1, 2, 3]):
        """
        switch the relay, the ones in channel will be on
        the ones not in channel will be off

        :param channel: alist of [0,1,2,3]

        :return:*         author: weiwei
        date: 20181126
        """

        if self.ser.is_open:
            self.ser.close()
        stat = 0
        for channelid in channel:
            if channelid not in [1, 2, 3, 4]:
                raise Exception("Wrong channel id " + str(channelid) + "!")
            stat = stat ^ self.channeldict[channelid]
        self.ser.open()
        print hex(stat)
        bv = "FF000" + "%x" % stat
        print bv.decode('hex')
        self.ser.write(bv.decode('hex'))
        self.ser.close()



if __name__ == "__main__":
    server = ThreadedServer(MyService, hostname="10.2.0.60", port=18401)
    server.start()
