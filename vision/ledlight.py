import serial

class Relay(object):
    def __init__(self, port="COM0", timeout=1):
        """

        :param port:
        """
        self.ser = serial.Serial(port, 9600, timeout = timeout)
        self.channeldict = {1:1, 2:2, 3:4, 4:8}

    def switchon(self, channel=[0,1,2,3]):
        """
        switch the relay, the ones in channel will be on
        the ones not in channel will be off

        :param channel: alist of [0,1,2,3]

        :return:

        author: weiwei
        date: 20181126
        """

        if self.ser.is_open:
            self.ser.close()
        stat = 0
        for channelid in channel:
            if channelid not in [1,2,3,4]:
                raise Exception("Wrong channel id " + str(channelid) + "!")
            stat = stat^self.channeldict[channelid]
        self.ser.open()
        print hex(stat)
        bv = "FF000"+"%x" % stat
        print bv.decode('hex')
        self.ser.write(bv.decode('hex'))
        self.ser.close()

import time
relay = Relay("COM3")
while True:
    relay.switchon([])
    time.sleep(.5)
    relay.switchon([2])
    time.sleep(.5)
    relay.switchon([3])
    time.sleep(.5)
    relay.switchon([4])
    time.sleep(.5)
