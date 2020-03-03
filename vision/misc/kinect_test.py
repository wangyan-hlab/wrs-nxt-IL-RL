#!/usr/bin/python

import cv2
import time
import kntv2
# import jason
import numpy as np
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime

kinect = kntv2.KinectV2(PyKinectV2.FrameSourceTypes_Color |
                        PyKinectV2.FrameSourceTypes_Depth |
                        PyKinectRuntime.FrameSourceTypes_Infrared)
threadKinectCam = kntv2.ThreadKinectCam(2, time.time(), kinect)
threadKinectCam.start()

while True:
    # if kinect.getInfraredFrame() is None:
    #     print "no infrared"
    #     continue
    if kinect.getColorFrame() is None:
        print "no color"
        continue
    if kinect.getDepthFrame() is None:
        print "no depth"
        continue
    # infrared
    # ifframe = kinect.getInfraredFrame()
    # iff8 = np.uint8(ifframe.clip(1, 4000) / 16.)
    # ifframe8bit = np.dstack((iff8, iff8, iff8)).reshape((kinect.infraredHeight, kinect.infraredWidth, 3))
    # color
    # clframe = kinect.getColorFrame()
    # clb = np.array(clframe[0::4])
    # clg = np.array(clframe[1::4])
    # clr = np.array(clframe[2::4])
    # cla = np.array(clframe[3::4])
    # clframe8bit = np.dstack((clb, clg, clr, cla)).reshape((kinect.colorHeight, kinect.colorWidth, 4))
    # clframe8bit2 = np.dstack((clb, clg, clr)).reshape((kinect.colorHeight, kinect.colorWidth, 3))

    #
    dframe = kinect.getDepthFrame()
    pcdarray = kinect.getPointCloud(dframe)
    print pcdarray
    break
    # df8 = np.uint8(dframe.clip(1, 4000) / 16.)
    # dframe8bit = np.dstack((df8, df8, df8)).reshape((kinect.depthHeight, kinect.depthWidth, 3))
    # cv2.imshow("Infrared", ifframe8bit)
    # cv2.imshow("Depth", dframe8bit)
    # cv2.imshow("Color", clframe8bit)
    # cv2.imshow("Color2", clframe8bit2)
    cv2.waitKey(40)

threadKinectCam.stop()