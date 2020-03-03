import rpyc
from rpyc.utils.server import ThreadedServer
import time
import kntv2
import hndcam
import pickle
import threading
import numpy as np
import serial
import os
import PySpin

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
            self.__connected = True

        # Retrieve singleton reference to system object
        system = PySpin.System.GetInstance()
        # Retrieve list of cameras from the system
        cam_list = system.GetCameras()
        self.__connected = False
        num_cameras = cam_list.GetSize()
        print('Number of cameras detected: %d' % num_cameras)

        # Finish if there are no cameras
        if num_cameras == 0:
            # Clear camera list before releasing system
            cam_list.Clear()

            # Release system instance
            system.ReleaseInstance()

            print('Not enough cameras!')
            return False

    def exposed_getCamImg(self):

        self.__cam = cam_list[0]

        # Retrieve TL device nodemap
        nodemap_tldevice = self.__cam.GetTLDeviceNodeMap()

        # Initialize camera
        self.__cam.Init()
        # Retrieve GenICam nodemap
        self.__nodemap = self.__cam.GetNodeMap()

        node_acquisition_mode = PySpin.CEnumerationPtr(self.__nodemap.GetNode('AcquisitionMode'))

        if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
            print('Unable to set acquisition mode to continuous (enum retrieval). Aborting...')
            return False

        # Retrieve entry node from enumeration node
        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
        if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(
                node_acquisition_mode_continuous):
            print('Unable to set acquisition mode to continuous (entry retrieval). Aborting...')
            return False

        # Retrieve integer value from entry node
        acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

        # Set integer value from entry node as new value of enumeration node
        node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

        print('Acquisition mode set to continuous...')

        #  Begin acquiring images
        #
        #  *** NOTES ***
        #  What happens when the camera begins acquiring images depends on the
        #  acquisition mode. Single frame captures only a single image, multi
        #  frame catures a set number of images, and continuous captures a
        #  continuous stream of images. Because the example calls for the
        #  retrieval of 10 images, continuous mode has been set.
        #
        #  *** LATER ***
        #  Image acquisition must be ended when no more images are needed.
        self.__cam.BeginAcquisition()

        try:

            #  Retrieve next received image
            #
            #  *** NOTES ***
            #  Capturing an image houses images on the camera buffer. Trying
            #  to capture an image that does not exist will hang the camera.
            #
            #  *** LATER ***
            #  Once an image from the buffer is saved and/or no longer
            #  needed, the image must be released in order to keep the
            #  buffer from filling up.
            image_result = self.__cam.GetNextImage()

            #  Ensure image completion
            #
            #  *** NOTES ***
            #  Images can easily be checked for completion. This should be
            #  done whenever a complete image is expected or required.
            #  Further, check image status for a little more insight into
            #  why an image is incomplete.
            if image_result.IsIncomplete():
                print('Image incomplete with image status %d ...' % image_result.GetImageStatus())

            else:

                #  Print image information; height and width recorded in pixels
                #
                #  *** NOTES ***
                #  Images have quite a bit of available metadata including
                #  things such as CRC, image status, and offset values, to
                #  name a few.
                width = image_result.GetWidth()
                height = image_result.GetHeight()
                print('Grabbed Image width = %d, height = %d' % (width, height))

                #  Convert image to mono 8
                #
                #  *** NOTES ***
                #  Images can be converted between pixel formats by using
                #  the appropriate enumeration value. Unlike the original
                #  image, the converted one does not need to be released as
                #  it does not affect the camera buffer.
                #
                #  When converting images, color processing algorithm is an
                #  optional parameter.
                # image_converted = image_result.Convert(PySpin.PixelFormat_RGB8Packed, PySpin.HQ_LINEAR)
                # image_converted = image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)
                #  convert imagePtr to imageArray
                image_result.Save("ttest.jpg")
            image = image_result.GetNDArray()
            print(image.shape)

            image_result.Release()
            self.__cam.EndAcquisition()

            # Deinitialize camera
            self.__cam.DeInit()
            # Clear camera list before releasing system

            del self.__cam

            cam_list.Clear()
            # Release system instance
            system.ReleaseInstance()
            # pickle the imageArray and return it
            return pickle.dumps(image)

            # return image

        except PySpin.SpinnakerException as ex:
            print('Error: %s' % ex)
            return False

if __name__ == "__main__":
    server = ThreadedServer(MyService, hostname="10.2.0.60", port=18300)
    server.start()
