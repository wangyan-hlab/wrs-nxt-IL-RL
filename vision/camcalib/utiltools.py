import os
import cv2
import time
import glob
import numpy as np

def genworldpoints(nrow, ncolumn, markersize):
    """
    get the realworld positions of the chessmarkers
    z is 0
    upperleft is the origin

    :param nrow:
    :param ncolumn:
    :param markersize:
    :return:

    author: weiwei
    date: 20190420
    """

    worldpoints = np.zeros((nrow*ncolumn, 3), np.float32)
    worldpoints[:, :2] = np.mgrid[:nrow, :ncolumn].T.reshape(-1, 2)*markersize
    return worldpoints

def captureimgbytime(camid=0, savepath="./"):
    """

    :param camid:
    :param savepath:
    :return:
    """

    camera = cv2.VideoCapture(camid)
    imgid = 0
    while True:
        return_value, image = camera.read()
        cv2.imwrite(savepath+'opencv' + str(imgid) + '.png', image)
        cv2.imshow('The captured image...', image)
        k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
        imgid+=1
        time.sleep(1)

def captureimgbytimemulticam(camids=[0,2,4], savepath="./"):
    """

    :param camid:
    :param savepath:
    :return:
    """

    savepaths = []
    cameras = []
    windows = []
    for camid in camids:
        foldername = "./camimgs"+str(camid)+"/"
        if not os.path.exists(foldername):
            os.mkdir(foldername)
        else:
            files = glob.glob(foldername+"*")
            for f in files:
                os.remove(f)
        savepaths.append(foldername)
        cameras.append(cv2.VideoCapture(camid))
        windows.append('cam'+str(camid))
    imgid = 0
    while True:
        returnvalues = []
        images = []
        for id, camera in enumerate(cameras):
            returnvalue, image = camera.read()
            images.append(image)
            returnvalues.append(returnvalue)
        for id, camera in enumerate(cameras):
            cv2.imwrite(savepaths[id]+'opencv' + str(imgid) + '.png', images[id])
            cv2.imshow(windows[id], images[id])
            cv2.moveWindow(windows[id], 450+id*700, 200)
            k = cv2.waitKey(1)
            if k%256 == 27:
                # ESC pressed
                print("Escape hit, closing...")
                return
        imgid+=1
        time.sleep(.5)

def captureimgbychessdetect(nrow, ncolumn, camid=0, savepath="./"):
    """

    :param camid:
    :param nrow: nrow of checker board
    :param ncolumn: ncolumn of checker board
    :param savepath:
    :return:
    """

    camera = cv2.VideoCapture(camid)
    imgid = 0
    lastcaptime = time.time()
    while True:
        newcaptime = time.time()
        return_value, image = camera.read()
        cv2.imshow('The captured image...', image)
        k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
        ret, corners = cv2.findChessboardCorners(image, (ncolumn, nrow))
        if ret and (len(corners) == nrow*ncolumn) and (newcaptime-lastcaptime > 1):
            cv2.imwrite(savepath+'opencv' + str(imgid) + '.png', image)
            print(str(imgid+1)+" images have been captured!")
            lastcaptime = newcaptime
            imgid+=1

if __name__=='__main__':
    # makechessboard(7,5,markersize=40)
    # worldpoints = genworldpoints(8,6, 25)
    # print(worldpoints)
    # captureimgbydetect(8,6)
    captureimgbytimemulticam()

