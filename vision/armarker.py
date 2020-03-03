import cv2
import numpy as np
import math

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

def arGenerator(filename):
    fileName = filename
    generator = aruco.drawMarker(dictionary, 0, 100)
    cv2.imwrite(fileName, generator)

    img = cv2.imread(fileName)
    cv2.imshow('ArMaker',img)
    cv2.waitKey(0)

def arReader():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        Height, Width = frame.shape[:2]
        img = cv2.resize(frame,(int(Width),int(Height)))
        corners, ids, rejectedImgPoints = aruco.detectMarkers(img, dictionary)
        aruco.estimatePoseSingleMarkers(corners, size, mtx, dist)
        # print np.mean(np.array(corners[0][0]), axis = 0)
        aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
        cv2.imshow('drawDetectedMarkers', img)
        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()

def arReaderCoord():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    Height, Width = frame.shape[:2]
    img = cv2.resize(frame,(int(Width),int(Height)))
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, dictionary)
    print corners
    dv = np.array(corners[0][0][0])-np.array(corners[0][0][1])
    dv2 = np.array(corners[0][0][1])-np.array(corners[0][0][2])
    dv3 = np.array(corners[0][0][2])-np.array(corners[0][0][3])
    dv4 = np.array(corners[0][0][3])-np.array(corners[0][0][0])
    print dv0
    print math.sqrt(dv[0]**2+dv[1]**2)
    print dv2
    print math.sqrt(dv2[0]**2+dv2[1]**2)
    print dv3
    print math.sqrt(dv3[0]**2+dv3[1]**2)
    print dv4
    print math.sqrt(dv4[0]**2+dv4[1]**2)
    return np.mean(np.array(corners[0][0]), axis = 0)


if __name__ == "__main__":
    #
    # arGenerator("ar.png")
    # print arReaderCoord()
    arReader()