import rpyc
import cv2
import numpy as np
import pickle

if __name__ == "__main__":
    rkint = rpyc.connect("10.2.0.101", 18812)
    while True:
        print rkint.root.iskinectstarted()
        if rkint.root.iskinectstarted():
            break

    while True:
        # ifnpa = pickle.loads(rkint.root.getifarray())
        # clnpa = pickle.loads(rkint.root.getclarray())
        dnpa = pickle.loads(rkint.root.getdarray())

        # cv2.imshow("Infrared", ifnpa)
        cv2.imshow("Depth", dnpa)
        # cv2.imshow("Color", clnpa)
        cv2.waitKey(40)
