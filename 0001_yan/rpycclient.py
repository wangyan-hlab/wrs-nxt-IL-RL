import rpyc
import cv2
import numpy as np
import pickle

if __name__ == "__main__":
    # rkint = rpyc.connect("10.2.0.60", 18300)
    # while True:
    #     print rkint.root.iskinectstarted()
    #     if rkint.root.iskinectstarted():
    #         break

    import time
    rkint1 = rpyc.connect("10.2.0.60", 18300)

    while True:
        # ifnpa = pickle.loads(rkint.root.getifarray())
        # clnpa = pickle.loads(rkint.root.getclarray())
        # dnpa = pickle.loads(rkint.root.getrcimg())
        # dnpa = pickle.loads(rkint1.root.getrc1img())
        # cv2.imshow("Depth", dnpa)
        # cv2.waitKey(100)
        dnpa = pickle.loads(rkint1.root.getrc1img())
        cv2.imshow("Depth", dnpa)
        cv2.waitKey(100)
        # dnpa = pickle.loads(rkint1.root.getlc0img())
        # cv2.imshow("Depth", dnpa)
        # cv2.waitKey(100)
        # dnpa = pickle.loads(rkint1.root.getlc1img())
        # cv2.imshow("Depth", dnpa)
        # cv2.waitKey(100)

        # cv2.imshow("Infrared", ifnpa)
