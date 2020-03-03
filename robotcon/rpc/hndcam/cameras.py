import numpy as np
import os
import cv2
from panda3d.core import *
import utiltools.robotmath as rm
import cv2.aruco as aruco

class HndCam(object):

    def __init__(self, rgtcamid = [0,1], lftcamid = [2,3]):
        self.rgtcamid = rgtcamid
        self.lftcamid = lftcamid
        # pose of the camera in the local coordinate system of right hand
        # rgthndcamparr = os.path.join(this_dir, "hndcamparameters", "right_arm_camera_calib__.yaml")
        # this_dir, this_filename = os.path.split(__file__)
        # cv_file = cv2.FileStorage(rgthndcamparr, cv2.FILE_STORAGE_READ)
        # self.rgtmtx = cv_file.getNode("camera_matrix").mat()
        # self.rgtdist = cv_file.getNode("dist_coeff").mat()
        # self.T_CR_rgt = np.array([[-9.99442764e-01, -2.18179570e-02, 2.52614411e-02, -4.21361377e+00],
        #                          [2.07905605e-02, -9.98973601e-01, -4.02428632e-02, -2.62700248e+01],
        #                          [2.61135184e-02, -3.96952281e-02, 9.98870549e-01, -8.44215037e+01],
        #                          [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

    def getrc0img(self):
        camcap = cv2.VideoCapture(self.rgtcamid[0]+cv2.CAP_DSHOW)
        img = camcap.read()[1]
        camcap.release()

        return img

    def getrc1img(self):
        camcap = cv2.VideoCapture(1+cv2.CAP_DSHOW)
        img = camcap.read()[1]
        camcap.release()

        return img

    def getlc0img(self):
        camcap = cv2.VideoCapture(self.lftcamid[0] + cv2.CAP_DSHOW)
        img = camcap.read()[1]
        camcap.release()

        return img

    def getlc1img(self):
        camcap = cv2.VideoCapture(0 + cv2.CAP_DSHOW)
        img = camcap.read()[1]
        camcap.release()

        return img

    # def getObjPoseRgtCam(self, eepos, eerotmat, marker_id = 5):
    #     """
    #     get the homo pose of obj using rgtcam
    #
    #     :param eepos: endeffector pose
    #     :param eerotmat:  end effector rotmat
    #     :return:
    #
    #     author: weiwei, mmd
    #     date: 20180925
    #     """
    #     self.rgtcap0.release()
    #     self.rgtcap0 = cv2.VideoCapture(self.rgtcamid[0])
    #
    #     rgtarmrot = eerotmat
    #     rgtarmpos = eepos
    #     T_RGTW_a = np.eye(4)
    #     T_RGTW_a[:3, :3] = rgtarmrot
    #     T_RGTW_a[:3, 3] = rgtarmpos
    #     T_CW_a = np.dot(T_RGTW_a, self.T_CR_rgt)
    #     det_T_MC = np.eye(4)
    #
    #     T_MW_a = None
    #     for i in range(1000):
    #         detected = False
    #         ret, frame = self.rgtcap0.read()
    #         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #         aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    #         parameters = aruco.DetectorParameters_create()
    #         corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #         if np.all(ids != None):
    #             rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.02, self.rgtmtx, self.rgtdist)  # the second value is the marker size in meters
    #             for i in range(ids.shape[0]):
    #                 if ids[i] == marker_id:
    #                     det_rvec = rvec[i]  # check this if ERROR
    #                     det_tvec = 1000 * tvec[i]  # check this if ERROR
    #                     det_rmat, _ = cv2.Rodrigues(det_rvec)
    #                     det_T_MC[:3, :3] = det_rmat
    #                     det_T_MC[:3, 3] = det_tvec[0]
    #                     T_MW_a = np.dot(T_CW_a, det_T_MC)  # ought to be the left hand pose
    #                     detected = True
    #                     break
    #             if detected:
    #                 break
    #     final_T = T_MW_a
    #     return final_T

if __name__ == "__main__":
    import time
    hdc = HndCam()
    while True:
        # ifnpa = pickle.loads(rkint.root.getifarray())
        # clnpa = pickle.loads(rkint.root.getclarray())
        # dnpa = pickle.loads(rkint.root.getrcimg())
        dnpa = hdc.getlc0img()
        cv2.imshow("Depth", dnpa)
        cv2.waitKey(100)
        dnpa1 = hdc.getlc1img()
        cv2.imshow("Depth", dnpa1)
        cv2.waitKey(200)
        cv2.imwrite('test.jpg',dnpa)