import cv2
import time
# for i in range(5,1000):
#
#     cap=cv2.VideoCapture(i+cv2.CAP_DSHOW)
#
#     try:
#         ret,frame = cap.read()
#         print i
#         cv2.imshow("lala.png",frame)
#         cv2.waitKey(0)
#     except:
#         continue

# cap=cv2.VideoCapture(1+cv2.CAP_DSHOW)
# #
# #     try:
# ret,frame = cap.read()
#
# cv2.imshow("camera.png",frame)
# cv2.waitKey(0)

cap1=cv2.VideoCapture(1+cv2.CAP_DSHOW)
cap2=cv2.VideoCapture(2+cv2.CAP_DSHOW)
cap3=cv2.VideoCapture(3+cv2.CAP_DSHOW)
cap0=cv2.VideoCapture(0+cv2.CAP_DSHOW)
#
#     try:
while True:

    ret,frame = cap0.read()
    cv2.imshow("cap0",frame)
    ret,frame = cap1.read()
    cv2.imshow("cap1",frame)
    ret,frame = cap2.read()
    cv2.imshow("cap2",frame)
    ret,frame = cap3.read()
    cv2.imshow("cap3",frame)

    cv2.waitKey(100)