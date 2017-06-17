import cv2, platform
import numpy as np
import urllib
import os

cam2 = "http://localhost:8080/stream?topic=/rviz1/camera2/image&quality=40"
cam1 = "http://localhost:8080/stream?topic=/rviz1/camera1/image&quality=40"

stream2 = urllib.urlopen(cam2)
stream1 = urllib.urlopen(cam1)
bytes1=''
bytes2=''
frame2 = None
frame1 = None
sucessFrame1 = False
sucessFrame2 = False

buffersize = 17200
while True:
    sucessFrame1 = False
    sucessFrame2 = False
    # to read mjpeg frame -
    bytes2+=stream2.read(buffersize)
    a = bytes2.find('\xff\xd8')
    b = bytes2.find('\xff\xd9')

    if a!=-1 and b!=-1:
        jpg = bytes2[a:b+2]
        bytes2= bytes2[b+2:]
        frame2 = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
    # we now have frame stored in frame.

        #cv2.imshow('cam2',frame2)
        sucessFrame2 = True


    bytes1+=stream1.read(buffersize)
    a = bytes1.find('\xff\xd8')
    b = bytes1.find('\xff\xd9')

    if a!=-1 and b!=-1:
        print a, b
        jpg = bytes1[a:b+2]
        bytes1= bytes1[b+2:]
        frame1 = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
    # we now have frame stored in frame.

        sucessFrame1 = True

        #cv2.imshow('cam1',frame1)

    if(sucessFrame1 and sucessFrame2):
        
        h1, w1 = frame1.shape[:2]
        h2, w2 = frame2.shape[:2]

        #create empty matrix
        vis = np.zeros((max(h1, h2), w1+w2,3), np.uint8)

        #combine 2 images
        vis[:h1, :w1,:3] = frame1
        vis[:h2, w1:w1+w2,:3] = frame2

        cv2.imshow('oculus', vis)
    # Press 'q' to quit 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
            