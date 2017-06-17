import cv2, platform
import numpy as np
import urllib
import os


class cameraBuffer():
    def __init__(self, url, buffersize = 18000):
        self.buffer = ''
        self.buffersize = 18000
        self.n = 1
        self.frame = None
        self.url = url
        self.stream = urllib.urlopen(url)
        self.has_frame = False

    def read_frame(self):
        if(len(buffer) > 5*buffersize):
            self.stream = urllib.urlopen(self.url)
            self.buffer = self.stream.read(buffersize)
        else:
            self.buffer+= self.stream.read(self.buffersize)

        a = buffer.find('\xff\xd8')
        b = buffer.find('\xff\xd9')

        if a!=-1 and b!=-1:
            jpg = bytes2[a:b+2]
            bytes2= bytes2[b+2:]
            n+=1
            self.buffersize = b - a 
            self.frame = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)

            #cv2.imshow('cam2',frame2)
            has_frame = True

    def get_frame(self):
        if(self.has_frame):
            return self.frame
        else:
            return None

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


buffersize = 17000

n = 1.0

while True:
    sucessFrame1 = False
    sucessFrame2 = False
    # to read mjpeg frame -
    if(len(bytes2) > 5*buffersize or len(bytes1) > 5*buffersize):
        stream1 = urllib.urlopen(cam1)
        bytes1 = stream1.read(buffersize)
        stream2 = urllib.urlopen(cam2)
        bytes2 =stream2.read(buffersize)
    else:
        bytes1+= stream1.read(buffersize)
        bytes2+= stream2.read(buffersize)

    a = bytes2.find('\xff\xd8')
    b = bytes2.find('\xff\xd9')

    if a!=-1 and b!=-1:
        jpg = bytes2[a:b+2]
        bytes2= bytes2[b+2:]
        n+=1
        buffersize = int(round(buffersize*(n-1)/n + (b - a)/n)) 
        frame2 = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
    # we now have frame stored in frame.

        #cv2.imshow('cam2',frame2)
        sucessFrame2 = True

    a = bytes1.find('\xff\xd8')
    b = bytes1.find('\xff\xd9')

    if a!=-1 and b!=-1:
        print a, b, len(bytes1), buffersize
        jpg = bytes1[a:b+2]
        bytes1= bytes1[b+2:]
        n+=1
        buffersize = int(round(buffersize*(n-1)/n + (b - a)/n))
        frame1 = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
    # we now have frame stored in frame.

        sucessFrame1 = True

        #cv2.imshow('cam1',frame1)

    if(sucessFrame1 and sucessFrame2):
        if(frame1 is not None and frame2 is not None):
            vis = np.append(frame1, frame2, 1)
            cv2.imshow('oculus', vis)
    # Press 'q' to quit 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
            