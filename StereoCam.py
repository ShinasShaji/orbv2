import cv2
import time

class StereoCam:
    """Class to handle a camera pair as a synchronized stereo pair"""
    def __init__(self, leftID, rightID, width = 540, height = 360, fps=2):
        self.leftCam = cv2.VideoCapture(leftID)
        self.rightCam = cv2.VideoCapture(rightID)
        self.leftCam.set(3, width)
        self.leftCam.set(4, height)
        self.rightCam.set(3, width)
        self.rightCam.set(4, height)
        self.previousTime = time.time()
        self.frameTime = float(1/fps)

    def checkOpen(self):
        """Check if the cameras have initialized"""
        if self.leftCam.isOpened() and self.rightCam.isOpened():
            return True
        else:
            return False

    def grabFrame(self):
        """Grab a frame from both cameras at the same time"""
        for i in range(10):
            self.leftCam.grab()
            self.rightCam.grab()

    def retrieveFrame(self):
        """Retrieve the grabbed frames"""
        retLeft, self.leftImg = self.leftCam.retrieve()
        retRight, self.rightImg = self.rightCam.retrieve()
        if retLeft and retRight:
            return True
        else:
            return False
    
    def camPreview(self):
        """Preview the camera outputs"""
        if self.checkOpen():
            self.grabFrame()
            retVal = self.retrieveFrame()
            while retVal:
                if ((time.time()-self.previousTime)>=self.frameTime):
                    self.previousTime = time.time()
                    self.grabFrame()
                    retVal = self.retrieveFrame()
                    cv2.imshow('Left', self.leftImg)
                    cv2.imshow('Right', self.rightImg)
                    key = cv2.waitKey(20)
                    if key == 27: # Exit on ESC
                        break
            cv2.destroyAllWindows()
        else:
            print('Could not initialize camera pair')

if __name__=='__main__':
    Stereo = StereoCam(leftID=0, rightID=2, width=540, height=360, fps=2)
    Stereo.camPreview()