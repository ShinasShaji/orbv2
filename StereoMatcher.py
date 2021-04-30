import math

import cv2
import matplotlib.pyplot as plt
import numpy

from helperScripts import jsonHelper
from helperScripts.Keys import Keys


class StereoMatcher:
    """Class to handle StereoSGBM and StereoBM stereo matchers"""
    def __init__(self, matcher="SGBM", vertical=True, \
                                                createRightMatcher=False):
        assert matcher in ["SGBM", "BM"], "Invalid matcher selection"

        # Flags
        self.matcher = matcher
        self.vertical = vertical
        self.hasRightMatcher = createRightMatcher
        self.parametersLoaded = False

        # Initializing matcher
        self.loadParameters()
        self.createMatcher()


    def saveParameters(self, path="data/"):
        """Export parameters of matcher as a json"""
        if self.matcher=="SGBM":
            path = "".join([path, "parametersSGBM.json"])
        
        else:
            path = "".join([path, "parametersBM.json"])

        # Crafting dictionary to hold parameters
        parameterDict = {
            "blockSize":self.blockSize,
            "minDisparity":self.minDisparity,
            "maxDisparity":self.maxDisparity,
            "uniquenessRatio":self.uniquenessRatio,
            "speckleWindowSize":self.speckleWindowSize,
            "speckleRange":self.speckleRange,
            "disp12MaxDiff":self.disp12MaxDiff,
            "kernelOrder":self.kernelOrder
        }

        jsonHelper.dictToJson(parameterDict, path)

    
    def loadParametersFromJson(self, path="data/"):
        """Load parameters of matcher currently in use"""
        if self.matcher=="SGBM":
            path = "".join([path, "parametersSGBM.json"])
        
        else:
            path = "".join([path, "parametersBM.json"])

        parameterDict = jsonHelper.jsonToDict(path)

        # Loading to class variables
        self.blockSize = parameterDict["blockSize"]
        self.minDisparity = parameterDict["minDisparity"]
        self.maxDisparity = parameterDict["maxDisparity"]
        self.numDisparities = self.maxDisparity - self.minDisparity
        self.penalty1 = 8*3*self.blockSize**2
        self.penalty2 = 32*3*self.blockSize**2
        self.uniquenessRatio = parameterDict["uniquenessRatio"]
        self.speckleWindowSize = parameterDict["speckleWindowSize"]
        self.speckleRange = parameterDict["speckleRange"]
        self.disp12MaxDiff = parameterDict["disp12MaxDiff"]
        self.kernelOrder = parameterDict["kernelOrder"]
        self.kernel = numpy.ones((self.kernelOrder,\
                                self.kernelOrder),numpy.uint8)

        self.parametersLoaded = True

    
    def loadDefaultParameters(self):
        """Load default matcher parameters"""
        # Loading default parameters to class variables
        self.blockSize = 3
        self.minDisparity = 2
        self.maxDisparity = 34
        self.numDisparities = self.maxDisparity - self.minDisparity
        self.penalty1 = 8*3*self.blockSize**2
        self.penalty2 = 32*3*self.blockSize**2
        self.uniquenessRatio = 10
        self.speckleWindowSize = 50
        self.speckleRange = 2
        self.disp12MaxDiff = 5
        self.kernelOrder = 3
        self.kernel = numpy.ones((self.kernelOrder,\
                                self.kernelOrder),numpy.uint8)

        self.parametersLoaded = True

    
    def loadParameters(self):
        """Loads matcher parameters from json; if unavailable loads default
        parameters"""
        if not self.parametersLoaded:
            try:
                self.loadParametersFromJson()
            except:
                self.loadDefaultParameters()

        
    def createSGBM(self):
        """Create StereoSGBM (Semi Global Block Matcher) object"""
        # Creating StereoSGBM matchers
        # Left
        self.stereoL = cv2.StereoSGBM_create(minDisparity=self.minDisparity, \
            numDisparities=self.numDisparities, blockSize=self.blockSize, \
            uniquenessRatio=self.uniquenessRatio, \
            speckleRange=self.speckleRange, \
            speckleWindowSize=self.speckleWindowSize, \
            disp12MaxDiff=self.disp12MaxDiff, \
            P1=self.penalty1, P2=self.penalty2)

        # Right
        if self.hasRightMatcher:
            self.stereoR = cv2.ximgproc.createRightMatcher(self.stereoL)


    def createMatcher(self):
        """Creates stereo object based on matcher flag"""
        if self.matcher=="SGBM":
            self.createSGBM()

        else:
            self.createBM()

    
    def createBM(self):
        """Create StereoBM (Block Matcher) object"""
        # Left
        self.stereoL = cv2.StereoBM_create(\
                                    numDisparities=self.numDisparities, \
                                    blockSize=self.blockSize)

        # Right
        if self.hasRightMatcher:
            self.stereoR = cv2.ximgproc.createRightMatcher(self.stereoL)

    
    def computeDisparity(self, grayImageL, grayImageR):
        """Compute left and right (if enabled) disparity"""
        self.disparityMapL = self.stereoL.compute(\
                                        grayImageL, grayImageR)
        
        if self.hasRightMatcher:
            self.disparityMapR = self.stereoR.compute(\
                                        grayImageR, grayImageL)

    
    def clampDisparity(self):
        """Sets 0 for most distant object that can be detected"""
        self.disparityMapL = ((self.disparityMapL.astype(numpy.float32)\
                            /16))

        if self.hasRightMatcher:
            self.disparityMapR = ((self.disparityMapR.astype(numpy.float32)\
                            /16))

    
    def applyClosingFilter(self):
        """Applies a closing filter to the disparity map"""
        self.disparityMapL = cv2.morphologyEx(\
                    self.disparityMapL, cv2.MORPH_CLOSE, self.kernel)

        if self.hasRightMatcher:
            self.disparityMapR = cv2.morphologyEx(\
                    self.disparityMapR, cv2.MORPH_CLOSE, self.kernel)

    
    def createDisparityWLSFilter(self):
        """Create disparity map filter based on Weighted Least Squares 
        filter"""
        # Parameters
        self.lmbda = 80000
        self.sigma = 1.8

        # Creating filter
        self.wlsFilter = cv2.ximgproc.createDisparityWLSFilter(\
                                            matcher_left=self.stereoL)
        
        # Setting parameters
        self.wlsFilter.setLambda(self.lmbda)
        self.wlsFilter.setSigmaColor(self.sigma)

    
    def applyWLSFilterDisparity(self):
        """Apply created WLS disparity filter on disparity maps"""
        self.disparityMapL = self.wlsFilter.filter(self.disparityMapL, \
                self.grayImageL, disparity_map_right=self.disparityMapR)


    def referenceDispToDepthMatrix(self, dispToDepthMatrix):
        """Create class reference to the disparity to depth matrix, Q"""
        self.dispToDepthMatrix = dispToDepthMatrix

    
    def generatePointCloud(self):
        """Generate point cloud from dispariy map"""

        # Guesstimate parameters
        h, w = 360, 720
        f = 0.8*w                      # guess for focal length
        Q = numpy.float32(\
                   [[1, 0, 0, -0.5*w],
                    [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                    [0, 0, 0,     -f], # so that y-axis looks up
                    [0, 0, 1,      0]])

        # Temporary implementation to view point clouds
        points = cv2.reprojectImageTo3D(self.disparityMapL, \
            self.dispToDepthMatrix)
        pointCloud = points.reshape(\
                    (points.shape[0]*points.shape[1], 3))[0::20]
        pointCloud = pointCloud[pointCloud[:, 2]>0]  

        fig = plt.figure()
        ax = fig.add_subplot(111, projection = "3d")
        ax.scatter(pointCloud[:,0], pointCloud[:,1], pointCloud[:,2], s=2)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")

        plt.show(block=True)

    
    def generateDepthMap(self):
        """Generate depth map from disparity"""

        ### To do: Implement

        pass


    def captureImages(self, imageProcessor, path="captures/testImages"):
        """Call the image capture method of the passed ImageProcessing
        object"""
        imageProcessor.captureImages(path)

    
    def tuneParameters(self, imageProcessor):
        """Tune stereo matcher parameters"""
        key = cv2.waitKey(20)

        if key == Keys.esc: # Exit on ESC
            print("Exiting disparity preview")
            return False

        elif key == Keys.p: # Generate point cloud on space
            self.generatePointCloud()

        elif key == Keys.space: # Capture images on space
            self.captureImages(imageProcessor)

        elif key == Keys.b: # blocksize on b
            print("blockSize: {}".format(self.blockSize))
            key = cv2.waitKey(0)

            if key == Keys.up: # Increment on up arrow
                self.blockSize+=2
            elif key == Keys.down: # Decrement on down arrow
                self.blockSize-=2
            # Verify value is valid
            if self.blockSize%2==0:
                self.blockSize+=1
            if self.blockSize<1:
                self.blockSize=1

            print("blockSize --> {}".format(self.blockSize))

        elif key == Keys.k: # kernelOrder on k
            print("kernelOrder: {}".format(self.kernelOrder))
            key = cv2.waitKey(0)

            if key == Keys.up:
                self.kernelOrder+=2
            elif key == Keys.down:
                self.kernelOrder-=2
            
            if self.kernelOrder%2==0:
                self.kernelOrder+=1
            if self.kernelOrder<3:
                self.kernelOrder=3

            print("kernelOrder --> {}".format(self.kernelOrder))

        elif key == Keys.m: # minDisparity on m
            print("minDisparity: {}; numDisparities: {}".format(\
                                    self.minDisparity, self.numDisparities))
            key = cv2.waitKey(0)

            if key == Keys.up:
                self.minDisparity+=1
                self.maxDisparity+=1
            elif key == Keys.down:
                self.minDisparity-=1
                self.maxDisparity-=1
            
            self.numDisparities = self.maxDisparity - self.minDisparity

            print("minDisparity --> {}; numDisparities --> {}".format(\
                                    self.minDisparity, self.numDisparities))

        elif key == Keys.n: # numDisparities on n
            print("maxDisparity: {}; numDisparities: {}".format(\
                                self.maxDisparity, self.numDisparities))
            key = cv2.waitKey(0)

            numDisparities = self.maxDisparity - self.minDisparity
            power2 = int(math.log(numDisparities, 2))

            if key == Keys.up:
                power2+=1
            elif key == Keys.down:
                power2-=1
                if power2<1:
                    power2=1

            numDisparities = 2**power2
            maxDisparity = numDisparities + self.minDisparity
            if numDisparities>self.minDisparity:
                self.numDisparities = numDisparities
                self.maxDisparity = maxDisparity

            print("maxDisparity --> {}; numDisparities --> {}".format(\
                                self.maxDisparity, self.numDisparities))

        elif key == Keys.u: # uniquenessRatio on u
            print("uniquenessRatio: {}".format(self.uniquenessRatio))
            key = cv2.waitKey(0)

            if key == Keys.up:
                self.uniquenessRatio+=1
            elif key == Keys.down:
                self.uniquenessRatio-=1

            if self.uniquenessRatio<0:
                self.uniquenessRatio=0
            elif self.uniquenessRatio>100:
                self.uniquenessRatio=100

            print("uniquenessRatio --> {}".format(self.uniquenessRatio))

        elif key == Keys.w: # speckleWindowSize on w
            print("speckleWindowSize: {}".format(self.speckleWindowSize))
            key = cv2.waitKey(0)

            if key == Keys.up:
                self.speckleWindowSize+=1
            elif key == Keys.down:
                self.speckleWindowSize-=1

            if self.speckleWindowSize<0:
                self.speckleWindowSize=0

            print("speckleWindowSize --> {}".format(self.speckleWindowSize))

        elif key == Keys.r: # speckleRange on r
            print("speckleRange: {}".format(self.speckleRange))
            key = cv2.waitKey(0)

            if key == Keys.up:
                self.speckleRange+=1
            elif key == Keys.down:
                self.speckleRange-=1

            if self.speckleRange<0:
                self.speckleRange=0

            print("speckleRange --> {}".format(self.speckleRange))

        elif key == Keys.d: # disp12MaxDiff on d
            print("disp12MaxDiff: {}".format(self.disp12MaxDiff))
            key = cv2.waitKey(0)

            if key == Keys.up:
                self.disp12MaxDiff+=1
            elif key == Keys.down:
                self.disp12MaxDiff-=1

            if self.disp12MaxDiff<-1:
                self.disp12MaxDiff=-1

            print("disp12MaxDiff --> {}".format(self.disp12MaxDiff))

        elif key == Keys.l:
            self.loadParameters()
            print(" ".join(["Reloaded", self.matcher, "parameters"]))

        elif key == Keys.s:
            self.saveParameters()
            print(" ".join(["Saved", self.matcher, "parameters"]))
        
        return True



if __name__=="__main__":
    print("Import to use")
