import datetime
import math

import cv2
import matplotlib.pyplot as plt
import numpy as np

from helperScripts import jsonHelper
from helperScripts.Keys import Keys


class StereoMatcher:
    """Class to handle StereoSGBM and StereoBM stereo matchers"""
    def __init__(self, imageProcessor, matcher="SGBM", vertical=True, \
                                                createRightMatcher=False):
        assert matcher in ["SGBM", "BM"], "Invalid matcher selection"

        # Flags
        self.matcher = matcher
        self.vertical = vertical
        self.hasRightMatcher = createRightMatcher
        self.parametersLoaded = False

        # Object references
        self.imageProcessor = imageProcessor

        # Initializing matcher
        self.loadParameters()
        self.setBaseline(32)
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
        self.kernel = np.ones((self.kernelOrder,\
                                self.kernelOrder),np.uint8)

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
        self.kernel = np.ones((self.kernelOrder,\
                                self.kernelOrder),np.uint8)

        self.parametersLoaded = True

    
    def loadParameters(self):
        """Loads matcher parameters from json; if unavailable loads default
        parameters"""
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
        self.grayImageL = grayImageL
        self.disparityMapL = self.stereoL.compute(\
                                        grayImageL, grayImageR)
        
        if self.hasRightMatcher:
            self.disparityMapR = self.stereoR.compute(\
                                        grayImageR, grayImageL)

    
    def clampDisparity(self):
        """Sets 0 for most distant object that can be detected"""
        self.disparityMapL = ((self.disparityMapL.astype(np.float32)\
                            /16))

        if self.hasRightMatcher:
            self.disparityMapR = ((self.disparityMapR.astype(np.float32)\
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


    def setBaseline(self, baseline):
        """Use to set the baseline for the stereo rig"""
        self.baseline = baseline

    
    def generateDepthMap(self):
        """Generate depth map from disparity"""
        # Get focal length from projection matrix
        focalLength = self.imageProcessor.projectionMatrixL[0][0]

        self.disparityMapL[self.disparityMapL==0] = 0.9
        self.disparityMapL[self.disparityMapL==-1] = 0.9

        self.depthMap = np.empty_like(self.disparityMapL)
        self.depthMap = (focalLength*self.baseline)/self.disparityMapL[:]

    
    def saveDepthMap(self, path="testImages/voxelTestImages/", \
                                                    imageFormat=".png"):
        """Save the current depth map"""
        imageName = "".join(["topDepth_{}".format(\
                        self.imageProcessor.timeString), imageFormat])

        cv2.imwrite("".join([path, imageName]), \
                                    self.depthMap.astype(np.uint16))
        print("Saved {} to {}".format(imageName, path))

    
    def writePly(self, points, imageL):
        """Export current point cloud as a .ply"""
        plyHeader = "\n".join([\
                    "format ascii 1.0",
                    "element vertex %(vert_num)d",
                    "property float x",
                    "property float y",
                    "property float z",
                    "property uchar red",
                    "property uchar green",
                    "property uchar blue",
                    "end_header"])

        mask = self.disparityMapL > self.disparityMapL.min()
        vertices = points[mask]
        colors = cv2.cvtColor(imageL, cv2.COLOR_BGR2RGB)
        colors = colors[mask]
        vertices = vertices.reshape(-1, 3)
        colors = colors.reshape(-1, 3)
        vertices = np.hstack([vertices, colors])

        timeString = datetime.datetime.now().strftime("%d%m%y%H%M%S")
        fileName = "".join(["pointCloud_", timeString, ".ply"])

        with open(fileName, 'wb') as plyFile:
            plyFile.write((plyHeader % dict(vert_num=len(vertices)))\
                                                        .encode('utf-8'))
            np.savetxt(plyFile, vertices, fmt='%f %f %f %d %d %d ')

    
    def referenceVoxelGrid(self, voxelGrid):
        """Creates class references to passed VoxelGrid"""
        self.voxelGrid = voxelGrid


    def captureImages(self, path="testImages/voxelTestImages/"):
        """Call the image capture method of the referenced ImageProcessor"""
        self.imageProcessor.captureImages(path=path)

    
    def viewVoxelGrid(self):
        """Call the voxel grid viewer method of the referenced VoxelGrid"""
        self.voxelGrid.viewVoxelGrid()

    
    def resetVoxelGrid(self):
        """Call the voxel grid reset method of the referenced VoxelGrid"""
        self.voxelGrid.resetVoxelGrid()

    
    def tuneParameters(self):
        """Tune stereo matcher parameters"""
        key = cv2.waitKey(20)

        if key == Keys.esc: # Exit on ESC
            print("Exiting disparity preview")
            return False

        elif key == Keys.p: # Generate point cloud on p
            self.viewVoxelGrid()

        elif key == Keys.o: # Reset voxel grid on o
            self.resetVoxelGrid()

        elif key == Keys.space: # Capture images on space
            self.captureImages()
            self.generateDepthMap()
            self.saveDepthMap()

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
