import cv2
import numpy


class StereoMatcher:
    """Class to handle StereoSGBM and StereoBM stereo matchers"""
    def __init__(self, matcher="SGBM", vertical=True, \
                                                createRightMatcher=False):
        assert matcher in ["SGBM", "BM"], "Invalid matcher selection"

        # Flags
        self.matcher = matcher
        self.vertical = vertical
        self.hasRightMatcher = createRightMatcher
        self.hasWLSFilter = False

        # Parameters
        self.minDisparity = 0
        self.kernel = numpy.ones((3,3),numpy.uint8)

        if self.matcher=="SGBM":
            self.createSGBM()

        else:
            self.createBM()

        
    def createSGBM(self):
        """Create StereoSGBM (Semi Global Block Matcher) object"""
        self.blockSize = 3
        self.minDisparity = 2
        self.numDisparities = 32 - self.minDisparity
        self.penalty1 = 8*3*self.blockSize**2
        self.penalty2 = 32*3*self.blockSize**2
        self.uniquenessRatio = 10
        self.speckleWindowSize = 50
        self.speckleRange = 2
        self.disp12MaxDiff = 5

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

    
    def createBM(self):
        """Create StereoBM (Block Matcher) object"""
        # Parameters
        self.numDisparities = 96
        self.blockSize = 21

        # Left
        self.stereoL = cv2.StereoBM_create(\
                                    numDisparities=self.numDisparities, \
                                    blockSize=self.blockSize)

        # Right
        if self.hasRightMatcher:
            self.stereoR = cv2.ximgproc.createRightMatcher(self.stereoL)

    
    def computeDisparity(self, grayImageL, grayImageR):
        """Compute left and right (if enabled) disparity"""
        if self.vertical:
            self.grayImageL = grayImageL

            grayImageL = cv2.rotate(grayImageL, \
                                        cv2.ROTATE_90_COUNTERCLOCKWISE)
            grayImageR = cv2.rotate(grayImageR, \
                                        cv2.ROTATE_90_COUNTERCLOCKWISE)

        self.disparityMapL = self.stereoL.compute(\
                                        grayImageL, grayImageR)
        if self.vertical:
            self.disparityMapL = cv2.rotate(self.disparityMapL, \
                                        cv2.ROTATE_90_CLOCKWISE)
        
        if self.hasRightMatcher:
            self.disparityMapR = self.stereoR.compute(\
                                        grayImageR, grayImageL)
            if self.vertical:
                self.disparityMapR = cv2.rotate(self.disparityMapR, \
                                        cv2.ROTATE_90_CLOCKWISE)

    
    def clampDisparity(self):
        """Sets 0 for most distant object that can be detected"""
        self.disparityMapL = ((self.disparityMapL.astype(numpy.float32)\
                            /16)-self.minDisparity)/self.numDisparities

        if self.hasRightMatcher:
            self.disparityMapR = ((self.disparityMapR.astype(numpy.float32)\
                            /16)-self.minDisparity)/self.numDisparities

    
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

        self.hasWLSFilter = True

    
    def checkWLSFilter(self):
        """Check if WLS filter has been initialized"""
        if self.hasWLSFilter:
            return True

        else:
            print("WLS disparity filter not initialized")
            return False

    
    def checkRightMatcher(self):
        """Check if right matcher has been initialized"""
        if self.hasRightMatcher:
            return True

        else:
            print("Right stereo matcher not initialized")
            return False

    
    def applyWLSFilterDisparity(self):
        """Apply created WLS disparity filter on disparity maps"""
        if self.checkWLSFilter() and self.checkRightMatcher():
            self.disparityMapL = self.wlsFilter.filter(self.disparityMapL, \
                self.grayImageL, disparity_map_right=self.disparityMapR)
        
        else:
            print("Could not apply WLS filter")



if __name__=="__main__":
    print("Import to use")