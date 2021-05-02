import cv2
import numpy as np

from helperScripts.TimeKeeper import TimeKeeper


class VoxelGrid:
    """Class to process and handle voxelized representation of 
    pointclouds"""
    def __init__(self, voxelSize, stereoMatcher):
        # Paramaters
        self.voxelSize = voxelSize

        # StereoMatcher
        self.stereoMatcher = stereoMatcher
        self.stereoMatcher.referenceVoxelGrid(self)
        
        # Debug
        self.verbose = True
        if self.verbose:
            self.timeKeeper = TimeKeeper()


    def generatePointCloud(self):
        """Generate point cloud from disparity map and disparity-to-depth
        mapping matrix, Q"""
        if self.verbose:
            self.timeKeeper.startPerfCounter()

        points = cv2.reprojectImageTo3D(\
                            self.stereoMatcher.disparityMapL, \
                            self.stereoMatcher.dispToDepthMatrix)

        # Reshaping to a list of 3D coordinates
        self.pointCloud = points.reshape(\
                        (points.shape[0]*points.shape[1],3))[0::16]\
                                                .astype(np.int16)

        if self.verbose:
            print("".join(["Points in unfiltered pointcloud: {}; ",\
                    "completed in {:.5f} sec"]).format(\
                    self.pointCloud.shape[0], \
                    self.timeKeeper.returnPerfCounter()))

    
    def filterPointCloud(self):
        """Filter extreme points from the generate point cloud"""
        if self.verbose:
            self.timeKeeper.startPerfCounter()

        # Filtering x values
        self.pointCloud = self.pointCloud[np.logical_and(\
            self.pointCloud[:, 0]>self.pointCloud[:, 0].min(), \
            self.pointCloud[:, 0]<self.pointCloud[:, 0].max())]
        # Filtering y values
        self.pointCloud = self.pointCloud[np.logical_and(\
            self.pointCloud[:, 1]>self.pointCloud[:, 1].min(), \
            self.pointCloud[:, 1]<self.pointCloud[:, 1].max())]
        # Filtering z values
        self.pointCloud = self.pointCloud[np.logical_and(\
            self.pointCloud[:, 2]>self.pointCloud[:, 2].min(), \
            self.pointCloud[:, 2]<self.pointCloud[:, 2].max())]

        if self.verbose:
            print("".join(["Points in filtered pointcloud: {}; ",\
                    "completed in {:.5f} sec"]).format(\
                    self.pointCloud.shape[0], \
                    self.timeKeeper.returnPerfCounter()))

    
    def redefinePointCloudCoordinate(self):
        """Rotate the point cloud so that camera faces +y, with z
        vertical"""
        rotationMatrix = np.array([ [ 0,  0, -1],
                                    [-1,  0,  0],
                                    [ 0,  1,  0] ])
        self.pointCloud = np.dot(self.pointCloud[:], rotationMatrix)
        