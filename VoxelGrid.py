import threading

import cv2
import matplotlib.pyplot as plt
import numpy as np

from helperScripts.TimeKeeper import TimeKeeper


class VoxelGrid:
    """Class to process and handle voxelized representation of 
    pointclouds"""
    def __init__(self, stereoMatcher, imageProcessor, pointSubsample=20, \
                voxelSize=100, occupancyThreshold=10, voxelStopFraction=10):

        # Paramaters
        # Size of each voxel in mm
        self.voxelSize = voxelSize
        # Fraction of raw points taken
        self.pointSubsample = int(pointSubsample)
        # Stop voxelizing when less than this fraction of points remain
        self.voxelStopFraction = voxelStopFraction 
        # Minimum number of points after a voxel is marked occupied
        self.occupancyThreshold = occupancyThreshold

        # Voxel grid
        self.voxelGrid = None

        # Object references
        self.stereoMatcher = stereoMatcher
        self.imageProcessor = imageProcessor
        
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
                            self.imageProcessor.dispToDepthMatrix)

        # Reshaping to a list of 3D coordinates
        self.pointCloud = points.reshape(\
            (points.shape[0]*points.shape[1],3))[0::self.pointSubsample]\
                                                .astype(np.int16)

        if self.verbose:
            print("".join(["Points in unfiltered pointcloud: {}; ",\
                    "completed in {:.5f} sec"]).format(\
                    self.pointCloud.shape[0], \
                    self.timeKeeper.returnPerfCounter()))

    
    def filterPointCloud(self):
        """Filter extreme points from the generated point cloud"""
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
                                    [ 0,  1,  0],
                                    [ 1,  0,  0] ])
        self.rotatePointCloud(rotationMatrix)


    def rotatePointCloud(self, rotationMatrix):
        """Rotate the point cloud using the given rotation matrix"""
        self.pointCloud = np.dot(self.pointCloud[:], rotationMatrix)
        

    def displayGrid_Internal(self, grid):
        """Display the generated unfiltered/filtered point clouds or
        voxel grids using matplotlib. Calling without a separate thread
        will block the calling thread"""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        ax.scatter(grid[:,0], \
                   grid[:,1], \
                   grid[:,2])

        ax.set_xlabel("$x$")
        ax.set_ylabel("$y$")
        ax.set_zlabel("$z$")

        # Assuming camera axis begins at 0
        ax.set_xlim(-1000,1000)
        ax.set_ylim(0,2000)
        ax.set_zlim(-1000,1000)

        plt.show(block=True)

    
    def resetVoxelGrid(self):
        """Reinitialize voxel grid"""
        self.voxelGrid = None
        print("Voxel grid reset")


    def displayGrid(self, grid):
        """Display the generated unfiltered/filtered point clouds or
        voxel grids using matplotlib"""
        displayThread = threading.Thread(\
            target=self.displayGrid_Internal, \
            args=(grid,))
        displayThread.start()

    
    def voxelizePointCloud(self):
        """Create a voxel grid representation of the point cloud. 
        Filter the point cloud before voxelizing"""
        if self.verbose:
            self.timeKeeper.startPerfCounter()

        iterations = 0
        voxelGrid = []
        initialSize = self.pointCloud.shape[0]
        remainingPoints = initialSize
        
        while remainingPoints>(initialSize/self.voxelStopFraction):

            sampledPoint = \
                self.pointCloud[np.random.randint(0,remainingPoints)]

            samplingLimit = np.empty_like(sampledPoint)
            for n in range(len(sampledPoint)):
                samplingLimit[n]=\
                    (sampledPoint[n]//self.voxelSize)*self.voxelSize

            mask = np.ones(remainingPoints, dtype=bool)

            for n in range(len(sampledPoint)):
                mask = np.logical_and(mask, np.logical_and(\
                    self.pointCloud[:,n]>=samplingLimit[n], \
                    self.pointCloud[:,n]<samplingLimit[n]+self.voxelSize))

            pointsInVoxel = self.pointCloud[mask]

            if len(pointsInVoxel)>self.occupancyThreshold:
                voxelMidpoint = samplingLimit+self.voxelSize/2
                voxelGrid.append(voxelMidpoint)

            self.pointCloud = self.pointCloud[np.invert(mask)]

            iterations+=1

            remainingPoints = self.pointCloud.shape[0]

        voxelGrid = np.array(voxelGrid, dtype=np.int16)

        ### These steps will potentially need to be changed

        if self.voxelGrid is None:
            self.voxelGrid = voxelGrid

        else:
            self.voxelGrid = np.unique(\
                    np.vstack((self.voxelGrid, voxelGrid)), axis=0)

        ###

        if self.verbose:
            print("".join(["Voxels in grid: {}; ",\
                    "completed in {:.5f} sec; {} iterations"]).format(\
                    self.voxelGrid.shape[0], \
                    self.timeKeeper.returnPerfCounter(), \
                    iterations))


    def viewVoxelGrid(self):
        """Compute and display voxel grid"""
        self.generatePointCloud()
        self.filterPointCloud()
        self.redefinePointCloudCoordinate()
        self.voxelizePointCloud()
        self.displayGrid(self.voxelGrid)



if __name__=="__main__":
    print("Import to use")
    