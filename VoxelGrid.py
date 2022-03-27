import threading

import cv2
import matplotlib.pyplot as plt
import numpy as np

from helperScripts import nputils
from helperScripts.TimeKeeper import TimeKeeper


class VoxelGrid:
    """Class to process and handle voxelized representation of 
    pointclouds"""
    def __init__(self, stereoMatcher):
        # Paramaters
        # Size of each voxel in mm
        self.voxelSize = 100
        # Fraction of raw points taken
        self.pointSubsample = 20
        # Stop voxelizing when less than this fraction of points remain
        self.voxelStopFraction = 10 
        # Minimum number of points after a voxel is marked occupied
        self.occupancyThreshold = 10
        # Distance from camera inside which voxels may be refined
        self.voxelCheckDistance = 1500

        # Rotation matrix to redefine camera axis
        self.redefineRotationMatrix = np.array([ [ 0,  0, -1],
                                                 [ 0,  1,  0],
                                                 [ 1,  0,  0] ])

        # Voxel grid
        self.voxelGrid = None

        # Object references
        self.stereoMatcher = stereoMatcher
        
        # Debug
        self.verbose = True
        if self.verbose:
            self.timeKeeper = TimeKeeper()

        # Flags
        self.stateBufferReady = False
        self.stateEventReady = False

        # Camera fields of view
        # Horizontal field of view (degrees)
        self.fovH = ((self.stereoMatcher.imageProcessor.fovYL + \
                    self.stereoMatcher.imageProcessor.fovYR)/4)*np.pi/180
        self.fovH -= self.fovH/8

        # Vertical field of view (degrees)
        self.fovV = ((self.stereoMatcher.imageProcessor.fovXL + \
            self.stereoMatcher.imageProcessor.fovXR)/4)*np.pi/180
        self.fovV -= self.fovV/8


    
    def referenceStateBuffers(self, buffers, bufferLength):
        """Create class references to passed state buffers"""
        # Creating class references to buffers
        assert buffers is not None and bufferLength is not None, \
                                            "Initialize state buffers"
        self.rotationBuffer = buffers[0]
        self.positionBuffer = buffers[1]

        self.stateBufferLength = bufferLength

        # Creating wrapper arrays from memory buffers
        self.rotationWrapper = np.frombuffer(self.rotationBuffer, \
                dtype=np.float64).reshape((self.stateBufferLength, 3, 3))
        self.positionWrapper = np.frombuffer(self.positionBuffer, \
                dtype=np.float64).reshape((self.stateBufferLength, 4))

        self.stateBufferReady = True

    
    def isStateBufferReady(self):
        """Check if capture buffers have been referenced"""
        if self.stateBufferReady:
            return True
        
        else:
            print("State buffers not referenced")
            return False


    def referenceStateEvent(self, event):
        """Create class references to passed visual odometry state buffer
        write event"""
        self.stateEvent = event

        self.stateEventReady = True


    def isStateEventReady(self):
        """Check if visual odometry state event has been referenced"""
        if self.stateEventReady:
            return True

        else:
            print("Visual odometry state event not referenced")
            return False


    def isVisualOdometryPipelineReady(self):
        """Check if visual odometry pipeline is ready"""
        if not self.isStateBufferReady() or not \
            self.isStateEventReady():
            print("Visual odometry pipeline not ready")
            return False
        
        else:
            return True


    def resetVoxelGrid(self):
        """Reinitialize voxel grid"""
        self.voxelGrid = None
        print("Voxel grid reset")


    def getStateFromBuffers(self):
        """Get camera state at pickupTime from Visual Odometry state buffers."""
        # Masking required estimate from buffer based on time
        if self.stateEvent.wait():            
            timeDiff = abs(self.positionWrapper[:,3] - \
                self.stereoMatcher.imageProcessor.pickupTime)
            index = np.argmin(timeDiff)

            # Extracting required state estimates with mask
            self.rotationEstimate = self.rotationWrapper[index].T
            self.positionEstimate = self.positionWrapper[index][:-1]


    def generatePointCloud(self):
        """Generate point cloud from disparity map and disparity-to-depth
        mapping matrix, Q"""
        if self.verbose:
            self.timeKeeper.startPerfCounter()

        points = cv2.reprojectImageTo3D(\
                    self.stereoMatcher.disparityMapL, \
                    self.stereoMatcher.imageProcessor.dispToDepthMatrix)

        # Reshaping to a list of 3D coordinates
        self.pointCloud = points.reshape(\
            (points.shape[0]*points.shape[1],3))[0::self.pointSubsample]\
                                                .astype(np.int16)

        if self.verbose:
            print("".join(["\nPoints in unfiltered pointcloud: {}; ",\
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


    def translatePointCloud(self, positionVector):
        """Translate point cloud by given position vector"""
        self.pointCloud += positionVector


    def rotatePointCloud(self, rotationMatrix):
        """Rotate point cloud using given rotation matrix"""
        self.pointCloud = np.dot(self.pointCloud[:], rotationMatrix)

    
    def redefinePointCloudCoordinate(self):
        """Rotate point cloud so that camera faces +x, with z
        vertical"""
        self.rotatePointCloud(self.redefineRotationMatrix)


    def voxelizePointCloud(self):
        """Create a voxel grid representation of the point cloud. 
        Filter the point cloud before voxelizing"""
        if self.verbose:
            self.timeKeeper.startPerfCounter()

        iterations = 0
        newVoxelGrid = []
        initialSize = self.pointCloud.shape[0]
        remainingPoints = initialSize
        samplingLimit = np.zeros_like(self.pointCloud[0])
        
        while remainingPoints>(initialSize/self.voxelStopFraction):

            sampledPoint = \
                self.pointCloud[np.random.randint(0,remainingPoints)]

            for n in range(3):
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
                newVoxelGrid.append(voxelMidpoint)

            self.pointCloud = self.pointCloud[np.invert(mask)]

            iterations+=1

            remainingPoints = self.pointCloud.shape[0]

        self.newVoxelGrid = np.array(newVoxelGrid, dtype=np.int16)

        if self.verbose:
            print("".join(["Voxels in new grid: {}; ",\
                    "completed in {:.5f} sec; {} iterations"]).format(\
                    self.newVoxelGrid.shape[0], \
                    self.timeKeeper.returnPerfCounter(), \
                    iterations))


    def getNewVoxelGrid(self):
        """Generate new voxel grid from disparity map"""
        # Compute point cloud
        self.generatePointCloud()
        # Filter point cloud
        self.filterPointCloud()
        # Rotate point cloud
        self.rotatePointCloud(self.rotationEstimate)
        # Translate point cloud
        self.translatePointCloud(self.positionEstimate)

        # Compute new voxel grid
        self.voxelizePointCloud()


    def findVoxelsInRange(self):
        """Find voxels that are within given distance from camera position 
        in (base) voxelGrid, along with yaw and distance"""
        # Compute distance to all voxels in grid
        translatedVoxels = self.voxelGrid - self.positionEstimate
        distanceToVoxels = np.linalg.norm(translatedVoxels, axis=1)

        # Voxels in base grid in range of camera
        self.voxelsInRange = \
            self.voxelGrid[distanceToVoxels<=self.voxelCheckDistance]
        translatedVoxelsInRange = self.voxelsInRange - self.positionEstimate

        # Distance of those voxels from camera
        self.distanceToVoxelsInRange = \
            distanceToVoxels[distanceToVoxels<=self.voxelCheckDistance]

        # Yaw of those voxels with respect to world frame translated to camera
        self.yawToVoxelsInRange = \
            np.arctan2(translatedVoxelsInRange[:,1], \
                                        translatedVoxelsInRange[:,0])


    def findCameraYawRange(self):
        """Find camera yaw and field of view in terms of yaw"""
        cameraDirectionVector = np.array([0,0,100])
        cameraDirectionVector = \
            np.dot(cameraDirectionVector, self.rotationEstimate)

        self.cameraYaw = \
            np.arctan2(cameraDirectionVector[1], cameraDirectionVector[0])

        self.cameraYawRange = \
            np.array([self.cameraYaw+self.fovH, self.cameraYaw-self.fovH])

        # Wrapping around values at -180, 180 degrees
        for n in range(len(self.cameraYawRange)):
            if self.cameraYawRange[n]>np.pi:
                self.cameraYawRange[n] -= 2*np.pi
            if self.cameraYawRange[n]<=-np.pi:
                self.cameraYawRange[n] += 2*np.pi

        self.cameraYawRange = np.sort(self.cameraYawRange)[::-1]
        
        if self.verbose:
            print("Camera yaw: {:.5f}".format(self.cameraYaw*180/np.pi))
            print("Camera yaw range:", self.cameraYawRange*180/np.pi)


    def removeVoxelsInView(self):
        """Remove voxels in base grid that are in range and in view of 
        the camera"""
        if self.cameraYawRange[0]>np.pi/2 and self.cameraYawRange[1]<-np.pi/2:
            voxelsToRemove = self.voxelsInRange[np.logical_and(\
                self.yawToVoxelsInRange[:]>self.cameraYawRange[0], \
                self.yawToVoxelsInRange[:]<self.cameraYawRange[1]
                )]
        else:
            voxelsToRemove = self.voxelsInRange[np.logical_and(\
                self.yawToVoxelsInRange[:]<self.cameraYawRange[0], \
                self.yawToVoxelsInRange[:]>self.cameraYawRange[1]
                )]

        if voxelsToRemove.shape[0]!=0:
            self.voxelGrid = \
                nputils.in1d_dot_approach(self.voxelGrid, voxelsToRemove)

        if self.verbose:
            print("Voxels removed from base grid: {}".format(\
                                            voxelsToRemove.shape[0]))

    
    def combineVoxelGrids(self):
        """Combine unique voxels from given base and new voxel grids"""
        self.voxelGrid = np.unique(\
                np.vstack([self.voxelGrid, self.newVoxelGrid]), axis=0)

        if self.verbose:
            print("Voxels in combined grid: {}".format(\
                                            self.voxelGrid.shape[0]))


    def displayGrid_Internal(self, grid):
        """Display generated unfiltered/filtered point clouds or voxel 
        grids using matplotlib. Calling without a separate thread will 
        block the calling thread"""
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


    def displayGrid(self, grid):
        """Display the generated unfiltered/filtered point clouds or
        voxel grids using matplotlib"""
        displayThread = threading.Thread(\
                    target=self.displayGrid_Internal, args=(grid,))

        displayThread.start()


    def viewVoxelGrid(self):
        """Compute and display voxel grid"""
        self.generatePointCloud()
        self.filterPointCloud()
        self.redefinePointCloudCoordinate()
        self.voxelizePointCloud()
        self.displayGrid(self.voxelGrid)

    
    def assistedVoxelGlobalMapping(self):
        """Voxel global mapping routine assisted by Visual Odometry"""
        self.getStateFromBuffers()

        self.getNewVoxelGrid()

        if self.voxelGrid is not None:
            self.findCameraYawRange()
            self.findVoxelsInRange()
            self.removeVoxelsInView()
            self.combineVoxelGrids()

        else:
            self.voxelGrid = self.newVoxelGrid
        



if __name__=="__main__":
    print("Import to use")
    