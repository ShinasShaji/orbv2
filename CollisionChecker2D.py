import numpy as np

class Swathcomputation:
    """Class for computing swath footprint"""
    def __init__(self):
        self.initialLocation = np.array([0, 0])

    
    def setdestination(self, destination):
        """Setting the location of our robot"""
        self.initialLocation[:2] = destination[:]

    def translation(self):
        self.initialLocation[:] = np.array([collisionChecker.robotFootprint[0]+self.initialLocation[0], collisionChecker.robotFootprint[1]+self.initialLocation[1]])    



class CollisionChecker2D:
    """Class for 2D Collision Checking"""
    def __init__(self):
        self.robotFootprint = 200 # x, y, radius
        self.robotLocation = np.array([0, 0]).astype(np.int16)

        self.voxelGrid = None
        self.voxelSize = 100

        self.distanceThreshold = int(self.robotFootprint + (self.voxelSize/np.sqrt(2)))

        self.pathFree = True


    def setLocation(self, location):    
        """Set robot location"""
        self.robotLocation[:2] = location[:]


    def checkCollision(self, obstacleFootprint):
        """Check collison with obstacle"""    
        distance = np.linalg.norm(self.robotLocation[:2]-obstacleFootprint[:2])
        
        if distance<(self.robotFootprint+obstacleFootprint[2]):
            return True
        
        else:
            return False


    def setVoxelGrid(self, voxelGrid):
        """Set obstacle grid for collision checking"""
        self.voxelGrid = voxelGrid


    def checkPathForCollision(self, path):
        """Checks given path for collision with voxels in voxelGrid"""
        self.pathFree = True

        for point in path:
            distance = np.linalg.norm(self.voxelGrid[:,:2]-point, axis=1).astype(np.int16)

            voxelsInCollision = self.voxelGrid[distance<=self.distanceThreshold]

            if len(voxelsInCollision)>0:
                self.pathFree = False

                return False, point, voxelsInCollision

            else:
                continue
        
        return True, None, None
                


if __name__=="__main__":
    #obstacleFootprint = np.array([10, 5, 12])

    #collisionChecker = CollisionChecker2D()
    #collisionChecker.setLocation([5, 5])
    #

    #collision = collisionChecker.checkCollision(obstacleFootprint)
    #if collision:
    #    print("Collided")

    #else:
    #    print("Avoided Collision")    

    #swath = Swathcomputation()
    #swath.setdestination([10,10])   
    #swath.translation() 

    collisionChecker = CollisionChecker2D()

    path = np.linspace(0, 50, 6).reshape((-1, 1))
    path = np.hstack((path, np.zeros_like(path)))
    print("\nPath waypoints: \n", path, "\n")

    voxelGrid = (np.random.randint(100, size=(20, 2)))*10
    print("Random Voxel Grid: \n", voxelGrid, "\n")

    collisionChecker.setVoxelGrid(voxelGrid)
    pathFree, collisionObstacle, voxelsInCollision = collisionChecker.checkPathForCollision(path)        

    if pathFree:
        print("Path free")

    else:
        print("Path collision at", collisionObstacle, "with voxels\n", voxelsInCollision)
