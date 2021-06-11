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
        self.robotFootprint = np.array([0, 0, 20])


    def setLocation(self, location):    
        """Set robot location"""
        self.robotFootprint[:2] = location[:]


    def checkCollision(self, obstacleFootprint):
        """Check collison with obstacle"""    
        distance = np.linalg.norm(self.robotFootprint[:2]-obstacleFootprint[:2])
        
        if distance<(self.robotFootprint[2]+obstacleFootprint[2]):
            return True
        
        else:
            return False  


if __name__=="__main__":
    obstacleFootprint = np.array([10, 5, 12])

    collisionChecker = CollisionChecker2D()
    collisionChecker.setLocation([5, 5])
    

    collision = collisionChecker.checkCollision(obstacleFootprint)
    if collision:
        print("Collided")

    else:
        print("Avoided Collision")    

    swath = Swathcomputation()
    swath.setdestination([10,10])   
    swath.translation() 