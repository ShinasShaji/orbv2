import numpy as np
 

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
        