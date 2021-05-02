class VoxelGrid:
    """"""
    def __init__(self, voxelSize):
        self.voxelSize = voxelSize


    def importPointCloud(self, points):
        """Import a list of points in 3D as a point cloud"""
        self.points = points
