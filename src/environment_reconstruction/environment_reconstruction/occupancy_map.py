import rclpy 
import rclpy.node as node
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from environment_reconstruction.utils import pcl2array


class OccupancyConverter(node.Node):
    
    def __init__(self):
        
        # Resolution is the physical sidelength of one square in the grid
        self.resolution = 1.
        # -1 indicates unknown. Initialise empty square 
        self.grid = np.ones((1,1)) * -1
        
        # This is the physical position of cell [0,0] in the format [m m rad]. 
        # should be updated when the map expands towards the 'negative cell indices' 
        self.origin = (0, 0, 0)

        self.rot_mat = np.array([[np.cos(self.origin[2]), -np.sin(self.origin[2])],
                                 [np.sin(self.origin[2]), np.cos(self.origin[2])]])

        # If there are obstacles above this height it is ignored, the vehicle can drive underneath
        self.gap_height = 2.

        self.pcl_listener = self.create_subscription(PointCloud2,
                                                     'filtered_reconstruction',
                                                     self.update_map,
                                                     10)
        
        self.map_publisher = self.create_publisher(OccupancyGrid,
                                                   'occupancy_map',
                                                   10)
        
    def update_map(self, pointcloud):

        points = pcl2array(pointcloud)
        obstructing_points = points[np.where(points[:,2] <= self.gap_height)]

        """
        I may need pose information at the time the pcl was created.
        A cell can be obstructed, open, or unknown. I can see which ones are obstructed, but to see
        the difference between unknown and empty I should trace between the pose and the obstruction
        and set everything in between there to empty

        """



    def determine_idx_bounds(self, obstructing_xy):
        rotated_points = np.einsum("bi, ii->bi", obstructing_xy, self.rot_mat.T) 
        
        x_bounds = np.min(rotated_points[:,0]), np.max(rotated_points[:,0])
        y_bounds = np.min(rotated_points[:,1]), np.max(rotated_points[:,1])

        