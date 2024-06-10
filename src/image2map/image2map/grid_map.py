import rclpy 
import rclpy.node as node 
import numpy as np
import matplotlib.path as mpath

## These are just used for debugging 
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
# from matplotlib.collections import LineCollection

from nav_msgs.msg import OccupancyGrid
from interfaces.srv import CreateOccupancyMap

from image2map.utils import pcl2array, euler_from_quaternion


class GridMapper(node.Node):
    def __init__(self):
        super().__init__('gridmapper')

        self.declare_parameters(
            namespace='',
            parameters = [('world_frame', 'world'),
                          ('horizontal_fov', np.pi/3),
                          ('camera_depth_lim', 30.),
                          ('point_distance_tolerance', 1.2),
                          ('voxel_connection_tolerance', 1.95),
                          ('angular_tolerance', 0.005)]
        )

        self.world_frame = self.get_parameter('world_frame').value

        # The horizontal field of view of the camera - this is used to see what the 'vision cone' of the camera is
        self.fov = self.get_parameter('horizontal_fov').value
        # The depth to which the camera data is reconstructed 
        self.depth_lim = self.get_parameter('camera_depth_lim').value

        self.voxel_tolerance = self.get_parameter('voxel_connection_tolerance').value
        self.point_dist_tolerance = self.get_parameter('point_distance_tolerance').value

        # This parameter is used for checking whether two points are close to each other in angular position.
        # This is the maximum length of the norm of (unit_vector_1 - unit_vector_2). This is used to check whether one point obscures the other
        self.angular_tolerance = self.get_parameter('angular_tolerance').value

        # Set up publisher for map visualisation
        self.map_serv = self.create_service(CreateOccupancyMap,
                                            'create_occupancy_map',
                                            self.update_map)        

        # Initialise variables 
        self.position = np.array([0.0, 0.0], dtype=float)

        # Initialise variables for occupancy grid creation
        self.resolution = .25   # Resolution of occupancy grid (metres per cell)
        self.x_width      = 75.0 # Width of occupancy grid (metres)
        self.y_width     = 75.0  # Height of occupancy grid (metres)

        self.grid       = None

        # This is the number of points that will be looked into the future, to see whether a more preferable point is available 
        self.lookahead_points = 5

    def update_map(self, request, result) -> CreateOccupancyMap:
        """
        Update occupancy grid with new pointcloud messages.
        """
        position = request.pose.position 
        pos_ar = np.array([position.x, position.y]) 
        orientation = request.pose.orientation
        yaw_angle = euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)[2]
        pcl = pcl2array(request.pointcloud)[:,:2]

        # Create occupancy grid using Lidar data and relative position
        if self.grid is None:
            # Initialize occupancy grid
            self.grid = OccupancyGrid()
            self.grid.header.frame_id = self.world_frame
            self.grid.info.resolution = self.resolution
            self.grid.info.width = int(self.x_width/self.resolution)
            self.grid.info.height = int(self.y_width/self.resolution)
            self.grid.info.origin.position.x = position.x - self.x_width / 2 
            self.grid.info.origin.position.y = position.y - self.y_width / 2
            self.grid.data = [-1]*self.grid.info.width*self.grid.info.height


            mesh_grid_x, mesh_grid_y = np.meshgrid(np.arange(self.grid.info.origin.position.x + self.resolution/2, self.grid.info.origin.position.x + self.x_width + self.resolution/2, self.resolution),
                                    np.arange(self.grid.info.origin.position.y + self.resolution/2, self.grid.info.origin.position.y + self.y_width + self.resolution/2, self.resolution))
            self.mesh_points = np.vstack((mesh_grid_x.flatten(), mesh_grid_y.flatten())).T

        # Rotate pcl points to global orientation and shift reference frame to the origin of the grid map
        rot_mat = np.array([[np.cos(yaw_angle), -np.sin(yaw_angle)],
                            [np.sin(yaw_angle), np.cos(yaw_angle)]])
        
        origin =  np.array([self.grid.info.origin.position.x, self.grid.info.origin.position.y])

        # Transform to the global reference frame (rotate to align, then add translation to position)
        global_pcl = np.einsum("bi, ij -> bj", pcl, rot_mat.T) + pos_ar 

        # Transform to the gridmap reference frame, which has the origin in one corner 
        pcl_wrt_origin = global_pcl - origin

        # TODO: CHECK WHETHER ANY OF THE POINTS ARE NEGATIVE - IF SO, INCREASE SIZE AND SHIFT ORIGIN - also update meshgrid
        assert (pcl_wrt_origin >= 0).all(), "Points should be positive with respect to the origin - if they are negative the map size should be increased and the origin shifted"

        pcl_2dim_indices = np.floor(pcl_wrt_origin / self.resolution).astype(int)
        pcl_flat_indices = self.grid.info.width * pcl_2dim_indices[:,1] + pcl_2dim_indices[:,0]

        for idx in pcl_flat_indices:
            self.grid.data[idx] = 100
        
        empty_indices, high_certainty_observed_indices = self.fill_out_polygon(pos_ar, yaw_angle, global_pcl, pcl)
        for idx in empty_indices:
            # previously unkown, now it is observed
            if self.grid.data[idx] == -1:
                self.grid.data[idx] = 30 
            # elif self.grid.data[idx] != 100:
            #     self.grid.data[idx] = int(self.grid.data[idx] * self.prev_weight + (1-self.prev_weight) * 30)
        
        for idx in high_certainty_observed_indices:
            if self.grid.data[idx] != 100:
                self.grid.data[idx] = 1 # int(self.grid.data[idx] * self.prev_weight + (1-self.prev_weight) * 1)

        
        # Publish occupancy grid
        result.occupancygrid = self.grid
        return result
    
    def fill_out_polygon(self, position, yaw_angle, pointcloud, pointcloud_relative):
        """
        This function draws a polygon of the area that has been observed. This polygon is essentially the 'cone of vision', with the 
        obstacles and their projection cut out. The polygon is used to obtain the indices of the observed gridmap cells. 

        A distinction is made between 'observed' cells and 'high certainty' cells. An observed cell is a cell that is within the cone of vision,
        and not obstructed by any point in the pointcloud. However, the pointcloud is far from perfect. When there is no pointcloud point in a 
        certain position, that does not mean there is no obstacle, it may simply have been unable to reconstruct that point. This means that you can
        not be sure a point in the cone was actually observed, instead there may have been an obstacle that was missed. 

        The 'high certainty' cells are cells that were traced in line with an observed pointcloud point. If a pointcloud point exists, most likely
        the cells between the current position and the pointcloud point is actually empty. These are indicated as the high certainty cells. 

        The algorithm that is followed in this function is the following:
            All points are ordered by their radial angle position.
            Loop through the points
            - If the previous point is not close to the new point, project the previous and new point to the 'back wall' of the vision cone
            - Add the point to the list of vertices

            Additionally, the next few point are checked on two criteria
            - The future point is close to the current point in angular position
            - The future point is closer to the position of the vehicle than the current point
            If these two criteria are met, the loop skips to this point. This ensures that points close to the vehicle obscure points behind them.
            If there are two pointcloud clusters behind each other, we want to skip all of the points of the furthest cluster, because this would cause a very
            jagged polygon that may lead to obscured gridpoints being set to seen. 
        
        """
        
        # Draw a triangle with an angle equivalent to the fov, and a depth equivalent to the maximum camera depth 
        outer_left_relative = np.array([np.cos(self.fov/2), np.sin(self.fov/2)]) * self.depth_lim / np.cos(self.fov/2)
        outer_right_relative = np.array([np.cos(-self.fov/2), np.sin(-self.fov/2)]) * self.depth_lim / np.cos(-self.fov/2)

        rot_mat_T = np.array([[np.cos(yaw_angle), np.sin(yaw_angle)],
                            [-np.sin(yaw_angle), np.cos(yaw_angle)]])

        outer_point_left = position + outer_left_relative @ rot_mat_T
        outer_point_right = position + outer_right_relative @ rot_mat_T
        

        # Pad by adding a point on either side of the pointcloud points
        # Pad size of 0.5 means that on either side of a point, another point will be drawn a distance of half a resolution away.
        # pad_size = 1
        # perp_vec = pad_size* self.resolution * np.array(np.sin(yaw_angle), np.cos(yaw_angle))
        pointcloud = np.vstack((pointcloud, outer_point_left, outer_point_right))
        # pointcloud = np.vstack((pointcloud, pointcloud - perp_vec, pointcloud + perp_vec, outer_point_left, outer_point_right))

        # Sort points by angular position
        sorting_pcl = np.vstack((pointcloud_relative, outer_left_relative, outer_right_relative))
        pcl = pointcloud[np.arctan2(sorting_pcl[:,1], sorting_pcl[:,0]).argsort()]

        # The polygon should start at the current position, then move to the corner of the triangle
        vertices = [np.array((position[0], position[1]))]
        seen_vertices = [np.array((position[0], position[1]))]
        skip_points = 0   
        recently_skipped = False
        total_points = pcl.shape[0]
        for idx, point in enumerate(pcl):
            if skip_points > 0:
                skip_points -= 1
                if skip_points == 0:
                    recently_skipped = True
                continue
            
            # Check how far the point is from the previous one to link them up
            # The point dist tolerance ensures that points closer by get priority. This means that points in the foreground can skip over poinst in the background
            # but points in the background cannot cause a skip over a point in the foreground
            point_dist = np.linalg.norm(point - position)
            unit_dir = (point - position) / point_dist

            # Check whether there are points coming up that would make it preferable to skip the current one 
            forecast = np.array([(np.linalg.norm((pcl[idx+i]-position)/np.linalg.norm(pcl[idx+i]-position) - unit_dir) < self.angular_tolerance)
                                and
                                (np.linalg.norm(pcl[idx+i] - position) < (point_dist * self.point_dist_tolerance))
                                for i in range(1,self.lookahead_points+1) if idx+i < total_points])
            
            prefered_next_points = np.where(forecast)[0]
            if prefered_next_points.shape[0] > 0 and not recently_skipped:
                skip_points = prefered_next_points[0]
                continue
            recently_skipped = False
            
            if np.linalg.norm(vertices[-1] - point) > self.voxel_tolerance * self.resolution:
                if idx > 0:
                    prev_proj = self.project_point_backwall(position, vertices[-1], yaw_angle)
                    vertices.append(prev_proj)
                    seen_vertices.append(position)

                current_proj = self.project_point_backwall(position, point, yaw_angle)
                vertices.append(current_proj)
        
            vertices.append(point)
            if idx > 0 and idx < total_points - 1:
                seen_vertices.append(point)

        # Close the loop
        vertices += [np.array((position[0], position[1]))]
        seen_vertices.append(position)

        # =============================================== PLOTS FOR DEBUGGING ===============================================  
        # vertex_arr = np.array(vertices)
        
        # vertex_nr = list(range(1,vertex_arr.shape[0]))


        # fig, ax = plt.subplots()
        # # lc = LineCollection(segments=vertex_arr, cmap='viridis',linewidth=4)
        # lc = LineCollection(segments=[(vertices[i-1], vertices[i]) for i in range(1, len(vertices))], cmap='jet',linewidth=1)
        # lc.set_array(vertex_nr)
        # line = ax.add_collection(lc)
        # axis_lims = [np.min(vertex_arr[:,0]), np.max(vertex_arr[:,0]), np.min(vertex_arr[:,1]), np.max(vertex_arr[:,1])]
    
        # ax.axis(axis_lims) #set axis limits. This is [xlow, xhigh, ylow, yhigh]
        # plt.colorbar(line)
       
        # plt.show()
        # input()

        #===================================================================================================================

        path = mpath.Path(vertices, closed=True)
        seen_path = mpath.Path(seen_vertices, closed=True)

        # Check what the gridpoints are that fall within the polygon
        observed_mask = path.contains_points(self.mesh_points, radius=-2*self.resolution)
        observed_indices = np.where(observed_mask)[0]

        # Check what points are on the lines traced between the current position and the pointcloud points 
        # These are most likely to actually be empty 
        high_certainty_observed_map = seen_path.contains_points(self.mesh_points, radius=self.resolution)
        high_certainty_observed_indices = np.where(high_certainty_observed_map)[0]

        return observed_indices, high_certainty_observed_indices
    
    def project_point_backwall(self, position, point, yaw_angle):
        ray = point - position
        ray_dir = ray / np.linalg.norm(ray)
        relative_ray_angle = np.arctan2(ray_dir[1], ray_dir[0]) - yaw_angle
        projected_point = position + ray_dir * self.depth_lim / np.cos(relative_ray_angle)
        return projected_point
    

def main(args=None):
    rclpy.init(args=args)
    
    mapper = GridMapper()
    rclpy.spin(mapper)

    mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
