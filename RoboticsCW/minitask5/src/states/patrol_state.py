import rospy
import math
import numpy as np 
from .abstract_state import AbstractState 

class PatrolState(AbstractState):
    """
    PatrolState - State for exploring the environment by moving to predefined waypoints
    """

    def __init__(self, robot):
        super().__init__(robot)
        self.robot = robot 
        self.movement = robot.movement_controller 
        self.sensor = robot.sensor_controller 
        
        # Grid-based exploration parameters
        self.grid_spacing = 2.0      # Distance between grid points in meters
        self.position_threshold = 0.5 # Distance threshold for reaching waypoints 
        self.wall_offset = 1.0       # Distance to maintain from walls 
        self.max_waypoint_attempts = 3  # Maximum attempts to reach a waypoint 

        # Position tracking for waypoint updates
        self.last_waypoint_position = None 
        self.current_waypoint = None 
        self.explored_waypoints = [] 
        self.waypoints = None 
        self.failed_waypoints = []  # Track failed attempts for each waypoint 


    def generate_waypoints(self):
        """
        generate_waypoints - Generates a list of waypoints to explore in the environment
        Hardcoded - frontier couldnt get working
        """

        # Grid on map
        default_points = [(-1.3, 4.2),(-1.3, -0),(-1.3, -3.5),(2.5, 4.2),(2.5, 0),(2.5, -3.5),(5.8, -3.5),(5.8, 0),(5.8, -4.2)]

        return default_points

    def check_waypoint_for_obstacle(self, x, y):
        """
        check_waypoint_for_obstacle - Checks if a waypoint is obstructed by an obstacle

        :param x: x-coordinate of the waypoint
        :param y: y-coordinate of the waypoint

        :return: True if the waypoint is obstructed, False otherwise
        """

        # Dont continue if map data is not available
        if self.sensor.map_data is None or self.sensor.map_info is None:
            rospy.logwarn("Map data not available. Cannot check for obstacles.")
            return None
        
        # Convert x, y coordinates to grid coordinates based on res
        grid_x, grid_y = int((x - self.sensor.map_info.origin.position.x) / self.sensor.map_info.resolution), int((y - self.sensor.map_info.origin.position.y) / self.sensor.map_info.resolution)

        # Check if the grid coordinates are within the map boundaries
        if (grid_x < 0 or grid_x >= self.sensor.map_info.width) or (grid_y < 0 or grid_y >= self.sensor.map_info.height):
            rospy.logwarn("Waypoint outside map bounds. Cannot check for obstacles.")
            return None
        
        # Convert 2D coordinates to 1D array index
        index = grid_y * self.sensor.map_info.width + grid_x

        # is greater than 0 (ie occupied by obstacle)
        return self.sensor.map_data[index] > 0
        
    def generate_nearby_waypoints(self, x, y):
        """
        generate_nearby_waypoints - Generates nearby waypoint if the current waypoint is obstructed 
        Uses 3x3 grid around the waypoint to find a valid point (assumes standard obstacle size)

        :param x: x-coordinate of the waypoint
        :param y: y-coordinate of the waypoint

        :return: New waypoint if found, None otherwise
        """

        for i in range(-1,1, 0.25):
            for j in range(-1,1, 0.25):
                new_x = x + i
                new_y = y + j
                if not self.check_waypoint_for_obstacle(new_x, new_y):
                    return (new_x, new_y)    
        return None      

    def get_next_waypoint(self):
        """
        get_next_waypoint - Returns the closest waypoint to the robot's current position
        If waypoint is too close (considered reached), it is removed from the list

        :return: Closest waypoint to explore
        """

        robot_pos = self.movement.get_current_position()

        closest = None
        closest_dist = float('inf')
        for waypoint in self.waypoints:
            if waypoint in self.explored_waypoints:
                continue
            dist = math.sqrt((waypoint[0] - robot_pos['x'])**2 + (waypoint[1] - robot_pos['y'])**2)

            # We'll consider the waypoint reached if we're within the threshold
            if dist < 1.5:
                self.waypoints.remove(waypoint)
                self.explored_waypoints.append(waypoint)
                continue

            if dist < closest_dist and waypoint not in self.explored_waypoints:
                closest = waypoint
                closest_dist = dist

        return closest

    def execute(self):
        """
        execute - Moves the robot to predefined waypoints to explore the environment
        """

        # Init new points if none have been initialised yet 
        if self.waypoints == None:
            self.waypoints = self.generate_waypoints() 

        # Start again if finished
        if self.waypoints == []:
            rospy.logwarn("No valid waypoints generated. Making more")
            self.waypoints = self.generate_waypoints()
        
        # Get closest waypoint to robot
        closest_waypoint = self.get_next_waypoint()

        # If no valid waypoints to explore - panic
        if closest_waypoint is None:
            rospy.logwarn("No valid waypoints to explore. Starting again.")
            self.waypoints = self.generate_waypoints()
            return

        # Set current waypoint to the closest one
        self.current_waypoint = closest_waypoint

        #  Remove current waypoint from list
        self.waypoints.remove(self.current_waypoint)

        # Check if the waypoint is obstructed by an obstacle
        if self.check_waypoint_for_obstacle(self.current_waypoint[0], self.current_waypoint[1]):
            new = self.generate_nearby_waypoints(self.current_waypoint[0], self.current_waypoint[1])
            if new:
                self.current_waypoint = new
            else:
                rospy.logwarn("Waypoint invalid, no neighbours, skipping")
                return

        # Gets the x, y, theta tuple from the list of generated waypoints  
        x,y =  self.current_waypoint 

        theta = self.movement.get_current_position()['theta']

        # Checks if this waypoint has already been explored 
        if  self.current_waypoint in self.explored_waypoints:
            rospy.loginfo("Skipping already explored waypoint: x=%.3f, y=%.3f, theta=%.3f", x, y, theta) 

        # Send goal to move to desired point 
        navigation_success, failed_points = self.movement.send_navigation_goal(x, y, theta, self.failed_waypoints)

        # Check if there is a path to the waypoint
        if self.sensor.available_nav_paths == 0:
            rospy.logwarn("No path for this waypoint (no path). Skipping")
            navigation_success = False

        # If the robot reached the waypoint
        if navigation_success:
            self.explored_waypoints.append( self.current_waypoint) 
            rospy.loginfo("Waypoint marked as explored: x=%.3f, y=%.3f, theta=%.3f", x, y, theta) 

            if self.waypoints == []:
                rospy.loginfo("No more waypoints to explore. Start again.")
                self.waypoints = self.generate_waypoints()   
                return


# ----------------- Not used (Better waypoints) ----------------- #


def generate_waypoints_unused(self):
        """Generate grid-based exploration waypoints and avoid walls.""" 

        # Get the complete map information
        map_info = self.sensor.get_map_info() 
        if map_info is None:
            rospy.logwarn("No map available for waypoint generation")
            return [] 

        validated_waypoints = [] 

        # Calculate map boundaries in world coordinates
        min_x = map_info.origin.position.x # Real-world coordinate of the map's origin point 
        min_y = map_info.origin.position.y # Real-world coordinate of the map's origin point 
        # Calculate maximum boundaries in the map 
        # map_info.width and map_info.height represent the number of grid cells in each dimension 
        # map_info.resolution represents the size of each grid cell in meters 
        max_x = min_x + (map_info.width * map_info.resolution) # multiplies the number of cells by the size of each cell and adds this to the origin position 
        max_y = min_y + (map_info.height * map_info.resolution)
        
        # Get current position to start exploration from nearest grid point to the robot 
        current_pose = self.movement.get_current_position() 
        # Dividing the current position by the grid spacing (current_pose['x'] / self.grid_spacing) 
        # Multiplying the result by the grid spacing again ((current_pose['x'] / self.grid_spacing) * self.grid_spacing) 
        # Rounds the final result to 3 decimal places 
        start_x = round((current_pose['x'] / self.grid_spacing) * self.grid_spacing, 3) 
        start_y = round((current_pose['y'] / self.grid_spacing) * self.grid_spacing, 3) 
        
       # Generate all waypoints using concatenation 
        self.waypoints = (self._generate_base_grid(start_x, start_y, max_x, max_y) + 
                        self._generate_wall_waypoints(map_info)) 

        # Validating all waypoints based on the cell free space 
        # Iterating through each waypoint in the waypoints list to ensure they are all safely traversable 
        for waypoint in self.waypoints: 
            # getting the x, y, and theta values from the waypoint tuple 
            x, y, theta = waypoint 
            # validation considers the map data, resolution, and origin to ensure the waypoint isn't inside or too close to obstacles 
            if self.is_waypoint_valid(x, y, map_info.data, map_info.resolution, map_info.origin):
                validated_waypoints.append((x, y, theta)) # add to list 
    
        return validated_waypoints  

def _generate_base_grid_unused(self, start_x, start_y, max_x, max_y): 
    waypoints_list = [] 

    # creates a sequence of x-coordinates from start_x to max_x, spaced according to self.grid_spacing 
    # enumerate function provides both the index (i) and value (x) for each iteration 
    for i, x in enumerate(np.arange(start_x, max_x, self.grid_spacing)): 

        # for each x coordinate, generate a sequence of y-coordinates from start_y to max_y 
        y_range = np.arange(start_y, max_y, self.grid_spacing)
        
        # when the x-index (i) is odd (i % 2 == 1), the y-range is reversed using np.flip() 
        # this creates a back-and-forth movement pattern 
        if i % 2 == 1:
            y_range = np.flip(y_range)
            
        for y in y_range: 
            # verifies that the point isn't near any previously explored waypoints 
            if not self._is_near_explored_waypoint(x, y): 
                # calculates the appropriate orientation (theta) for the robot at that position 
                theta = self._calculate_orientation(x, y, max_x, max_y)
                waypoints_list.append((x, y, theta)) # add to list 

    return waypoints_list   

def _generate_wall_waypoints_unused(self, map_info):
    """Generate additional waypoints along walls""" 
    waypoints_l = []
    resolution = map_info.resolution # defines the size of each grid cell in meters 
    
    # examines every cell in the map by iterating through the height and width dimensions 
    for y in range(map_info.height):
        for x in range(map_info.width): 
            # checks for a wall 
            if self._is_wall(x, y, map_info): 
                # if wall identified then convert the grid coordinates to world coordinates 
                world_x = x * resolution + map_info.origin.position.x
                world_y = y * resolution + map_info.origin.position.y
                
                # generates a series of waypoints around each wall cell using a circular pattern 
                # creates eight equidistant points around the wall (at 45-degree intervals) using trigonometric calculations 
                # these points are at a fixed distance from the wall 
                for angle in np.arange(0, 2*math.pi, math.pi/4):
                    offset_x = world_x + self.wall_offset * math.cos(angle)
                    offset_y = world_y + self.wall_offset * math.sin(angle)
                    
                    # checks if the waypoint is in a safe, traversable location 
                    if self.is_waypoint_valid(offset_x, offset_y, map_info.data, 
                                            resolution, map_info.origin): 
                        
                        # calculates the robot's orientation (theta) at each valid waypoint 
                        # orienting the robot to face the wall 
                        # this orientation is calculated using the arctangent of the relative positions 
                        theta = math.atan2(world_y - offset_y, world_x - offset_x)
                        waypoints_l.append((offset_x, offset_y, theta)) # append to list 
    
    return waypoints_l 

def _get_surrounding_indices_unused(self, x, y, width, height): 
    """ 
        This method identifies all valid neighboring cells around a given position in the occupancy grid. 
        Get indices of surrounding cells for obstacle by checking the entire immediate area around it. 
    """
    indices = [] 
    # uses a 3x3 grid centered on the target position 
    # the nested loops generate offsets of -1, 0, and 1 in both the x and y directions
    # these offsets, when added to the target position, 
    # create coordinates for all eight surrounding cells plus the center cell 
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]: 
            # verifies that both new coordinates (new_x and new_y) are non-negative 
            # and less than their respective maximum dimensions (width and height) 
            new_x, new_y = x + dx, y + dy 
            # when a valid position is identified, 2D grid coordinates is converted into a 1D array index 
            if 0 <= new_x < width and 0 <= new_y < height:
                indices.append(new_y * width + new_x) # each valid index is added to the indices list 
    return indices 

def _is_wall_unused(self, x, y, map_info):
    """Determines whether a specific grid cell in the occupancy grid map contains a wall or obstacle""" 
    # performs a boundary check to ensure the provided coordinates fall within the map's dimensions 
    if 0 <= x < map_info.width and 0 <= y < map_info.height: 
        # map data is stored as a one-dimensional array, 
        # converts the two-dimensional coordinates (x,y) into a single index 
        index = y * map_info.width + x 
        # Any cell with a value of 50 or greater is considered to contain a wall or obstacle 
        return map_info.data[index] >= 50
    return False 

def _is_near_explored_waypoint_unused(self, x, y): 
    """ 
        This method uses a mathematical approach based on the Euclidean distance formula, 
        to determine if a potential waypoint is too close to previously explored locations. 
    """
    # Iterate through the x, y coordinates of explored waypoints list 
    for explored_x, explored_y, _ in self.explored_waypoints: 
        # calculates the straight-line distance (Euclidean) between two points in a 2D space 
        # calculated distance is compared against half of the grid spacing 
        # creating a circular "buffer zone" around each explored waypoint with a radius equal to half the grid spacing 
        if math.sqrt((x - explored_x)**2 + (y - explored_y)**2) < self.grid_spacing/2:
            # if the new point falls within this buffer zone, indicates that the point is too close to an existing waypoint 
            return True 
    # otherwise, if the point maintains a safe distance from all previously explored waypoints it is a suitable candidate for exploration
    return False 

def _calculate_orientation_unused(self, x, y, max_x, max_y): 
    """ 
        This method calculates the orientation by using trigonometry to determine 
        the appropriate angle between the robot's current position and the room's center point. 
    """
    # to determine the center point of the room: 
    # use simple calculation by dividing the maximum coordinates by 2, resulting in center_x and center_y 
    center_x = max_x / 2
    center_y = max_y / 2 
    # the math.atan2() function computes the arctangent of the ratio between two values, returning the angle in radians 
    return math.atan2(center_y - y, center_x - x)  

def is_waypoint_valid_unused(self, x, y, map_data, map_resolution, map_origin): 
    """ 
        Check if a waypoint is valid and safe for navigation by examining both
        the target position and its surrounding cells. 
    """ 
    if map_data is None:
        return False

    map_info = self.sensor.get_map_info()
    if map_info is None:
        return False

    # Convert to grid coordinates
    grid_x = int((x - map_origin.position.x) / map_resolution)
    grid_y = int((y - map_origin.position.y) / map_resolution)
    
    # Use the width and height from map_info to verify if the grid coordinates 
    # (grid_x, grid_y) fall within the map boundaries by checking if they are non-negative 
    # and less than the map's width and height. 
    # This prevents the robot from attempting to move outside the known map area. 
    if 0 <= grid_x < map_info.width and 0 <= grid_y < map_info.height: 
        # Convert 2D coordinates to 1D array index as the map is stored as a 1D array 
        # formula used: index = grid_y * map_info.width + grid_x 
        # Calculate index for target cell
        center_index = grid_y * map_info.width + grid_x  
        
        # Get indices for surrounding cells 
        surrounding_indices = self._get_surrounding_indices(grid_x, grid_y, 
                                                        map_info.width, 
                                                        map_info.height)
        
        # Check if target cell is safe 
        if map_data[center_index] >= 50:
            return False
        
        # Check if all surrounding cells are safe
        return all(map_data[idx] < 50 for idx in surrounding_indices) # Less than 50 indicates traversable space 
    
    return False 

def is_exploration_complete_unused(self):
    """Check if 90% of map has been explored based on explored_waypoints list. """ 

    # Checks the percentage of map explored
    map_info = self.sensor.get_map_info() 

    if map_info is not None:
        # Calculate area covered by explored waypoints
        # Each grid point covers a square of grid_spacing x grid_spacing
        explored_area = len(self.explored_waypoints) * (self.grid_spacing ** 2)
        
        # Calculate total traversable area from map
        # Count traversable cells (value < 50 in map data)
        traversable_cells = sum(1 for cell in map_info.data if cell < 50)
        total_traversable_area = traversable_cells * (map_info.resolution ** 2)
        
        # Calculate coverage percentage
        if total_traversable_area > 0:
            coverage = (explored_area / total_traversable_area)
            rospy.loginfo(f"Current map coverage: {coverage*100:.1f}%")
            return coverage > 0.9  # Return True if 90% coverage achieved
    
    return False  


# ----------------- Not Finished/Used (Frontier Exploration) ----------------- #

def get_world_coordinates(self, index):
        """ 
        Convert grid cell index to world coordinates 
        """
        x = (index % self.sensor.get_map_width()) * self.sensor.get_map_resolution() + self.sensor.get_map_origin()['x']
        y = (index // self.sensor.get_map_width()) * self.sensor.get_map_resolution() + self.sensor.get_map_origin()['y']
        return (x, y) 

def get_cell_index(self, x, y):
        """
        Convert world coordinates to grid cell index
        """
        grid_x = int((x - self.sensor.get_map_origin()['x']) / self.sensor.get_map_resolution())
        grid_y = int((y - self.sensor.get_map_origin()['y']) / self.sensor.get_map_resolution())
        return grid_y * self.sensor.get_map_width() + grid_x  

def get_cell_neighbors(self, index):
        """ 
        Get indices of neighboring cells 
        """ 
        map_width = self.sensor.get_map_width()
        return [
            index - map_width,  # Up
            index + map_width,  # Down
            index - 1,          # Left
            index + 1           # Right
        ] 

def is_cell_unknown(self, index):
    """ 
    Check if cell is unknown 
    """
    return 0 <= index < len(self.sensor.get_map_data()) and self.sensor.get_map_data()[index] == -1

def is_cell_free(self, index):
    """ 
    Check if cell is free space 
    """ 
    return 0 <= index < len(self.sensor.get_map_data()) and self.sensor.get_map_data()[index] == 0

def is_frontier_cell(self, index):
    """ 
    Determine if a cell is a frontier cell by checking its boundary (free, unknown). 
    """
    if not self.sensor.get_map_data() or not self.is_cell_unknown(index):
        return False
            
    return any(self.is_cell_free(neighbor) for neighbor in self.get_cell_neighbors(index))

def find_frontiers(self):
    """ 
    Scans the map for frontier cells/regions, and filters invalid frontiers. 
    """
    if not self.sensor.get_map_data():
        return []

    frontiers = []
    for index in range(len(self.sensor.get_map_data())):
        if self.is_frontier_cell(index):
            frontier = self.get_world_coordinates(index)
            if self.is_valid_frontier(frontier):
                frontiers.append(frontier)
    return frontiers

def calculate_distance(self, point1, point2):
    """ 
    Calculate Euclidean distance between two points, 
    to maintain minimum distance between frontiers. 
    """
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def is_valid_frontier(self, frontier):
    """ 
    Check if a frontier point is valid, and 
    prevents revisiting explored areas. 
    """
    return not any(
        self.calculate_distance(frontier, visited) < self.frontier_distance_threshold 
        for visited in self.visited_frontiers
    )

def count_unknown_neighbors(self, frontier):
    """ 
    Counts number of unknown cells around a frontier point, 
    used in frontier prioritisation. 
    """ 
    index = self.get_cell_index(frontier[0], frontier[1])
    return sum(1 for neighbor in self.get_cell_neighbors(index) if self.is_cell_unknown(neighbor))

def select_best_frontier(self):
    """ 
    Select best frontier based on distance and information gain. 
    """
    frontiers = self.find_frontiers()
    if not frontiers:
        return None

    current_pos = self.movement.get_current_position()
    current_point = (current_pos['x'], current_pos['y'])
        
    frontier_priorities = [
        (frontier, self.count_unknown_neighbors(frontier) / 
        (self.calculate_distance(frontier, current_point) + 0.1))
        for frontier in frontiers
    ]
        
    return max(frontier_priorities, key=lambda x: x[1])[0] if frontier_priorities else None

# def create_navigation_goal(self, x, y): 
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position.x = x
#     goal.target_pose.pose.position.y = y
#     goal.target_pose.pose.orientation.w = 1.0
#     return goal 

# def navigate_to_point(self, x, y, timeout=30.0):
#     """ 
#     Navigate to a point and return success status 
#     """
#     goal = self.create_navigation_goal(x, y)
#     self.move_base_client.send_goal(goal)
#     success = self.move_base_client.wait_for_result(rospy.Duration(timeout))
#     return success and self.move_base_client.get_state() == GoalStatus.SUCCEEDED

# def execute(self):
#     """ 
#     Execute frontier exploration and generate new frontiers when robot moves to new cell.
#     """
#     # Get initial best frontier
#     frontier = self.select_best_frontier()
        
#     if not frontier:
#         rospy.loginfo("No valid frontiers found")
#         return
    
#     # Try to navigate to the frontier
#     if self.navigate_to_point(frontier[0], frontier[1]):
#         self.visited_frontiers.add(frontier)
#         rospy.loginfo(f"Successfully reached frontier at {frontier}")
        
#         # After reaching the frontier, regenerate frontiers from new position
#         new_frontiers = self.find_frontiers()
#         if new_frontiers:
#             rospy.loginfo(f"Found {len(new_frontiers)} new frontier points from current position")
#         else:
#             rospy.loginfo("No new frontiers found from current position")
#     else:
#         rospy.logwarn(f"Failed to reach frontier at {frontier}") 