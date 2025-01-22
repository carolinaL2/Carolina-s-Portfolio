import rospy 
import actionlib 
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from visualization_msgs.msg import MarkerArray, Marker 
from tf.transformations import quaternion_from_euler 
import math
import random 
from copy import copy



class MovementController():
    """
    MovementController - A class to control the movement of the robot
    """


    def __init__(self,robot): 
        self.linear = {'x': 0}
        self.angular = {'z': 0}
        self.current_position = {'x':0, 'y':0, 'theta': 0}
        self.reference_position = {'x':0, 'y':0, 'theta': 0}
        self.move_speed = 0.3
        self.rotate_speed = 0.3
        self.robot = robot

        # Move base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        
        # RViz Marker setup 
        self.waypoint_pub = rospy.Publisher('exploration_waypoints', MarkerArray, queue_size=10) 
        # Navigation setup 
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10) 

        # Velocity command publisher
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  

    def wait_for_move_base_server(self, timeout=10.0):
        """
        wait_for_move_base_server - Waits for the move_base action server to become available.

        :param timeout: The time to wait for the server to become available.
        :return: True if the server is available, False otherwise.
        """


        if not self.move_base_client.wait_for_server(rospy.Duration(timeout)):
            rospy.logerr(f"move_base action server not available after {timeout} seconds.")
            return False
        return True

    def send_navigation_goal(self, x, y, theta, failed_wp):
        """
        send_navigation_goal - Sends a navigation goal to the move_base action server

        :param x: The x-coordinate of the goal
        :param y: The y-coordinate of the goal
        :param theta: The orientation of the goal
        :param failed_wp: A list of failed waypoints
        :return: A tuple containing a boolean indicating success and the list of failed waypoints
        """

        # First otherwise fail the goal
        if not self.wait_for_move_base_server():
            return (False, failed_wp) 
        
        # Store the current time
        self.timestamp = rospy.get_time()

        # Create a new MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y 

        # Convert angle to quaternion orientation 
        q = quaternion_from_euler(0, 0, theta) # Roll=0, Pitch=0, Yaw=theta 
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3] 

        retry_count = 0 
        max_retries = 2 


        # Attempt to send the goal
        while retry_count < max_retries: 
            result_state = self.move_base_client.get_state()

            # Avoid overwriting the goal when it's still active
            if result_state == GoalStatus.ACTIVE:

                # Double check if we should override the goal - IE object detected
                if self.robot.sensor_controller.is_object_detected():
                    rospy.loginfo("Object detected - cancelling goal") 
                    self.move_base_client.cancel_goal()
                    return (False, None)

                # Check if we should timeout the goal
                if rospy.get_time() - self.timestamp > 30.0:
                    rospy.loginfo("Timed out on that waypoint - carrying on")
                    self.move_base_client.cancel_goal()
                    return (False, failed_wp)
                
                # Otherwise, just wait for the goal to complete
                rospy.loginfo("Navigation goal is active (not overriding)...") 
                rospy.sleep(1.0) 
                continue

            # Send the goal
            self.move_base_client.send_goal(goal)
            rospy.loginfo(f"Sending navigation goal to move_base: x={x}, y={y}, theta={theta}") 
            
            # Wait for result with timeout - 2 seconds to allow interupt of oject detection
            self.move_base_client.wait_for_result(rospy.Duration.from_sec(2.0)) 

            # Check for success
            if result_state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation goal succeeded!")
                return (True, failed_wp) 
            
            # Check for terminal failure states
            elif result_state == GoalStatus.LOST:
                rospy.logwarn("Robot lost! Attempting to recover...")  
                rospy.logwarn(f"Adding failed waypoint to list: x={x:.2f}, y={y:.2f}")  # Add failed coordinates to the list 
                failed_wp.append((x, y))  # Add failed coordinates to the list 
                self.move_base_client.cancel_goal() 
                return (False, failed_wp) 
                    
            # Check for terminal failure states
            elif result_state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                rospy.logwarn(f"ABORTED/REJECTED with state {self.move_base_client.get_state()}") 
                self.move_base_client.cancel_goal()
                break 
                
        # Retry the goal
        retry_count += 1
        if retry_count < max_retries:
            rospy.loginfo(f"Retrying navigation goal (attempt {retry_count + 1} of {max_retries})") 
                
        rospy.logwarn(f"Navigation goal failed after {max_retries} attempts")
        return (False, failed_wp)  

    def rotate_to(self, angle):
        """
        rotate_to - Rotates the robot to a specific angle without interruptions
        
        :param angle: The angle to rotate to
        """

        self.is_rotating = True
        self.set_angular(self.rotate_speed * 1.5)
        print(f"Rotating to {angle}")
        while self.is_rotating:
            self.publish_movement()
            if abs(self.normalise_angle(self.current_position['theta'] - angle)) < 0.1:
                self.is_rotating = False
                self.set_angular(0)
                self.publish_movement()
                break

    def random_direction(self): 
        """
        random_direction - Returns a random angle between 0 and 2*pi (0 and 360 degrees)
        """
        
        return random.uniform(0, 2 * math.pi)
    
    def publish_movement(self):
        """
        publish_movement - Publishes the current linear and angular velocities to the robot (via cmd_vel topic)
        """    

        move_cmd = Twist()  
        move_cmd.linear.x = self.linear['x']
        move_cmd.angular.z = self.angular['z']
        self.pub.publish(move_cmd)

    def calculate_distance(self):
        """
        calculate_distance - Calculates distance between current pos and stamped reference pos

        :return: The distance in meters
        """


        dx = self.current_position['x'] - self.reference_position['x']
        dy = self.current_position['y'] - self.reference_position['y']
        return math.sqrt(dx**2 + dy**2)
    
    def normalise_angle(self, angle):
        """
        normalise_angle - Normalises an angle to the range -pi to pi (circular normalisation)

        :param angle: The angle to normalise

        :return: The normalised angle
        """


        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    

# ----------------- Getter and Setter methods ----------------- #

    def get_angular(self):
        return self.angular

    def get_linear(self):
        return self.linear

    def get_current_position(self):
        return self.current_position

    def get_reference_position(self):
        return self.reference_position

    def get_goal_publisher(self):
        return self.goal_pub 

    def get_move_base_client(self):
        return self.move_base_client 

    def get_waypoint_publisher(self):
        return self.waypoint_pub 

    def set_position(self, x, y, theta):
        self.current_position['x'] = x
        self.current_position['y'] = y
        self.current_position['theta'] = theta

    def set_reference_position(self, x, y, theta):
        self.reference_position['x'] = x
        self.reference_position['y'] = y
        self.reference_position['theta'] = theta

    def set_angular(self, angular_z):
        self.angular['z'] = angular_z

    def set_linear(self, linear_x):
        self.linear['x'] = linear_x

    def set_movement(self, linear_x, angular_z):
        self.linear['x'] = linear_x
        self.angular['z'] = angular_z

    def stamp_reference_position(self):
        self.reference_position = copy(self.current_position) 
        return copy(self.current_position) 

# ----------------- Not used ----------------- #

    def publish_waypoint_markers(self, waypoints):
        marker_array = MarkerArray()
        
        for i, (x, y, theta) in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            
            # Create quaternion from theta
            q = quaternion_from_euler(0, 0, theta)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.id = i
            
            marker_array.markers.append(marker)
        
        # Publish the marker array
        self.waypoint_pub.publish(marker_array)
        rospy.loginfo(f"Published {len(waypoints)} waypoint markers") 


