#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import Image, PointCloud2 
from nav_msgs.msg import Odometry
import math
import random
import tf
import numpy as np
import cv2
from cv_bridge import CvBridge

#This class acts as a simple state machine for difference key actions
class RobotStates(): 
    RANDOM_WALKING = 'random_walk'
    OBSTACLE_AVOIDANCE = 'obstacle_avoidance'
    FOLLOW_WALL = 'follow_wall'
    FOLLOW_COLOUR = 'follow_colour' 
    FINISHED = 'finished'

    def __str__(self):
        return self.value

#This is the main class responsible for controlling the robot 
class CameraController(): 
    def __init__(self): 
        self.state = RobotStates.FOLLOW_COLOUR
        self.pose = {'x': 0, 'y': 0, 'theta': 0}
        self.linear = {'x': 0}
        self.angular = {'z': 0}
        self.reference_position = {'x':0, 'y':0, 'theta': 0}
        self.lidar_sensor_data = {'forward': 0, 'left': 0, 'right': 0, 'backward': 0}

        self.p_gain = 0.01 # Proportional gain for steering (i.e Following Colour)
        self.move_speed = 0.1 # How quick the robot should move 
        self.rotate_speed = 0.1 # How quick the robot should turn
        self.obstacle_threshold = 0.4 # Minimum obstacle avoidance distance
        self.wall_threshold = 0.5 # How close a wall should be to follow
        self.wall_follow_distance = 0.3 # How closely it should follow the wall

        self.green_center_x = 0  # The centre of the green camera vision
        self.image_width = 0  # How big (width) the camera output 
        self.colour_detected = False # Is there green on the camera output
        self.is_colour_too_close = False

        self.move_target = 1 # How many M to go in random movement
        self.rotate_target = 0 # How many rads to aim for
        self.is_rotating = False # Are we rotating to rotate_target


        self.is_simulator = False # Sim or Real world (Affects sensor vals)




        # Subscribe to /camera/depth/points topic for camerae depth data (for future use)
        rospy.Subscriber("/camera/depth/image_rect_raw", PointCloud2, self.depth_callback) 
        
        #Subscribes to /camera/rgb/image_raw topic for camera data 
        rospy.Subscriber("camera/color/image_raw", Image, self.camera_callback) 
        
        #Subscribes to /scan topic for laser scan data 
        rospy.Subscriber("scan", LaserScan, self.scan_callback) 
        
        #Subscribes to /odom topics for odometry data 
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        #Publishes to the /cmd_vel topic to control the robot's movement 
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) 

        
        self.bridge = CvBridge() 
        rospy.init_node('CameraController', anonymous=True)

    # Logs the 4 lidar directions values
    def scan_callback(self, msg): 
        # Extracting laser scan data with cone 
        self.lidar_sensor_data['forward'] = self.scan_range_laser(msg, 90, 0)
        self.lidar_sensor_data['left'] = self.scan_range_laser(msg, 90, 90)
        self.lidar_sensor_data['backward'] = self.scan_range_laser(msg, 90, 180)
        self.lidar_sensor_data['right'] = self.scan_range_laser(msg, 90, 270)

    # Process camera messages
    def camera_callback(self, msg): 
        # Between dark green and very bright green
        lightest = np.array([180, 255, 255], dtype="uint8")  
        darkest = np.array( [120, 80, 70], dtype="uint8") 

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        # Log image width for movement calcs
        self.image_width = image.shape[1] 

        # Process image into hsv mask out green values
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, darkest, lightest)
        output_image = cv2.bitwise_and(image, hsv, mask = mask)

        # Make gray for black and what
        gray_output = cv2.cvtColor(output_image, cv2.COLOR_BGR2GRAY)

        # Convert to bw to make counting green easier
        (bw_thresh, bw_output) = cv2.threshold(gray_output, 0, 255, cv2.THRESH_BINARY)



        # max_cnt = 1.0
        # cnt_final = 0
        contours = cv2.findContours(bw_output,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)[-2]
        # for cnt in contours:
        #     if cv2.contourArea(cnt) > max_cnt:
        #         max_cnt = cv2.contourArea(cnt)
        #         cnt_final = cnt

        if contours:
            cnt_final = sorted(contours, key=cv2.contourArea)[-1]

        # Count the ammount of white pixels
        # if its greater than 70% - Too close ?
        white = np.sum(bw_output == 255)
        black = np.sum(bw_output == 0)
        # print((white / (white + black)) * 100,'%')
        if white >= (white + black) * 0.10:
            self.is_colour_too_close = True
        else:
            self.is_colour_too_close = False


        # Calculates spatial moments of the binary image 
        M = cv2.moments(cnt_final) 

        # If any green pixels are detected (any white pixels in bw_output)
        # TODO: Might need to change threshold of a "cluster" (ie, another green object in room)
        if M["m00"] > 0 and cv2.contourArea(cnt_final) > 10: 
            # Get the centre of the green pixel clusters
            self.green_center_x = int(M['m10']/M['m00'])  
            cy = int(M['m01']/M['m00']) # y-coordinate of center 

            # Green is detected
            self.colour_detected = True 

            #Draw circle at center for visualisation 
            cv2.circle(image, (self.green_center_x, cy), 20, (0, 0, 255), -1) 
            
        else: 
            # No green was detected
            self.colour_detected = False 

        # Combine both normal image and green mask for debugging
        bw_output_colored = cv2.cvtColor(bw_output, cv2.COLOR_GRAY2BGR)
        combined_img = np.hstack((image, bw_output_colored))
        

        cv2.namedWindow('Debug', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Debug', 800, 300) 
        cv2.imshow('Debug', combined_img)

        cv2.waitKey(1)

    # Process position for robot 
    def odom_callback(self, msg):
        quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.pose['theta'] = yaw
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y

    # Process point cloud
    def depth_callback(self, msg): 
       print(msg)

    # Distance between last logged position and the robot
    def calculate_distance(self):
        dx = self.pose['x'] - self.reference_position['x']
        dy = self.pose['y'] - self.reference_position['y']
        return math.sqrt(dx**2 + dy**2)

    # Scans a range of laser values (finds minimum between) between angle ---> angle + fov
    def scan_range_laser(self, msg, fov, angle):
        start = (angle - math.floor(fov / 2)) % 360
        end = (angle + math.floor(fov / 2) ) % 360
    
        if start <= end:
            scan_range = msg.ranges[start:end]
        else:
            scan_range = msg.ranges[start:] + msg.ranges[:end]
    
        if self.is_simulator:
            valid_ranges = [r for r in scan_range if not math.isinf(r)]  #Ignore inf
        else:
            valid_ranges = [r for r in scan_range if not r == 0]  #Ignore 0
        return min(valid_ranges) if valid_ranges else float('inf')

    # Log output for robot (Includes sensor info)
    def print_status(self):
            d = self.calculate_distance()
            p = self.pose
            l = self.linear
            a = self.angular
            s = self.lidar_sensor_data
            ss = str(self.state)
            green = self.colour_detected
            print(f"{ss}|"
                f"P:({p['x']:.1f},{p['y']:.1f})|"
                f"θ:{math.degrees(p['theta']):.1f}|"
                f"D:{d:.1f}|"
                f"F:({l['x']:.2f},{a['z']:.2f})|"
                f"S:{s['forward']:.1f},{s['left']:.1f},{s['right']:.1f},{s['backward']:.1f} |"
                f"GRE?:{green}|")
   
    # Normalise rad angle (ie turns more than 420 should wrap to 60 (in rad obvs))
    def normalise_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    # Pick a random dit (random walking)
    def random_direction(self): 
        return random.uniform(0, 2 * math.pi)

    #State actions - Follow colour
    def follow_colour(self): 

        #If no green objects detected, rotate to search 
        if not self.colour_detected: 
            self.linear["x"] = 0 
            self.angular["z"] = self.rotate_speed 
            print("Following colour but cant see any")
            return 
        
        # Distance between the green centre and the camera centre (centre of robot axis)
        error = self.green_center_x - (self.image_width / 2)

        # Calculate steering using proportional control - relative to rotate speed
        self.angular["z"] = -float(error) * self.p_gain  * self.rotate_speed
        # P(ID) Controller 
        # Move forward
        self.linear["x"] = self.move_speed 

    def finished(self):
        self.angular['z'] = 0
        self.linear['x'] = 0

    # Updates robot state
    def update_state(self): 
        # if self.is_colour_too_close:
        #     self.state = RobotStates.FINISHED
        # else:
        #     self.state = RobotStates.FOLLOW_COLOUR

        # Priority: Don't Crash, then finding green, then wall following and then random walking
        if self.is_colour_too_close:
            self.state = RobotStates.FINISHED
        elif self.lidar_sensor_data['forward'] <= self.obstacle_threshold:
            self.state = RobotStates.OBSTACLE_AVOIDANCE
        elif self.colour_detected:
            self.state = RobotStates.FOLLOW_COLOUR
        elif self.lidar_sensor_data['right'] <= self.wall_threshold: 
            self.state = RobotStates.FOLLOW_WALL 
        else:
            self.state = RobotStates.RANDOM_WALKING

    #State actions - Random walk 
    def random_walking(self): 
        if self.is_rotating: # We are already rotating
            diff = self.normalise_angle(self.rotate_target - self.pose['theta'])

            if abs(diff) > 0.01: #  Keep rotating to the angle
                self.angular['z'] = self.rotate_speed if diff > 0 else -self.rotate_speed
            else:
                self.angular['z'] = 0 # Finished rotating
                self.reference_position = {'x': self.pose['x'], 'y': self.pose['y'], 'theta': self.pose['theta']}
                self.is_rotating = False 
        else: # We arent rotating
            if self.calculate_distance() >= self.move_target: # We should be rotating
                self.linear['x'] = 0
                self.is_rotating = True
                self.rotate_target = self.normalise_angle(self.reference_position['theta'] + self.random_direction())
            else: # We should be moving forrward
                self.linear['x'] = self.move_speed
                self.angular['z'] = 0

    #State actions - Obstacle avoidance. 
    def obstacle_avoidance(self):
        self.reference_position = {'x': self.pose['x'], 'y': self.pose['y'], 'theta': self.pose['theta']}
        self.linear['x'] = 0 #Stop moving forward 
        tolerance = 0.2

        #Determine rotation direction 
        #Compares left and right distance of the robot 
        if not self.is_rotating:
            if self.lidar_sensor_data['left'] >= self.lidar_sensor_data['right']:
                #If there is more space to the left, robot rotates counterclockwise
                self.angular['z'] = self.rotate_speed #Positive angular velocity 
            elif self.lidar_sensor_data['right'] > self.lidar_sensor_data['left']:
                #If there is more space to the right, robot rotates clockwise 
                self.angular['z'] = -self.rotate_speed #Negative angular velocity
            print(self.angular['z'])
            self.is_rotating = True

        #Check if path forward is clear and obstacle has been avoided 
        if self.lidar_sensor_data['forward'] > self.obstacle_threshold: 
            self.is_rotating = False
            self.state = RobotStates.RANDOM_WALKING 

    #State actions - Wall following 
    def follow_wall(self): 
        self.reference_position = {'x': self.pose['x'], 'y': self.pose['y'], 'theta': self.pose['theta']}
        #Calculating the difference between current distance to the right wall and desired following distance 
        error = self.lidar_sensor_data['right'] - self.wall_follow_distance 
        self.linear['x'] = self.move_speed 
        #The angular velocity is set proportional to the negative of the error 
        self.angular['z'] = -error * 0.5 #Proportional control/gain 

        # # Corner met - turn into the corner
        # if self.lidar_sensor_data['forward'] == self.lidar_sensor_data['right']: 


        #State transition 
        #If distance to the right wall becomes greater than threshold, means wall has been lost 
        if self.lidar_sensor_data['right'] > self.wall_threshold: 
            self.is_rotating = False
            self.state = RobotStates.RANDOM_WALKING 

    #This method contains the main control loop. 
    #It updates the state, print status information, 
    #and calls appropriate behaviour methods based on current state. 
    def vision(self):
        rate = rospy.Rate(10) #10hz 

        self.reference_position = {'x': self.pose['x'], 'y': self.pose['y'], 'theta': 0}

        while not rospy.is_shutdown():

            # Check if old state is the same
            previous_state = self.state

            self.update_state()

            # Dont allow the same state to lock in rotating (causes undesired behaviour betwen random walk and obstacle avoidance)
            if previous_state != self.state:
                self.is_rotating = False

            # print important info
            self.print_status()

            # select the correct state action
            if self.state == RobotStates.RANDOM_WALKING:
                self.random_walking()
            elif self.state == RobotStates.FOLLOW_WALL:
                self.follow_wall()
            elif self.state == RobotStates.OBSTACLE_AVOIDANCE:
                self.obstacle_avoidance()
            elif self.state == RobotStates.FOLLOW_COLOUR:
                self.follow_colour()
            elif self.state == RobotStates.FINISHED:
                self.finished()
            move_cmd = Twist()  
            move_cmd.linear.x = self.linear['x']
            move_cmd.angular.z = self.angular['z']

            self.pub.publish(move_cmd)
            
            rate.sleep() 

if __name__ == '__main__':
    try:
        controller = CameraController() 
        controller.vision() 
    except rospy.ROSInterruptException:
        pass 