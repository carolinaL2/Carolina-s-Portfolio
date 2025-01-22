#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry
import math
import random
import tf
from enum import Enum, auto

#This class acts as a simple state machine with three states 
class RobotStates():
    RANDOM_WALKING = 'random_walk'
    OBSTACLE_AVOIDANCE = 'obstacle_avoidance'
    FOLLOW_WALL = 'follow_wall'

    def __str__(self):
        return self.value

#This is the main class responsible for controlling the robot 
class LidarController(): 
    def __init__(self): 
        self.state = RobotStates.RANDOM_WALKING
        self.pose = {'x': 0, 'y': 0, 'theta': 0}
        self.linear = {'x': 0}
        self.angular = {'z': 0}
        self.reference_position = {'x':0, 'y':0, 'theta': 0}
        self.sensor_data = {'forward': 0, 'left': 0, 'right': 0, 'backward': 0}

        self.move_speed = 0.1
        self.rotate_speed = 0.1 
        self.obstacle_threshold = 0.4 #checks for obstacles within 0.3m in front of the robot 
        self.wall_threshold = 0.5 
        self.wall_follow_distance = 0.3 

        self.move_target = 1
        self.rotate_target = 0
        self.is_rotating = False
        self.is_simulator = False

        #Publishes to the /cmd_vel topic to control the robot's movement 
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        #Subscribes to /scan topic for laser scan data 
        rospy.Subscriber("scan", LaserScan, self.scan_callback) 
        #Subscribes to /odom topics for odometry data 
        rospy.Subscriber("odom", Odometry, self.odom_callback)
    
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

    #This method processes the laser scan data, dividing it into 4 directions 
    def scan_callback(self, msg): 
        # Extracting laser scan data with cone 
        self.sensor_data['forward'] = self.scan_range_laser(msg, 90, 0)
        self.sensor_data['left'] = self.scan_range_laser(msg, 90, 90)
        self.sensor_data['backward'] = self.scan_range_laser(msg, 90, 180)
        self.sensor_data['right'] = self.scan_range_laser(msg, 90, 270)

    def calculate_distance(self):
        dx = self.pose['x'] - self.reference_position['x']
        dy = self.pose['y'] - self.reference_position['y']
        return math.sqrt(dx**2 + dy**2)
    
    def print_status(self):
            d = self.calculate_distance()
            p = self.pose
            l = self.linear
            a = self.angular
            s = self.sensor_data
            ss = str(self.state)
            print(f"{ss}|"
                f"P:({p['x']:.1f},{p['y']:.1f})|"
                f"θ:{math.degrees(p['theta']):.1f}|"
                f"D:{d:.1f}|"
                f"F:({l['x']:.2f},{a['z']:.2f})|"
                f"S:{s['forward']:.1f},{s['left']:.1f},{s['right']:.1f},{s['backward']:.1f}")

    def odom_callback(self, msg):
        quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.pose['theta'] = yaw
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y

    def normalise_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def random_direction(self): 
        return random.uniform(0, 2 * math.pi)

    #THis method determines which state the robot should be in 
    def update_state(self): 
        #Checks for obstacles in front of the robot to switch states 
        if self.sensor_data['forward'] <= self.obstacle_threshold:
            self.state = RobotStates.OBSTACLE_AVOIDANCE 
        elif self.sensor_data['right'] <= self.wall_threshold: 
            self.state = RobotStates.FOLLOW_WALL 
        else:
            self.state = RobotStates.RANDOM_WALKING

    #State actions - Random walk 
    #Moves the robot forward for a set distance (move_target), 
    #then it rotates to a random direction. 
    #This method loops if no obstacle are detected. 
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
            if self.sensor_data['left'] >= self.sensor_data['right']:
                #If there is more space to the left, robot rotates counterclockwise
                self.angular['z'] = self.rotate_speed #Positive angular velocity 
            elif self.sensor_data['right'] > self.sensor_data['left']:
                #If there is more space to the right, robot rotates clockwise 
                self.angular['z'] = -self.rotate_speed #Negative angular velocity
            print(self.angular['z'])
            self.is_rotating = True

        #Check if path forward is clear and obstacle has been avoided 
        if self.sensor_data['forward'] > self.obstacle_threshold: 
            self.is_rotating = False
            self.state = RobotStates.RANDOM_WALKING 


    #State actions - Wall following 
    def follow_wall(self): 
        self.reference_position = {'x': self.pose['x'], 'y': self.pose['y'], 'theta': self.pose['theta']}
        #Calculating the difference between current distance to the right wall and desired following distance 
        error = self.sensor_data['right'] - self.wall_follow_distance 
        self.linear['x'] = self.move_speed 
        #The angular velocity is set proportional to the negative of the error 
        self.angular['z'] = -error * 0.5 #Proportional control/gain 

        # # Corner met - turn into the corner
        # if self.sensor_data['forward'] == self.sensor_data['right']: 


        #State transition 
        #If distance to the right wall becomes greater than threshold, means wall has been lost 
        if self.sensor_data['right'] > self.wall_threshold: 
            self.is_rotating = False
            self.state = RobotStates.RANDOM_WALKING 

    #This method contains the main control loop. 
    #It updates the state, print status information, 
    #and calls appropriate behaviour methods based on current state. 
    def mover(self):
        rospy.init_node('LidarController', anonymous=True)
        rate = rospy.Rate(10) #10hz 

        self.reference_position = {'x': self.pose['x'], 'y': self.pose['y'], 'theta': 0}

        while not rospy.is_shutdown():


            # Check if old state is the same
            previous_state = self.state

            self.update_state()

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

            move_cmd = Twist()  
            move_cmd.linear.x = self.linear['x']
            move_cmd.angular.z = self.angular['z']

            self.pub.publish(move_cmd)
            
            rate.sleep() 

if __name__ == '__main__':
    try:
        controller = LidarController() 
        controller.mover() 
    except rospy.ROSInterruptException:
        pass 