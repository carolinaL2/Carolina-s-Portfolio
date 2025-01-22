#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf

class Robot():
    def __init__(self):
        self.pose = {'x': 0, 'y': 0, 'theta': 0}
        self.linear = {'x': 0}
        self.angular = {'z': 0}
        self.start_position = {'x':0, 'y':0, 'theta': 0}

    def odom_callback(self, msg):
        quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.pose['theta'] = yaw
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y

    def calculate_distance(self):
        dx = self.pose['x'] - self.start_position['x']
        dy = self.pose['y'] - self.start_position['y']
        return math.sqrt(dx**2 + dy**2)

    def print_status(self, state):
        distance = self.calculate_distance()
        print(f"Robot: {state:^8} | Pos: ({self.pose['x']:6.2f}, {self.pose['y']:6.2f}) | θ: {math.degrees(self.pose['theta']):6.2f}° | Dist: {distance:6.2f}m | Force: (Lin: {self.linear['x']:4.2f}, Ang: {self.angular['z']:4.2f})")


    def normalise_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def mover(self):
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        

        move_cmd = Twist()
        move_speed = 0.05 # Slow speed
        turning_angle = math.pi/2
        turning_speed = 0.1
        state = "start"
        square_size = 1


        while not rospy.is_shutdown():
            self.print_status(state)
            
            # Robot has just started
            if state == "start":
                self.start_position = {'x': self.pose['x'], 'y': self.pose['y'], 'theta': 0}
                state = "forward"
            
            # Robot is moving forward
            if state == "forward":
                self.linear['x'] = move_speed

        
                # Has the robot moved enough
                if self.calculate_distance() >= square_size:
                    self.linear['x'] = 0
                    self.start_position = {'x': self.pose['x'], 'y': self.pose['y'], 'theta': self.pose['theta']}
                    state = "turning"
                
                # Detect 0.05 rad drift degree drift
                if self.pose['theta'] >= math.pi / 200: #0.015 rad
                    print("Drift detected -")

            
            # Robot is turning
            if state == "turning":
                target = self.normalise_angle(self.start_position['theta'] + turning_angle)
                diff = self.normalise_angle(target - self.pose['theta'])
                print(math.degrees(target), diff)
                if abs(diff) > 0.01: 
                   # make correction
                    self.angular['z'] = turning_speed if diff > 0 else -turning_speed
                else:
                    self.start_position['theta'] = target
                    self.angular['z'] = 0
                    state = "forward"

            move_cmd.linear.x = self.linear['x']
            move_cmd.angular.z = self.angular['z']
            pub.publish(move_cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        robo = Robot()
        robo.mover()
    except rospy.ROSInterruptException:
        pass



# 1. Initialise current `pose(x,y,theta)` of the robot and select a desired `pose(x*,y*,theta*)`. The `x*` and `y*` should be set as the first corner coordinates of the 1m-edge square you will traverse. `theta*` should be current `theta` plus 90 degrees
# 2. Move forward and accumulate the travelled distance. 
# 3. Stop if the total distance is greater than 1m. This is the end of your first edge.
# 4. Now set the desired orientation as `theta* = theta*+PI/2`
# 5. Rotate in place (remember the `Twist` message) until the accumulated orientation is greater than `PI/2`
# 6. Pause
# 7. Repeat until the square is complete.