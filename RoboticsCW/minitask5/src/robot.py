#!/usr/bin/env python3

import rospy
import math
from movement_controller import MovementController
from sensor_controller import SensorController
from pid_controller import PIDController
from objective_controller import ObjectiveTracker
from states.random_walking import RandomWalkingState
from states.object_detection import ObjectDectionState
from states.follow_wall import FollowWallState
from states.obstacle_avoidance import ObstacleAvoidanceState
from states.patrol_state import PatrolState
from states.abstract_state import AbstractState
from states_enum import RobotStates
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_srvs.srv import Empty

class Robot():
    """
    Robot - The main robot controller class that manages the robot's state and behaviour
    """

    def __init__(self):
        self.is_simulator = True
        self.objective_controller = ObjectiveTracker(self)
        self.movement_controller = MovementController(self)
        self.sensor_controller = SensorController(self)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.states = {
            RobotStates.RANDOM_WALKING: RandomWalkingState(self),
            RobotStates.OBJECT_DETECT: ObjectDectionState(self),
            RobotStates.FOLLOW_WALL: FollowWallState(self),
            RobotStates.OBSTACLE_AVOIDANCE: ObstacleAvoidanceState(self),
            RobotStates.PATROL: PatrolState(self)
        }

        self.debug_level = 0

        self.curent_state = None

        rospy.init_node('RobotController', anonymous=True)

        # Wait for the move_base action server to come up        
        self.client.wait_for_server(rospy.Duration(10))
        if not self.client.wait_for_server(rospy.Duration(10)):
            rospy.logerr("Failed to connect to move_base server within 10 seconds.")


        # Clear the costmap from being cluttered on intitialisation 
        clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        try:
            clear_costmap()
        except rospy.ServiceException as e:
            rospy.logerr("Failed to clear costmap: %s" % e)

    def print_status(self):
        """
        print_status - Print the current robot status to the console
        """

        d = self.movement_controller.calculate_distance()
        p = self.movement_controller.get_current_position()
        l = self.movement_controller.get_linear()
        a = self.movement_controller.get_angular()
        s = self.sensor_controller.get_lidar_data()
        ss = str(self.curent_state)
        # green = self.colour_detected
        print(f"{ss}|"
            f"P:({p['x']:.1f},{p['y']:.1f})|"
            f"θ:{math.degrees(p['theta']):.1f}|"
            f"D:{d:.1f}|"
            f"F:({l['x']:.2f},{a['z']:.2f})|"
            f"S:{s['forward']:.1f},{s['left']:.1f},{s['right']:.1f},{s['backward']:.1f} |")
            # f"GRE?:{green}|")

    def update_state(self): 
        """
        update_state - Update the robot state machine based on sensor data
        """

        last_state = self.curent_state


        if self.sensor_controller.get_lidar_data()['forward'] <= self.states[RobotStates.OBSTACLE_AVOIDANCE].obstacle_threshold:
            self.curent_state = self.states[RobotStates.OBSTACLE_AVOIDANCE]
        elif self.sensor_controller.is_object_detected():
            self.curent_state = self.states[RobotStates.OBJECT_DETECT]
        elif self.states[RobotStates.FOLLOW_WALL].is_robot_too_close():
            self.curent_state = self.states[RobotStates.FOLLOW_WALL]
        else:
            self.curent_state = self.states[RobotStates.PATROL] 

        if last_state != self.curent_state:
            if self.debug_level == 0:
                self.print_status()

    def run(self):
        """
        run - The main robot control loop
        """

        rate = rospy.Rate(10) #10hz  
        
        while not rospy.is_shutdown():

            self.update_state()

            if self.debug_level > 0:
                self.print_status()

            if (self.curent_state != self.states[RobotStates.RANDOM_WALKING]):
                self.movement_controller.stamp_reference_position()

            self.curent_state.execute()
            self.movement_controller.publish_movement()
            rate.sleep() 

