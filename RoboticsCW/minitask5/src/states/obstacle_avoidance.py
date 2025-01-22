from .abstract_state import AbstractState
import rospy
import math

class ObstacleAvoidanceState(AbstractState):
    """
    ObstacleAvoidanceState - Stops the robot if it's within a threshold of an obstacle and rotates accordingly
    """
    def __init__(self, robot):
        super().__init__(robot)
        self.is_rotating = False 
        self.movement = self.robot.movement_controller 
        self.sensor = self.robot.sensor_controller 
        self.obstacle_threshold = 0.45
        self.history = []


    def update_history(self,direction):
        """
        update_history - Stores the previous direction of the robot in a list
        """
        self.history.append(direction)
        if len(self.history) > 10:
            self.history.pop(0)

    def is_stuck(self):
        """
        is_stuck - Checks if the robot is stuck in a loop (bug we found - quality fix)

        :return: True if the robot is stuck in a loop, False otherwise
        """
        if len(self.history) < 10:
            return False

        # Checks alternation (left, right, left, right, etc) 
        for i in range(1, len(self.history)):
            if self.history[i] == self.history[i-1]:
                return False
            
        return True
        
       
    def execute(self):
        """
        execute - Stops the robot if it's within a threshold of an obstacle and rotates accordingly
        """

        # Stop the robot
        self.movement.set_linear(0) 

        # Get side distances
        left_distance = self.sensor.get_lidar_data()['left']
        right_distance = self.sensor.get_lidar_data()['right']
        
        # Fixes bug where robot gets stuck in a loop
        if self.is_stuck(): 
            self.movement.rotate_to(self.movement.get_current_position()['theta'] + math.pi)
            rospy.logwarn("Robot is stuck in a loop - rotating 180 degrees")
            self.history.clear()
            return

        diff = left_distance - right_distance
        
        if abs(diff) < 0.2:
            # Dont't allow the robot to move to little changes
            last_direction = self.history[-1] if self.history else 'left'
            self.update_history(last_direction)
            self.movement.set_angular(self.movement.rotate_speed if last_direction == 'left' else -self.movement.rotate_speed)
        else: 
            # Turn to the side with the most space
            direction = 'left' if diff > 0 else 'right'
            self.update_history(direction)
            self.movement.set_angular(self.movement.rotate_speed if direction == 'left' else -self.movement.rotate_speed)



        self.movement.publish_movement()

    

