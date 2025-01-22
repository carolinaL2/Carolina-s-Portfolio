from .abstract_state import AbstractState 
from pid_controller import PIDController

class FollowWallState(AbstractState):
    """
    FollowWallState - State where robot follows a wall at a fixed distance
    """

    def __init__(self, robot):
        super().__init__(robot)
        self.wall_threshold = 0.5 # How close a wall should be to follow
        self.wall_follow_distance = 0.4 # How closely it should follow the wall 
        self.movement = self.robot.movement_controller 
        self.sensor = self.robot.sensor_controller 
        self.pid = PIDController(kp=0.3, ki=0.0036, kd=6.19)

    def is_robot_too_close(self): 
        """
        Check if robot is too close to the wall (within our threshold)
        """
        return self.sensor.get_lidar_data()['right'] <= self.wall_threshold

    def execute(self):  
        """
        execute - Calculates PID output for angular vel based on error in distance to wall
        """
        # Get current distance to right wall
        current_distance = self.sensor.get_lidar_data()['right'] 
        
        # Calculate error (positive when too far, negative when too close)
        error = current_distance - self.wall_follow_distance 

        # Calculate PID output
        pid_output = self.pid.calculate(error)

        # Limit PID output to be within -0.3 to 0.3 (turns around occasionally)
        pid_output = min(max(pid_output, -0.3),0.3)


        self.movement.set_movement(self.movement.move_speed, -pid_output) 
        self.movement.publish_movement() 


"""
Ziegler-Nichols method for tuning PID controller

kc = 0.5 (stable oscillation)
Pc = 150 (period of oscillation)

kp = 0.6 * kc = 0.3
ki = kp / (2 * Pc) = 0.0036
kd = kp * Pc / 8 = 6.19

"""