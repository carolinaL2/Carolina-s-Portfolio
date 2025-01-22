from .abstract_state import AbstractState
import math
from visualization_msgs.msg import Marker
import rospy
from pid_controller import PIDController


class ObjectDectionState(AbstractState):
    """
    ObjectDetectionState - State for detecting and travelling to objects in the environment
    """

    def __init__(self, robot):
        super().__init__(robot)
        self.sensor = self.robot.sensor_controller
        self.movement = self.robot.movement_controller
        self.standing_distance = 0.6
        self.safe_blue_distance = 0.5
        self.pid = PIDController(kp = 0.001, ki = 0.0, kd = 0.0)


    def get_front_distance(self, x, y,):
        return self.sensor.get_point_depth(x, y)
        
    def check_if_registered(self, object_name, distance, angle):
        """
        check_if_registered - Check if the object has already been registered in the objective controller

        :param object_name: identifier for the object
        :param distance: Distance from the robot to the object
        :param angle: Angle of the object from the robot

        :return: True if the object has already been registered, False otherwise
        """
        real_x = distance * math.cos(angle) + self.movement.get_current_position()['x']
        real_y = distance * math.sin(angle) + self.movement.get_current_position()['y']
        return self.robot.objective_controller.duplicate_check(object_name, real_x, real_y) is not None

    def register_object(self, object_name, distance, angle ):
        """
        register_object - Register the object in the objective controller

        :param object_name: identifier for the object
        :param distance: Distance from the robot to the object
        :param angle: Angle of the object from the robot
        """
        real_x = distance * math.cos(angle) + self.movement.get_current_position()['x']
        real_y = distance * math.sin(angle) + self.movement.get_current_position()['y']
        self.robot.objective_controller.track_object(object_name, real_x, real_y)

    def execute(self):
        """
        execute - Identifies objects and uses PID controller to move towards them for registration
        """

        # Get the biggest object detected by the sensor
        self.target_object = self.sensor.get_biggest_object_detected()

        # Only follow if an object is detected
        if(self.target_object is not None):
            

            colour, object_x, object_y, object_w, object_h, visited = self.target_object

            # Calculate the centre of the object
            centre_x = int(object_x + object_w / 2)
            centre_y = int(object_y + object_h / 2)
            
            # Get the distance to the object (approximate from depth image)
            point_depth = self.get_front_distance(centre_x, centre_y)

            # ignore if its been visited before
            if visited:
                self.movement.set_linear(0)
                self.movement.rotate_to(self.movement.get_current_position()['theta'] + math.pi)
                return

            # For blue tiles - Not currently in use
            if (point_depth and point_depth < self.safe_blue_distance and colour.lower() == "blue"):
                self.movement.set_linear(0)
                self.movement.set_angular(0)
                self.movement.publish_movement()

            
                sensor_height = 0.15

                # Sensor is lifted off the ground
                theta = math.asin(sensor_height / point_depth)
                altered_distance = point_depth * math.cos(theta)

                #  bit o trig
                offset_x = altered_distance * math.cos(self.movement.get_current_position()['theta']) 
                offset_y = altered_distance * math.sin(self.movement.get_current_position()['theta']) 

                real_x = offset_x + self.movement.get_current_position()['x']
                real_y = offset_y + self.movement.get_current_position()['y']

                # Check if the object has already been registered
                if(self.check_if_registered(colour, point_depth, self.movement.get_current_position()['theta'])):
                    # print("Object already registered - ignore")
                    return
                else:
                    self.robot.objective_controller.track_object(colour, real_x, real_y)
                    print("Blue object detected at: ", point_depth )
                return

            # For any other objects
            if (point_depth and point_depth < self.standing_distance):
                # Stop the robot
                self.movement.set_linear(0)
                self.movement.set_angular(0)
                self.movement.publish_movement()

                # Desired stop behaviour
                rospy.sleep(2)

                # Register the object
                self.register_object(colour, point_depth, self.movement.get_current_position()['theta'])
                self.movement.rotate_to(self.movement.get_current_position()['theta'] + math.pi)
                return
    
            # Get the width of the camera image
            image_width = self.sensor.get_camera_width()

            # Distance between the green centre and the camera centre (centre of robot axis)c
            error = centre_x - (image_width / 2) 
            

            # Calculate steering using PID controller
            pid_output = self.pid.calculate(-error) 

            # Limit the PID output to a maximum of 0.5 (to prevent the robot from spinning too fast)
            pid_output = max(min(pid_output, 0.5), -0.5)

            # Set the movement based on the PID output    
            self.movement.set_movement(self.movement.move_speed, pid_output)
            self.movement.publish_movement()
       


"""
Ziegler-Nichols method for tuning PID controller

kp = 0.001

- p gain worked fine


"""

