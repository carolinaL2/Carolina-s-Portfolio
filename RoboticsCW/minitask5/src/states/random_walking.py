from states.abstract_state import AbstractState

class RandomWalkingState(AbstractState):
    """
    [REPLACED WITH PATROL]
    RandomWalkingState - Moves a given distance and rotates randomly 
    """
    def __init__(self, robot):
        super().__init__(robot)
        self.move_target = 1 
        self.is_rotating = False 
        self.rotate_target = 0
        self.movement = self.robot.movement_controller

    def execute(self):
        """
        execute - Moves a given distance and rotates randomly
        """
        if self.is_rotating: # We are already rotating
            diff = self.movement.normalise_angle(self.rotate_target - self.robot.movement_controller.get_current_position()['theta'])
            if abs(diff) > 0.01: #  Keep rotating to the angle
                self.movement.set_angular(self.movement.rotate_speed if diff > 0 else -self.movement.rotate_speed)
            else:
                self.movement.set_angular(0)
                self.movement.stamp_reference_position()
                self.is_rotating = False 
        else: # We arent rotating
            if self.movement.calculate_distance() >= self.move_target: # We should be rotating
                self.movement.set_linear(0)
                self.is_rotating = True
                self.rotate_target = self.movement.normalise_angle(self.movement.get_reference_position()['theta'] + self.movement.random_direction()) 
            else: # We should be moving forrward
                self.movement.set_linear(self.movement.move_speed) 
                self.movement.set_angular(0) 

        self.movement.publish_movement() 
# 