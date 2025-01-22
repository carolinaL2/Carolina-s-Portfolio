class RobotStates():
    """
    Robot states enum
    """

    RANDOM_WALKING = 'random_walk'
    OBSTACLE_AVOIDANCE = 'obstacle_avoidance' 
    PATROL = 'patrol' 
    FOLLOW_WALL = 'follow_wall'
    OBJECT_DETECT = 'object_detect' 
    FINISHED = 'finished'

    def __str__(self):
        return self.value
