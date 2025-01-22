import rospy 


class PIDController():
    """
    PIDController - A simple PID controller implementation with time-based error tracking
    """

    def __init__(self, kp, ki, kd): 
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Error tracking 
        self.error_integral = 0 
        self.last_error = 0 
        self.last_time = None 


    def calculate(self, error):  
        """
        Calculate - Calculate the PID output value based on the error value

        :param error: The error value to calculate the PID output from
        :return: The PID output value
        """

        # Initialize last_time on first call
        if self.last_time is None:
            self.last_time = rospy.get_time()
            return 0 # Return 0 on first call since we can't calculate dt yet  
        
        # Calculate the time difference
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        
        # Prevent division by zero 
        if dt == 0:
            return 0
        
        # Calculate integral and derivative terms  
        self.error_integral += error * dt
        error_derivative = (error - self.last_error) / dt
        
        # Calculate PID output 
        output = (self.kp * error +
                 self.ki * self.error_integral +
                 self.kd * error_derivative)
        
        # Update last values
        self.last_error = error
        self.last_time = current_time
        
        return output 