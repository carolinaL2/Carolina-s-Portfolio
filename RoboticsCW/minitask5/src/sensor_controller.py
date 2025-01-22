import rospy
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid , Path
import numpy as np
import cv2
from cv_bridge import CvBridge 
import math
import random
import tf

class SensorController():
    """
    SensorController - Class to handle sensor data from the robot
    """

    def __init__(self, robot):
        self.robot = robot
        self.lidar_data = {'forward': 0, 'left': 0, 'right': 0, 'backward': 0, 'forward_narrow': 0}
        self.depth_data = None
        self.lidar_data_raw = None
        self.depth_image = None
        self.camera_width = 0
        self.available_nav_paths = 1000


        self.object_size_threshold = 100000

        self.detected_objects = []
        self.bridge = CvBridge()

        self.color_ranges = {
            'green_block': (np.array([35, 30, 30], dtype="uint8"), np.array([85, 255, 255], dtype="uint8")),
            'red': (np.array([0, 160, 25], dtype="uint8"), np.array([5, 255, 255], dtype="uint8")),
            'blue': (np.array([100, 100, 30], dtype="uint8"), np.array([140, 255, 255], dtype="uint8"))
        }


        # Subscribe to sensor topics
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_callback)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback) 
        rospy.Subscriber('map', OccupancyGrid, self.map_callback) 
        rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.planner_callback, queue_size=10)

        self.bad_hits_camera = 0
        self.objective_controller = self.robot.objective_controller

    def merge_nearby_objects(self, contours):
        """
        merge_nearby_objects - Merge contours in close proximity to each other (combines multiple shades of the hydrant)

        :param contours: The contours to merge

        :return: A new list of contours
        """
        # Check if any contours intersect
        for i in range(len(contours)):
            for j in range(i+1, len(contours)):

                # If they intersect, merge them (remove old space)
                if self.intersect(contours[i], contours[j]):
                    contours[i] = self.merge(contours[i], contours[j])
                    contours[j] = None
        
        return [c for c in contours if c is not None]

    def merge(self, contour1, contour2):
        """
        merge - Merge two contours into one by maxing the bounding box

        :param contour1: The first contour
        :param contour2: The second contour

        :return: A new contour that is the combination of the two
        """
        x1, y1, w1, h1 = cv2.boundingRect(contour1)
        x2, y2, w2, h2 = cv2.boundingRect(contour2)
        x = min(x1, x2)
        y = min(y1, y2)
        w = max(x1 + w1, x2 + w2) - x
        h = max(y1 + h1, y2 + h2) - y
        return np.array([[x, y], [x+w, y], [x+w, y+h], [x , y+h]])

    def intersect(self, contour1, contour2):
        """
        intersect - Check if two contours intersect using bounding boxes

        :param contour1: The first contour
        :param contour2: The second contour

        :return: If they intersect true, otherwise false
        """
        x1, y1, w1, h1 = cv2.boundingRect(contour1)
        x2, y2, w2, h2 = cv2.boundingRect(contour2)
        return x1 < x2 + w2 and x1 + w1 > x2 and y1 < y2 + h2 and y1 + h1 > y2          

    def scan_range_laser(self, msg, fov, angle):
        """
        scan_range_laser - Get the range of the laser scan in a cone

        :param msg: The laser scan message
        :param fov: The field of view of the cone
        :param angle: The angle of the cone

        :return: The range of the laser scan in the cone
        """
        start = (angle - math.floor(fov / 2)) % 360
        end = (angle + math.floor(fov / 2) ) % 360
    
        if start <= end:
            scan_range = msg.ranges[start:end]
        else:
            scan_range = msg.ranges[start:] + msg.ranges[:end]
    
        if self.robot.is_simulator:
            valid_ranges = [r for r in scan_range if not math.isinf(r)]  #Ignore inf
        else:
            valid_ranges = [r for r in scan_range if not r == 0]  #Ignore 0
        return min(valid_ranges) if valid_ranges else float('inf')
# ----------------- Callback methods ----------------- #
   
    def planner_callback(self, msg):
        """
        planner_callback - Callback for the path planner topic subscription
        Sets flag if there are available navigation paths

        :param msg: The topic msg
        """
        self.available_nav_paths = len(msg.poses) == 0 if hasattr(msg, 'poses') else None

    def odom_callback(self, msg):
        """
        odom_callback - Callback for the odometry topic subscription
        Sets the robot's position based on the odometry message

        :param msg: The topic msg
        """
        quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)


        self.robot.movement_controller.set_position(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
           yaw
        )

    def map_callback(self, msg): 
        """
        map_callback - Callback for the map topic subscription

        :param msg: The topic msg
        """
        self.map_data = msg.data 
        self.map_info = msg.info

    def depth_callback(self, msg):
        """
        depth_callback - Callback for the depth image topic subscription
        Converts the depth image to a numpy array for easier indexing

        :param msg: The topic msg
        """
        try:
            self.depth_data = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            cv_image_array = np.array(cv_image, dtype= np.dtype('f8'))
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
            self.depth_image = cv_image_norm

        except Exception as e:
            rospy.logerr(f"Error converting depth image: {str(e)}")
            self.depth_data = None
            self.depth_image = None

    def scan_callback(self, msg): 
        """
        scan_callback - Callback for the laser scan topic subscription
        Extracts the laser scan data in different directions

        :param msg: The topic msg
        """
        self.lidar_data_raw = msg.ranges
        
        # Extracting laser scan data with cone 
        self.lidar_data['forward'] = self.scan_range_laser(msg, 90, 0)
        self.lidar_data['left'] = self.scan_range_laser(msg, 90, 90)
        self.lidar_data['backward'] = self.scan_range_laser(msg, 90, 180)
        self.lidar_data['right'] = self.scan_range_laser(msg, 90, 270)
        self.lidar_data['forward_narrow'] = self.scan_range_laser(msg, 30, 0)

    def camera_callback(self, msg):
        """
        camera_callback - Callback for the camera topic subscription
        Detects objects in the camera image based on colour and logs them
        
        :param msg: The topic msg
        """

        # Convert image to opencv format
        self.detected_objects = []
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Get camera width
        self.camera_width = image.shape[1]

        # Create kernel for dilation
        kernel = np.ones((7,7),np.uint8)


        for colour, (darkest, lightest) in self.color_ranges.items():

            # Mask the image to get only the colour
            mask = cv2.inRange(hsv, darkest, lightest)
            mask = cv2.dilate(mask, kernel)
            contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        

            #  Filter out blue -- not working yet
            if colour == 'blue':
                continue

            # Filter out small contours
            contours = [c for c in contours if cv2.contourArea(c) > self.object_size_threshold]
            
            #  Print all contour areas
            # print([cv2.contourArea(c) for c in contours])

            # Merge nearby objects
            merged_contours = self.merge_nearby_objects(contours)

            # Detect each colour object
            for contour in merged_contours:
                # Draw countour on image
                # cv2.drawContours(image, [contour], -1, (random.randint(0,255), random.randint(0,255), random.randint(0,255)), 3)
                x, y, w, h = cv2.boundingRect(contour)

                # Get centre of contour
                cv2.circle(image, (int(x + w / 2), int(y + h / 2)), 5, (0, 0, 255), -1)

                # Check if object has been visited before 
                visited = self.is_object_visited(colour, int(x + w / 2), int(y + h / 2))
                
                # Ignore previously visited objects
                if(visited):
                    cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                else:
                    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    self.detected_objects.append((colour, x, y, w, h, visited))

        # CV2 preview for debugging
        smaller = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
        cv2.imshow('image', smaller)
        
        cv2.waitKey(1)

        # Handles laggy bug by counting bad hits
        if(self.detected_objects == []):
            self.bad_hits_camera += 1
        else:
            self.bad_hits_camera = 0

# ----------------- Getter and Setter methods ----------------- #

    def get_lidar_data(self):
        return self.lidar_data
    
    def get_detected_objects(self):
        return self.detected_objects 
    
    def get_map_info(self): 
        """
        Returns map metadata including dimensions, resolution and origin.
        Returns None if map data hasn't been received yet.
        """
        if self.map_data is None:
            return None
            
        return type('MapInfo', (), {
            'width': self.map_width,
            'height': self.map_height,
            'resolution': self.map_resolution,
            'origin': self.map_origin,
            'data': self.map_data
        }) 

    def get_camera_width(self):
        return self.camera_width

    def get_raw_lidar_data(self):
        return self.lidar_data_raw

    def get_biggest_object_detected(self):
        # Removes blues until we have a better way to detect them
        if self.detected_objects:
            return max(self.detected_objects, key=lambda x: x[3] * x[4] if x[0] != 'blue' else float('-inf'))
            
        return None
        
    def is_object_detected(self):
            return self.detected_objects != [] or self.bad_hits_camera < 10
    
    def is_object_visited(self, colour, image_x, image_y):
        """
        is_object_visited - Check if an object has been visited before based on aprox coords

        :param colour: The colour of the object
        :param image_x: The x position of the object in the image
        :param image_y: The y position of the object in the image
        """

        point_depth = self.get_point_depth(image_x, image_y)

        if point_depth is None:
            return False
        
        robot_pos = self.robot.movement_controller.get_current_position()

        # Aproximage real coords
        real_x = point_depth * math.cos(self.robot.movement_controller.get_current_position()['theta']) + robot_pos['x']
        real_y = point_depth * math.sin(self.robot.movement_controller.get_current_position()['theta']) + robot_pos['y']

        return self.objective_controller.duplicate_check(colour, real_x, real_y) is not None

    def get_point_depth(self, x, y):
        if self.depth_data is not None and x < self.depth_data.shape[1] and y < self.depth_data.shape[0]:
            return self.depth_data[y, x]
        return None
    
# ----------------- Not used ----------------- #

    def get_region_depth(self, x, y, w, h):
        # Return closest point in region
        if self.depth_data is not None:
            region =  self.depth_data[y:y+h, x:x+w]
          
            valid_depths = region[~np.isnan(region) & ~np.isinf(region)]
            
            # cropped = self.depth_image[y:y+h, x:x+w]
            # cv2.imshow('depth', cropped)
            # cv2.waitKey(1)

            if len(valid_depths) > 0:
                return np.min(valid_depths)
                
        return None
