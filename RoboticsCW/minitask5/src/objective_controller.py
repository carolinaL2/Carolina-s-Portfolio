import uuid
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose
from std_msgs.msg import Header, ColorRGBA


class ObjectiveTracker():
    """
    ObjectiveTracker - Class to track objects found in the environment
    """

    def __init__(self, robot):
        self.robot = robot
        self.objects_found = {}
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)

    def track_object(self, object_name, x, y):
        """
        Track object - Registers an object found in the environment

        :param object_name: Name of the object
        :param x: x coordinate
        :param y: y coordinate
        """


        if object_name not in self.objects_found:
            self.objects_found[object_name] = set()
        
        object_id = object_name + '-' + str(uuid.uuid4())
        dup = self.duplicate_check(object_name, x, y)
        if dup is None:
            self.objects_found[object_name].add((object_id, x, y))

            if(object_name == "blue"):
                self.show_avoidance_marker(x, y, object_id)
            else:
                self.show_marker(object_name, x, y, object_id)
            print(f"Object {object_name} number {len(self.objects_found[object_name])} found at ({x},{y})")

        else:
            if self.robot.debug_level > 0:
                print(f"Object {object_name} already found at ({x},{y}) with id {dup}")

    def duplicate_check(self, object_name, x, y):
        """
        Duplicate check - check if the object is already found at the same location (aprox)

        :param object_name: Name of the object
        :param x: x coordinate
        :param y: y coordinate

        :return: None if not found, object_id if found
        """


        # Green blocks are 0.5 x 0.5 meters - so extra 0.1 around that (centre)
        tolerance = 0.75

        if object_name not in self.objects_found:
            return None

        # print(self.objects_found[object_name])
        # print(x,y)

        for obj in self.objects_found[object_name]:
            if abs(obj[1] - x) < tolerance and abs(obj[2] - y) < tolerance:
                return obj[0]
        
        return None

    def show_marker(self, object_name, x, y, object_id):
        """
        Show marker - Show a text marker at the location of the object

        :param object_name: Name of the object
        :param x: x coordinate
        :param y: y coordinate
        :param object_id: Unique id of the object
        """

        marker = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=hash(object_id) % 10000,  
            pose=Pose(Point(x, y, 0.5), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.3, 0.3, 0.3),
            header=Header(frame_id='map'),
            color=ColorRGBA(0.0, 1.0, 0.0, 1),
            text=object_name
        )
        self.marker_publisher.publish(marker)


# ----------------- Not used ----------------- #
    def show_avoidance_marker(self, x, y, id):
        """
        Show avoidance marker - Show a red cylinder marker at the location of the object
        """

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.id = hash(id) % 10000
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        
        self.marker_publisher.publish(marker)
