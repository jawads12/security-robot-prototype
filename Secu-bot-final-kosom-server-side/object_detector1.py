#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32  # Import Int32 message type

def scan_callback(msg):
    # Define angle ranges in degrees
    angle_ranges_deg = {
        'forward': (-45, 45),         # Forward range: -45 to 45 degrees
        'left': (-180, -135),         # Left range: -180 to -135 degrees
        'right': (135, 180)           # Right range: 135 to 180 degrees
    }

    # Convert degrees to radians
    angle_ranges_rad = {k: (v[0] * (3.14159 / 180), v[1] * (3.14159 / 180)) for k, v in angle_ranges_deg.items()}

    # Get the total number of scan points
    num_scans = len(msg.ranges)

    def is_object_in_range(start_angle, end_angle):
        """ Helper function to check if an object is within the specific angle range and under 0.5 meters """
        start_index = max(0, int((start_angle - msg.angle_min) / msg.angle_increment))
        end_index = min(num_scans - 1, int((end_angle - msg.angle_min) / msg.angle_increment))

        for i in range(start_index, end_index + 1):
            if 0.0 < msg.ranges[i] < 0.5:  # Object within 0.5 meters, ignoring NaN or infinite
                return True
        return False

    # Check for objects in the forward, left, and right ranges
    object_detected_forward = is_object_in_range(*angle_ranges_rad['forward'])
    object_detected_left = is_object_in_range(*angle_ranges_rad['left'])
    object_detected_right = is_object_in_range(*angle_ranges_rad['right'])

    # Publish status based on detection
    status = 0  # Default: No object detected
    if object_detected_forward:
        status = 8  # Object detected in front
    elif object_detected_left or object_detected_right:
        status = 2  # Object detected in left or right

    # Log the detection status
    if status != 0:
        rospy.loginfo("Object avoidance status: %d", status)

    # Publish the status to the object_avoidance topic
    avoidance_pub.publish(status)

def main():
    rospy.init_node('object_detector')
    
    # Create a publisher for the object avoidance status
    global avoidance_pub
    avoidance_pub = rospy.Publisher('/object_avoidance', Int32, queue_size=10)

    # Subscribe to the /scan topic to receive LaserScan messages
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

