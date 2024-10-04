import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool

# Initialize variables
autonomous_mode = False

def laser_callback(scan_data):
    global autonomous_mode

    if not autonomous_mode:
        return  # Do nothing if autonomous mode is off

    # Laser data angles (in degrees) to check for obstacles between -45 to +45 degrees
    angle_min = -45
    angle_max = 45
    min_distance = 0.7  # Minimum distance for obstacle detection

    # Convert angles to indices
    angle_increment = scan_data.angle_increment
    scan_angle_min = scan_data.angle_min
    scan_angle_max = scan_data.angle_max

    # Check if the angles for -90 and +90 degrees are valid in the scan range
    if scan_angle_min <= -90 * (3.14159 / 180) and scan_angle_max >= 90 * (3.14159 / 180):
        index_left = int((90 * (3.14159 / 180) - scan_angle_min) / angle_increment)
        index_right = int((-90 * (3.14159 / 180) - scan_angle_min) / angle_increment)
    else:
        rospy.logwarn("Laser scan does not cover the full -90 to 90 degrees range.")
        return  # Exit if the scan range is not valid for these angles

    # Indices for -45 to 45 degrees for obstacle detection
    index_min = int((angle_min * (3.14159 / 180) - scan_angle_min) / angle_increment)
    index_max = int((angle_max * (3.14159 / 180) - scan_angle_min) / angle_increment)

    # Get distances in the specified range
    distances = scan_data.ranges[index_min:index_max]

    # Check if there's any object within the 0.7m range
    close_object = any(d < min_distance for d in distances if d > 0)  # Filter out invalid values

    direction_pub = rospy.Publisher('/direction', Int32, queue_size=10)

    if close_object:
        # Get distances for left (+90 degrees) and right (-90 degrees)
        distance_left = scan_data.ranges[index_left] if index_left < len(scan_data.ranges) else float('inf')
        distance_right = scan_data.ranges[index_right] if index_right < len(scan_data.ranges) else float('inf')

        if distance_left < distance_right:
            direction_pub.publish(6)  # Turn left
        else:
            direction_pub.publish(4)  # Turn right
    else:
        direction_pub.publish(8)  # Move forward

def autonomous_callback(data):
    global autonomous_mode
    autonomous_mode = data.data  # Update the autonomous mode

def main():
    rospy.init_node('autonomous_navigation')

    # Subscribe to the autonomous topic
    rospy.Subscriber('/autonomous', Bool, autonomous_callback)

    # Subscribe to the LaserScan topic
    rospy.Subscriber('/scan', LaserScan, laser_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

