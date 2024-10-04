from ultralytics import YOLO
import cv2
import rospy
from std_msgs.msg import Bool  # Standard ROS message type for boolean data
from sensor_msgs.msg import Image  # ROS message type for images
from cv_bridge import CvBridge, CvBridgeError  # Bridge between ROS and OpenCV

# Initialize ROS node
rospy.init_node('fire_detection_node')

# Create a ROS publisher for the /fire topic
fire_pub = rospy.Publisher('/fire', Bool, queue_size=20)

# Create a CvBridge object for converting ROS Image messages to OpenCV images
bridge = CvBridge()

# Load the trained YOLO model (replace 'best.pt' with the correct path to your model if needed)
model = YOLO('final_best.pt')

# Callback function to process the received image
def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV image
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Run YOLO model on the current frame
        results = model.predict(source=frame, imgsz=640, conf=0.4, save=False)  # Perform inference

        # Check if any fire detections are made (assuming fire class index is 0)
        fire_detected = False
        for result in results:
            if 0 in result.boxes.cls:  # Replace 0 with the class index for fire if different
                fire_detected = True
                break

        # Publish to the /fire topic
        fire_pub.publish(fire_detected)

        # Print status for debugging
        if fire_detected:
            rospy.loginfo("Fire detected, published 1 to /fire")
        else:
            rospy.loginfo("No fire detected, published 0 to /fire")

    except CvBridgeError as e:
        rospy.logerr(f"Error converting ROS Image to OpenCV: {e}")

# Subscribe to the ROS image topic
rospy.Subscriber('/camera/image_raw', Image, image_callback)

# Keep the node running
rospy.spin()

