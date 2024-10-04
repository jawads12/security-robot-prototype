import asyncio
import websockets
import rospy
import cv2
import base64
import json
import sounddevice as sd
from sensor_msgs.msg import Image  # For publishing images in ROS
from std_msgs.msg import Bool, Int32, Float32, String  # To subscribe to topics
from cv_bridge import CvBridge  # For converting OpenCV images to ROS Image messages

# Initialize ROS node
rospy.init_node('secubot_node', anonymous=True)

# ROS publishers
image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
direction_pub = rospy.Publisher('direction', Int32, queue_size=10)
autonomous_pub = rospy.Publisher('autonomous', Bool, queue_size=10)  # Publisher for autonomous mode


# Create a CvBridge object
bridge = CvBridge()

# WebSocket clients sets
camera_connected_clients = set()
fire_connected_clients = set()
sensor_connected_clients = set()
audio_connected_clients = set()


# Global variable for fire status
fire_detected = False
autonomous_mode = False  # New variable to track autonomous mode


# Sensor data dictionary
sensor_data = {
    'humidity': 0.0,
    'temperature': 0.0,
    'lpg_gas': 0.0,
    'co_gas': 0.0,
    'air_quality': 0.0,
    'motion': '',
    'smoke': ''
}

# Callback function for /fire topic
def fire_callback(msg):
    global fire_detected
    fire_detected = msg.data

# Callback functions for sensor data
def humidity_callback(msg):
    sensor_data['humidity'] = msg.data

def temperature_callback(msg):
    sensor_data['temperature'] = msg.data

def lpg_gas_callback(msg):
    sensor_data['lpg_gas'] = msg.data

def co_gas_callback(msg):
    sensor_data['co_gas'] = msg.data

def air_quality_callback(msg):
    sensor_data['air_quality'] = msg.data

def motion_callback(msg):
    sensor_data['motion'] = msg.data

def smoke_callback(msg):

    sensor_data['smoke'] = msg.data

# Subscribe to topics
rospy.Subscriber('/fire', Bool, fire_callback)
rospy.Subscriber('sensor/humidity', Float32, humidity_callback)
rospy.Subscriber('sensor/temperature', Float32, temperature_callback)
rospy.Subscriber('sensor/lpg_gas', Float32, lpg_gas_callback)
rospy.Subscriber('sensor/co_gas', Float32, co_gas_callback)
rospy.Subscriber('sensor/air_quality', Float32, air_quality_callback)
rospy.Subscriber('sensor/motion', String, motion_callback)
rospy.Subscriber('sensor/smoke', String, smoke_callback)








video_writer = None
is_recording = False

def start_recording(frame):
    global video_writer, is_recording
    try:
        if not is_recording:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            video_writer = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))
            is_recording = True
            print("Recording started.")
        video_writer.write(frame)
    except Exception as e:
        print(f"Failed to start recording: {e}")
        is_recording = False
        if video_writer is not None:
            video_writer.release()
            video_writer = None



def stop_recording():
    global video_writer, is_recording
    if is_recording and video_writer is not None:
        video_writer.release()
        video_writer = None
        is_recording = False
        print("Recording stopped and file saved.")
    else:
        print("Stop recording called without active recording.")


async def handle_recording_commands(websocket, path):
    global is_recording
    try:
        while True:
            try:
                command = await asyncio.wait_for(websocket.recv(), timeout=30)  # Timeout to trigger ping/pong
            except asyncio.TimeoutError:
                await websocket.ping()  # Keep connection alive
                continue
            # Process command
    except Exception as e:
        print(f"WebSocket command error: {e}")
    finally:
        stop_recording()

async def send_audio_stream(websocket, path):
    global main_event_loop
    audio_connected_clients.add(websocket)

    # Callback function for audio streaming
    def audio_callback(indata, frames, time, status):
        audio_data = indata.tobytes()
        asyncio.run_coroutine_threadsafe(websocket.send(audio_data), main_event_loop)

    try:
        # Start audio capture
        with sd.InputStream(samplerate=44100, channels=1, callback=audio_callback):
            while True:
                await asyncio.sleep(0.1)  # Prevents blocking
    finally:
        audio_connected_clients.remove(websocket)








# Camera streaming WebSocket server function
#async def send_camera_frames(websocket, path):
    #cap = cv2.VideoCapture(0)
   # camera_connected_clients.add(websocket)
    #try:
    #    while True:
   #         ret, frame = cap.read()
  #          if not ret:
 #               break
#
  #          image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
 #           image_pub.publish(image_msg)
#
        #    _, buffer = cv2.imencode('.jpg', frame)
       #     jpg_as_text = base64.b64encode(buffer).decode('utf-8')
      #      await websocket.send(jpg_as_text)
     #       await asyncio.sleep(0.1)
    #finally:
   #     camera_connected_clients.remove(websocket)
  #      cap.release()
 #

# F

# Initialize the CvBridge class


# Function to send camera frames through WebSocket with lower quality and publish to ROS
async def send_camera_frames(websocket, path):
    cap = cv2.VideoCapture(0)
    camera_connected_clients.add(websocket)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Resize the frame to a smaller resolution for WebSocket transmission and ROS publishing
            resized_frame = cv2.resize(frame, (680, 480))  # Lower resolution (e.g., 320x240)

            # Compress the frame to reduce file size (lower JPEG quality)
            _, buffer = cv2.imencode('.jpg', resized_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 30])  # 30% quality

            # Convert the low-resolution frame to an image message for ROS
            image_msg = bridge.cv2_to_imgmsg(resized_frame, encoding="bgr8")
            image_pub.publish(image_msg)  # Publish the low-resolution image to ROS

            # Encode the low-res image to base64 for WebSocket transmission
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')

            # Send the low-resolution image over WebSocket
            await websocket.send(jpg_as_text)

            # Introduce a slight delay to avoid overwhelming the WebSocket server
            await asyncio.sleep(0.1)
    
    finally:
        camera_connected_clients.remove(websocket)
        cap.release()



# Fire detection WebSocket server function
async def send_fire_status(websocket, path):
    fire_connected_clients.add(websocket)
    try:
        while True:
            fire_status = "1" if fire_detected else "0"
            await websocket.send(fire_status)
            await asyncio.sleep(0.5)
    finally:
        fire_connected_clients.remove(websocket)

"""
async def handle_autonomous_mode(websocket, path):
    global autonomous_mode
    try:
        while True:
            command = await websocket.recv()
            if command in ['1', '0']:  # Expecting '1' to activate and '0' to deactivate
                autonomous_mode = command == '1'  # Update autonomous_mode variable
                # Publish the autonomous mode state as a boolean
                await websocket.send(f"{1 if autonomous_mode else 0}")  # Echo back the new state
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")
"""
        
        
# Command direction WebSocket server function
"""
async def receive_directions(websocket, path):
    global autonomous_mode
    try:
        while True:
            try:
                command = await asyncio.wait_for(websocket.recv(), timeout=30)
                if command == 'ping':
                    await websocket.pong()
                elif command == 'toggle_autonomous':
                    autonomous_mode = not autonomous_mode  # Toggle the mode
                    direction_pub.publish(1 if autonomous_mode else 0)  # Publish to ROS
                    await websocket.send(f"{1 if autonomous_mode else 0}")  # Send back the status
                    print(f"Autonomous mode set to: {autonomous_mode}")
                else:
                    direction_pub.publish(int(command))  # Publish direction commands
            except asyncio.TimeoutError:
                await websocket.ping()  # Keep connection alive
    except Exception as e:
        rospy.loginfo(f"WebSocket error: {e}")
"""
async def receive_directions(websocket, path):
    global autonomous_mode
    try:
        while True:
            try:
                command = await asyncio.wait_for(websocket.recv(), timeout=30)

                if command == 'ping':
                    await websocket.pong()

                elif command == 'autonomous_on':
                    # Set autonomous mode to True
                    autonomous_mode = True

                    # Publish the autonomous status to a separate ROS topic
                    autonomous_pub.publish(Bool(autonomous_mode))  # True for ON
                    
                    # Send back the confirmation
                    await websocket.send("1")  # 1 for autonomous ON
                    
                    print("Autonomous mode enabled")

                elif command == 'autonomous_off':
                    # Set autonomous mode to False
                    autonomous_mode = False

                    # Publish the autonomous status to a separate ROS topic
                    autonomous_pub.publish(Bool(autonomous_mode))  # False for OFF
                    
                    # Send back the confirmation
                    await websocket.send("0")  # 0 for autonomous OFF
                    
                    print("Autonomous mode disabled")

                else:
                    # If the command is not related to autonomous, treat it as a normal direction command
                    try:
                        direction_pub.publish(int(command))  # Publish direction commands
                    except ValueError:
                        # Log invalid command input
                        rospy.logerr(f"Invalid command received: {command}")

            except asyncio.TimeoutError:
                # Send ping to keep the WebSocket connection alive
                await websocket.ping()

    except Exception as e:
        # Handle any WebSocket errors and log them
        rospy.loginfo(f"WebSocket error: {e}")



# Sensor data WebSocket server function
async def send_sensor_data(websocket, path):
    sensor_connected_clients.add(websocket)
    try:
        while True:
            await websocket.send(json.dumps(sensor_data))
            await asyncio.sleep(1)
    finally:
        sensor_connected_clients.remove(websocket)

# Example of handling close on the server
async def websocket_handler(websocket, path):
    try:
        async for message in websocket:
            # Process message
            print(message)
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")
    finally:
        if not websocket.closed:
            await websocket.close()



# Function to start all WebSocket servers simultaneously
async def main():
    global main_event_loop
    main_event_loop = asyncio.get_event_loop()  # Set the main event loop

    camera_server = await websockets.serve(send_camera_frames, '0.0.0.0', 8766)
    fire_server = await websockets.serve(send_fire_status, '0.0.0.0', 8767)
    command_server = await websockets.serve(receive_directions, '0.0.0.0', 8765)
    sensor_server = await websockets.serve(send_sensor_data, '0.0.0.0', 8768)
    #autonomous_server = await websockets.serve(handle_autonomous_mode, '0.0.0.0', 8771) 
    #recording_command_server = await websockets.serve(handle_recording_commands, '0.0.0.0', 8769)
    #audio_server = await websockets.serve(send_audio_stream, '0.0.0.0', 8770)

    rospy.loginfo("WebSocket servers started on ports 8766 (camera), 8767 (fire), 8765 (commands), and 8768 (sensors), .")
    await asyncio.gather(
        camera_server.wait_closed(),
        fire_server.wait_closed(),
        command_server.wait_closed(),
        sensor_server.wait_closed()
     
    )

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node shutdown")
    except Exception as e:
        rospy.loginfo(f"An unexpected error occurred: {e}")
