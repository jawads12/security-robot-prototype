
echo "Roscore starting"
roscore &
ROSCORE_PID=$!
sleep 5
roslaunch rplidar_ros rplidar.launch &
sleep 4


echo "Running python server"
python new_server.py &
sleep 2

echo "Running rosserial"
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200 &
sleep 2

rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB1 __name:=sensors _baud:=57600
