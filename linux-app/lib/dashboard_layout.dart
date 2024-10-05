import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'dart:async';
import 'dart:typed_data';
import 'main.dart';  // Ensure this is correctly imported to access WebSocketManager

class DashboardLayout extends StatefulWidget {
  final bool isFireDetected;  // This parameter indicates fire detection status

  const DashboardLayout({Key? key, required this.isFireDetected}) : super(key: key);

  @override
  _DashboardLayoutState createState() => _DashboardLayoutState();
}

class _DashboardLayoutState extends State<DashboardLayout> {
  final FocusNode _focusNode = FocusNode();
  bool isRecording = false;
  bool isAutonomousMode = false; // Add this to track the state

  Timer? _timer;
  Uint8List? _imageData;
  StreamSubscription<Uint8List>? _cameraSubscription;
  StreamSubscription<Map<String, dynamic>>? _sensorDataSubscription;

   Map<String, dynamic> sensorValues = {
    'temperature': 'Loading...',
    'humidity': 'Loading...',
    'lpg_gas': 'Loading...',
    'co_gas': 'Loading...',
    'air_quality': 'Loading...',
    'motion': 'Loading...',
    'smoke': 'Loading...'
  };

  @override
  void initState() {
    super.initState();

    // Subscribe to the camera stream from the centralized WebSocket manager
    _cameraSubscription = WebSocketManager.instance.cameraStreamController.stream.listen(
      (Uint8List data) {
        setState(() {
          _imageData = data;
        });
      },
      onError: (error) {
        print("Error on camera stream: $error");
      }
    );

    // Subscribe to the sensor data stream
    _sensorDataSubscription = WebSocketManager.instance.sensorDataController.stream.listen(
      (Map<String, dynamic> data) {
        setState(() {
          sensorValues['temperature'] = "${double.parse(data['temperature'].toString()).toStringAsFixed(2)}°C";
          sensorValues['humidity'] = "${double.parse(data['humidity'].toString()).toStringAsFixed(2)}%";
          sensorValues['lpg_gas'] = "${double.parse(data['lpg_gas'].toString()).toStringAsFixed(2)} ppm";
          sensorValues['co_gas'] = "${double.parse(data['co_gas'].toString()).toStringAsFixed(2)} ppm";
          sensorValues['air_quality'] = "${double.parse(data['air_quality'].toString()).toStringAsFixed(2)} ppm";
          sensorValues['motion'] = data['motion']; // Assuming this is a string value
          sensorValues['smoke'] = data['smoke']; // Assuming this is a
         });
      },
      onError: (error) {
        print("Error on sensor data stream: $error");
      }
    );
  }

  @override
  void dispose() {
    _focusNode.dispose();
    _timer?.cancel();
    _cameraSubscription?.cancel();
    _sensorDataSubscription?.cancel();
    super.dispose();
  }

  void handleKeyInteraction(KeyEvent event, int direction) {
    if (event is KeyDownEvent && _timer == null) {
      _timer = Timer.periodic(const Duration(milliseconds: 100), (timer) {
        WebSocketManager.instance.sendDirection(direction);
      });
    } else if (event is KeyUpEvent) {
      _timer?.cancel();
      _timer = null;
    }
  }

  KeyEventResult _handleKeyPress(FocusNode node, KeyEvent event) {
    if (event.logicalKey == LogicalKeyboardKey.numpad8 || event.logicalKey == LogicalKeyboardKey.arrowUp) {
      handleKeyInteraction(event, 8);
      return KeyEventResult.handled;
    } else if (event.logicalKey == LogicalKeyboardKey.numpad4 || event.logicalKey == LogicalKeyboardKey.arrowLeft) {
      handleKeyInteraction(event, 4);
      return KeyEventResult.handled;
    } else if (event.logicalKey == LogicalKeyboardKey.numpad6 || event.logicalKey == LogicalKeyboardKey.arrowRight) {
      handleKeyInteraction(event, 6);
      return KeyEventResult.handled;
    } else if (event.logicalKey == LogicalKeyboardKey.numpad2 || event.logicalKey == LogicalKeyboardKey.arrowDown) {
      handleKeyInteraction(event, 2);
      return KeyEventResult.handled;
    }
    return KeyEventResult.ignored;
  }

  @override
  Widget build(BuildContext context) {
    return Focus(
      focusNode: _focusNode,
      onKeyEvent: _handleKeyPress,
      autofocus: true,
      child: Row(
        children: [
          Expanded(
            flex: 2,
            child: Column(
              children: [
                Expanded(
                  flex: 2,
                  child: Container(
                    margin: const EdgeInsets.all(16.0),
                    color: Colors.grey[900],
                    child: Center(
                      child: _imageData == null
                          ? const CircularProgressIndicator()
                          : Image.memory(_imageData!),  // Display camera feed
                    ),
                  ),
                ),
                if (widget.isFireDetected)  // Show "Fire Detected" if fire is detected
                  Container(
                    padding: const EdgeInsets.all(16.0),
                    color: Colors.red,
                    child: const Text(
                      'Fire Detected!',
                      style: TextStyle(color: Colors.white, fontSize: 24),
                    ),
                  ),
                Expanded(
                  flex: 1,
                  child: Container(
                    margin: const EdgeInsets.all(16.0),
                    color: Colors.grey[900],
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        const Text(
                          'Control',
                          style: TextStyle(color: Colors.yellow, fontSize: 20),
                        ),
                        const SizedBox(height: 20),
                        Expanded(
                          child: Column(
                            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                            children: [
                              IconButton(
                                icon: const Icon(Icons.arrow_upward),
                                color: Colors.blue,
                                iconSize: 30,
                                onPressed: () => WebSocketManager.instance.sendDirection(8),
                              ),
                              Row(
                                mainAxisAlignment: MainAxisAlignment.center,
                                children: [
                                  IconButton(
                                    icon: const Icon(Icons.arrow_back),
                                    color: Colors.blue,
                                    iconSize: 30,
                                    onPressed: () => WebSocketManager.instance.sendDirection(4),
                                  ),
                                  const SizedBox(width: 24), // Space for better alignment
                                  IconButton(
                                    icon: const Icon(Icons.arrow_forward),
                                    color: Colors.blue,
                                    iconSize: 30,
                                    onPressed: () => WebSocketManager.instance.sendDirection(6),
                                  ),
                                ],
                              ),
                              IconButton(
                                icon: const Icon(Icons.arrow_downward),
                                color: Colors.blue,
                                iconSize: 30,
                                onPressed: () => WebSocketManager.instance.sendDirection(2),
                              ),
                            ],
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ],
            ),
          ),
          Expanded(
            flex: 1,
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Expanded(
                  flex: 2,
                  child: Container(
                    margin: const EdgeInsets.all(16.0),
                    color: Colors.grey[900],
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.start,
                      children: [
                        SizedBox(height: 15), // Adds space between the text and the image
                        if (widget.isFireDetected)  // Show "Fire" only when fire is detected
                          Text("Caution: Fire", style: TextStyle(color: Colors.redAccent, fontSize: 20)),
                        SizedBox(height: 15), // Adds space between the text and the image

                        Text("Temperature: ${sensorValues['temperature']}", style: TextStyle(color: Colors.green, fontSize: 16)),
                        Text("Humidity: ${sensorValues['humidity']}", style: TextStyle(color: Colors.green, fontSize: 16)),
                        Text("LPG Gas: ${sensorValues['lpg_gas']}", style: TextStyle(color: Colors.blue, fontSize: 16)),
                        Text("CO Gas: ${sensorValues['co_gas']}", style: TextStyle(color: Colors.white, fontSize: 16)),
                        Text("Air Quality: ${sensorValues['air_quality']}", style: TextStyle(color: Colors.white, fontSize: 16)),
                        Text("Motion: ${sensorValues['motion']}", style: TextStyle(color: Colors.white, fontSize: 16)),
                        Text("Smoke: ${sensorValues['smoke']}", style: TextStyle(color: Colors.white, fontSize: 16)),
                        SizedBox(height: 15), // Adds space between the text and the image
                        
                      ],
                    ),
                  ),
                ),
                Expanded(
                  flex: 1,
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        backgroundColor: isAutonomousMode ? Colors.red : Colors.green,
                        padding: const EdgeInsets.symmetric(horizontal: 32.0, vertical: 16.0),
                      ),
                      onPressed: () {
                        setState(() {
                          // Toggle the autonomous mode
                          isAutonomousMode = !isAutonomousMode;
                          
                          // Send WebSocket command based on the current state
                          if (isAutonomousMode) {
                            WebSocketManager.instance.sendCommand('autonomous_on'); // Send autonomous mode ON command
                          } else {
                            WebSocketManager.instance.sendCommand('autonomous_off'); // Send autonomous mode OFF command
                          }
                        });
                      },
                      child: Text(isAutonomousMode ? 'Stop Autonomous Mode' : 'Activate Autonomous Mode', style: TextStyle(fontSize: 18)),
                    ),


                      const SizedBox(height: 16.0),
                     Expanded(
                          child: Center(
                            child: Image.asset('assets/robot.gif'), // Placeholder image path
                          ),
                        ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
