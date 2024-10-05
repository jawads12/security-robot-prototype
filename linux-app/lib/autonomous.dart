import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'dart:async';
import 'main.dart';  // Ensure WebSocketManager is correctly imported

class AutonomousPage extends StatefulWidget {
  const AutonomousPage({Key? key}) : super(key: key);

  @override
  _AutonomousPageState createState() => _AutonomousPageState();
}

class _AutonomousPageState extends State<AutonomousPage> {
  final FocusNode _focusNode = FocusNode();
  Timer? _timer;
  String _sensorData = "Waiting for data...";
  bool isFireDetected = false;  // Track fire detection status
  StreamSubscription<String>? _sensorSubscription;
  StreamSubscription<bool>? _fireSubscription;  // For fire detection stream

  @override
  void initState() {
    super.initState();

    // Subscribe to the sensor data stream
    // _sensorSubscription = WebSocketManager.instance.sensorDataStreamController.stream.listen(
    //   (data) {
    //     setState(() {
    //       _sensorData = data;
    //     });
    //   },
    //   onError: (error) {
    //     print("Error in sensor data stream: $error");
    //   },
    // );

    // Subscribe to the fire detection stream
    _fireSubscription = WebSocketManager.instance.fireDetectionController.stream.listen(
      (fireStatus) {
        setState(() {
          isFireDetected = fireStatus;
        });
      },
      onError: (error) {
        print("Error in fire detection stream: $error");
      },
    );
  }

  @override
  void dispose() {
    _focusNode.dispose();
    _timer?.cancel();
    _sensorSubscription?.cancel();  // Cancel the sensor data stream subscription
    _fireSubscription?.cancel();  // Cancel the fire detection stream subscription
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Focus(
      focusNode: _focusNode,
      autofocus: true,
      child: Row(
        children: [
          Expanded(
            flex: 2,
            child: Column(
              children: [
                Expanded(
                  flex: 3,
                  child: Container(
                    margin: const EdgeInsets.all(16.0),
                    color: Colors.grey[900],
                    child: Center(
                      child: Text(
                        _sensorData,  // Display sensor data
                        style: TextStyle(color: Colors.white, fontSize: 16),
                      ),
                    ),
                  ),
                ),
                Expanded(
                  flex: 1,
                  child: Container(
                    margin: const EdgeInsets.all(16.0),
                    color: Colors.grey[900],
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.start,
                      children: [
                        Expanded(
                          child: Row(
                            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                            children: [
                              ElevatedButton(
                                onPressed: () => WebSocketManager.instance.sendDirection(8),
                                child: const Text('Start'),
                                style: ElevatedButton.styleFrom(
                                  backgroundColor: Colors.blue,
                                  foregroundColor: Colors.white,
                                ),
                              ),
                              ElevatedButton(
                                onPressed: () => WebSocketManager.instance.sendDirection(4),
                                child: const Text('Pattern'),
                                style: ElevatedButton.styleFrom(
                                  backgroundColor: Colors.blue,
                                  foregroundColor: Colors.white,
                                ),
                              ),
                              ElevatedButton(
                                onPressed: () => WebSocketManager.instance.sendDirection(1),
                                child: const Text('Create Map'),
                                style: ElevatedButton.styleFrom(
                                  backgroundColor: Colors.blue,
                                  foregroundColor: Colors.white,
                                ),
                              ),
                              ElevatedButton(
                                onPressed: () => WebSocketManager.instance.sendDirection(2),
                                child: const Text('Stop'),
                                style: ElevatedButton.styleFrom(
                                  backgroundColor: Colors.blue,
                                  foregroundColor: Colors.white,
                                ),
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
                        const SizedBox(height: 15),
                        if (isFireDetected)  // Conditionally show fire detection message
                          const Text(
                            "Caution: Fire",
                            style: TextStyle(color: Colors.redAccent, fontSize: 20),
                          ),
                        const SizedBox(height: 15),

                        const Text("Temperature: 25°C", style: TextStyle(color: Colors.white, fontSize: 16)),
                        const Text("Humidity: 10%", style: TextStyle(color: Colors.white, fontSize: 16)),
                        const Text("Gas: CO2", style: TextStyle(color: Colors.white, fontSize: 16)),
                        const SizedBox(height: 15),
                        Expanded(
                          child: Center(
                            child: Image.asset('assets/robot.jpeg'),  // Placeholder image
                          ),
                        ),
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
                          backgroundColor: Colors.red,
                          padding: const EdgeInsets.symmetric(horizontal: 32.0, vertical: 16.0),
                        ),
                        onPressed: () {},
                        child: const Text('E-STOP', style: TextStyle(fontSize: 18)),
                      ),
                      const SizedBox(height: 16.0),
                      Stack(
                        alignment: Alignment.center,
                        children: [
                          CircularProgressIndicator(
                            value: 0.6,
                            strokeWidth: 8.0,
                            backgroundColor: Colors.grey[800],
                          ),
                          const Text('60%', style: TextStyle(color: Colors.white)),
                        ],
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
