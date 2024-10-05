import 'dart:math';
import 'package:flutter/material.dart';
import 'dashboard_layout.dart'; // Ensure this file exists and defines DashboardLayout
import 'package:web_socket_channel/io.dart';
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'package:audioplayers/audioplayers.dart';

late Timer _pingTimer;




void main() {
  WidgetsFlutterBinding.ensureInitialized();
  WebSocketManager.instance.initWebSocket();
  runApp(MyApp());
}

class WebSocketManager {
  static final WebSocketManager instance = WebSocketManager._internal();

  WebSocketManager._internal();

  late IOWebSocketChannel commandChannel;
  late IOWebSocketChannel cameraChannel;
  late IOWebSocketChannel fireStatusChannel;  // Fire detection channel
  late IOWebSocketChannel sensorDataChannel;  // Sensor data channel
  late IOWebSocketChannel recordingChannel;  // New channel for recording commands
  late IOWebSocketChannel audioChannel;  // New channel for audio stream


  
  StreamController<Map<String, dynamic>> sensorDataController = StreamController<Map<String, dynamic>>.broadcast();  // Sensor data stream


  // Controllers for camera stream and fire detection status
  StreamController<Uint8List> cameraStreamController = StreamController<Uint8List>.broadcast();
  StreamController<bool> fireDetectionController = StreamController<bool>.broadcast();  // Fire detection stream
  final AudioPlayer _audioPlayer = AudioPlayer();  // Audio player for playing streamed audio


  void initWebSocket() {

    _pingTimer = Timer.periodic(Duration(seconds: 30), (timer) {
  if (commandChannel != null && commandChannel.sink != null) {
    commandChannel.sink.add('ping');
  }
  if (cameraChannel != null && cameraChannel.sink != null) {
    cameraChannel.sink.add('ping');
  }
  if (fireStatusChannel != null && fireStatusChannel.sink != null) {
    fireStatusChannel.sink.add('ping');
  }
  if (sensorDataChannel != null && sensorDataChannel.sink != null) {
    sensorDataChannel.sink.add('ping');
  }
});

    // Initialize WebSocket connections
    commandChannel = IOWebSocketChannel.connect('ws://192.168.68.101:8765');
    cameraChannel = IOWebSocketChannel.connect('ws://192.168.68.101:8766');
    fireStatusChannel = IOWebSocketChannel.connect('ws://192.168.68.101:8767');  // Fire detection channel
    sensorDataChannel = IOWebSocketChannel.connect('ws://192.168.68.101:8768');  // Sensor data channel
   // recordingChannel = IOWebSocketChannel.connect('ws://192.168.113.34:8769');
    //audioChannel = IOWebSocketChannel.connect('ws://192.168.113.34:8770');  // New WebSocket for audio




    // Listen to camera stream
    cameraChannel.stream.listen((message) {
      cameraStreamController.add(base64Decode(message));
    });

    // Listen to fire detection messages
    fireStatusChannel.stream.listen((message) {
      if (message == "1") {
        fireDetectionController.add(true);  // Fire detected
        print("Fire detected");
      } else {
        fireDetectionController.add(false);  // No fire detected
        print("No fire detected");
      }
    });

      sensorDataChannel.stream.listen((message) {
    sensorDataController.add(json.decode(message) as Map<String, dynamic>);
  });


   
    // audioChannel.stream.listen((message) {
    //   // Directly pass the Uint8List to play audio
    //   if (message is Uint8List) {
    //     _playAudioBytes(message);  // Correctly handle the audio data as bytes
    //   }
    // });
  
  commandChannel.stream.listen((message) {}, onDone: _handleWebSocketClose, onError: _handleWebSocketError);
  

  }



void _handleWebSocketClose() {
  print("Connection closed");
  _reconnectWebSocket();
}

void _handleWebSocketError(error) {
  print("Connection error: $error");
  _reconnectWebSocket();
}

void _reconnectWebSocket() {
  // Attempt to reconnect after a delay
  Future.delayed(Duration(seconds: 5), () {
    print("Reconnecting...");
    initWebSocket();
  });
}

  // Method to send direction commands to the robot
  void sendDirection(int direction) {
    try {
      commandChannel.sink.add(direction.toString());
      print('Direction sent: $direction');
    } catch (error) {
      print('Error sending direction: $error');
    }
  }

  void sendCommand(String command) {
  if (commandChannel != null) {
    commandChannel.sink.add(command);
    print('Command sent: $command');
  } else {
    print('WebSocket channel for commands is not connected.');
  }
}


//  Future<void> _playAudioBytes(Uint8List audioData) async {
//     try {
//       // Save the audio data to a local temporary file
//       final tempDir = await getTemporaryDirectory();
//       final tempFile = File('${tempDir.path}/temp_audio.mp3');
//       await tempFile.writeAsBytes(audioData);

//       // Play the audio file
//       await _audioPlayer.play(tempFile.path, isLocal: true);
//     } catch (e) {
//       print("Error playing audio stream: $e");
//     }
//   }

void sendRecordingCommand(String command) {
    recordingChannel.sink.add(command);
    print('Recording command sent: $command');
  }

  // Dispose method to close WebSocket connections and controllers
  void dispose() {
  _pingTimer?.cancel();
  cameraStreamController.close();
  fireDetectionController.close();  // Close fire detection stream
  commandChannel.sink.close();
  cameraChannel.sink.close();
  fireStatusChannel.sink.close();
  sensorDataController.close();  // Close sensor data stream
  sensorDataChannel.sink.close();
  recordingChannel.sink.close();
  _audioPlayer.dispose();  // Dispose the audio player
}
}

class MyApp extends StatelessWidget {
  const MyApp({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Robot Dashboard',
      home: HomeScreen(),
    );
  }
}

class HomeScreen extends StatefulWidget {
  @override
  _HomeScreenState createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  final GlobalKey<NavigatorState> _navigatorKey = GlobalKey<NavigatorState>();
  bool isFireDetected = false;  // Track fire detection status
  bool isAutonomousMode = false;
  StreamSubscription<bool>? fireSubscription;  // Subscription for fire detection stream
  final AudioPlayer _audioPlayer = AudioPlayer();  // Create an AudioPlayer instance

  @override
  void initState() {
    super.initState();

    // Listen to fire detection stream from WebSocketManager
    fireSubscription = WebSocketManager.instance.fireDetectionController.stream.listen((fireStatus) {
      setState(() {
        isFireDetected = fireStatus;
      });

      // Play sound when fire is detected
      if (fireStatus) {
        _playFireAlarm();  // Play the fire alarm sound
      } else {
        _stopFireAlarm();  // Stop the fire alarm when no fire is detected
      }
    });
  }

  // Function to play the fire alarm sound
 // Function to play the fire alarm sound
Future<void> _playFireAlarm() async {
  try {
    await _audioPlayer.play('assets/fire.mp3', isLocal: true);  // Play the asset
  } catch (e) {
    print("Error playing sound: $e");
  }
}

// Function to stop the fire alarm sound
Future<void> _stopFireAlarm() async {
  try {
    await _audioPlayer.stop();  // Stop the sound
  } catch (e) {
    print("Error stopping sound: $e");
  }
}

  @override
  void dispose() {
    fireSubscription?.cancel();
    _audioPlayer.dispose();  // Dispose the audio player
    super.dispose();
  }

  // Method to navigate to different pages
  void _navigateTo(Widget page, {bool replace = false}) {
    if (replace) {
      _navigatorKey.currentState?.pushReplacement(MaterialPageRoute(builder: (context) => page));
    } else {
      _navigatorKey.currentState?.push(MaterialPageRoute(builder: (context) => page));
    }
  }

  // Method to navigate to home (Dashboard)
  void _navigateHome() {
    _navigatorKey.currentState?.pushAndRemoveUntil(
      MaterialPageRoute(builder: (context) => DashboardLayout(isFireDetected: isFireDetected)),  // Pass fire detection status to DashboardLayout
      (Route<dynamic> route) => false,
    );


    void _toggleAutonomousMode(bool newValue) {
    setState(() {
      isAutonomousMode = newValue;
    });
    // Send command to the WebSocket server
    String command = newValue ? "activate_autonomous" : "deactivate_autonomous";
    WebSocketManager.instance.sendCommand(command);
  }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      body: SafeArea(
        child: Row(
          children: [
            Container(
              width: 80,
              color: Colors.black,
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  IconButton(
                    icon: const Icon(Icons.home),
                    color: Colors.grey[300],
                    onPressed: _navigateHome,
                  ),
                  
                  const SizedBox(height: 20),
                  IconButton(icon: const Icon(Icons.settings), color: Colors.grey[300], onPressed: () {}),
                ],
              ),
            ),
            Expanded(
              child: Navigator(
                key: _navigatorKey,
                onGenerateRoute: (RouteSettings settings) {
                  WidgetBuilder builder;
                  switch (settings.name) {
                    case '/':
                      builder = (BuildContext context) => DashboardLayout(isFireDetected: isFireDetected);
                      break;
                    default:
                      builder = (BuildContext context) => DashboardLayout(isFireDetected: isFireDetected);
                  }
                  return MaterialPageRoute(builder: builder, settings: settings);
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}
