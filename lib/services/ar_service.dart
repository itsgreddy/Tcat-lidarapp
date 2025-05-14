import 'package:flutter/services.dart';
import '../models/cuboid_model.dart';

class ARService {
  static const MethodChannel _channel = MethodChannel('com.tcat/ar_lidar');
  static const EventChannel _eventChannel = EventChannel('com.tcat/ar_lidar_events');

  // Initialize AR session
    Future<bool> initializeAR() async {
      try {
        final result = await _channel.invokeMethod('initializeAR');
        return result == true; // Ensures only `true` is returned, otherwise `false`
      } on PlatformException catch (e) {
        print("Failed to initialize AR: ${e.message}");
        return false;
      }
    }


  // Update cuboid dimensions
  Future<void> updateCuboidDimensions(CuboidModel model) async {
    try {
      await _channel.invokeMethod('updateCuboidDimensions', model.toMap());
    } on PlatformException catch (e) {
      print("Failed to update cuboid dimensions: ${e.message}");
    }
  }
  
  // Add method to update cuboid position
  Future<void> updateCuboidPosition(CuboidModel model) async {
    try {
      await _channel.invokeMethod('updateCuboidPosition', {
        'positionX': model.positionX ?? 0.0,
        'positionY': model.positionY ?? 0.0,
        'positionZ': model.positionZ ?? -1.5,
      });
    } on PlatformException catch (e) {
      print("Failed to update cuboid position: ${e.message}");
    }
  }

  // Stream of intersection events
  Stream<bool> get intersectionStream {
    return _eventChannel.receiveBroadcastStream()
      .map((event) => event as bool);
  }

  // Clean up AR resources
  Future<void> disposeAR() async {
    try {
      await _channel.invokeMethod('disposeAR');
    } on PlatformException catch (e) {
      print("Failed to dispose AR: ${e.message}");
    }
  }
}
