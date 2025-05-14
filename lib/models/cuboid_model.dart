import 'package:flutter/foundation.dart';

class CuboidModel extends ChangeNotifier {
  double _width = 70.0; // cm - default wheelchair width
  double _height = 150.0; // cm - default height with person
  double _depth = 120.0; // cm - default wheelchair length
  bool _isIntersecting = false;

  // Add position properties
  double? _positionX = 0.0;
  double? _positionY = 0.0;
  double? _positionZ = -1.5;

  // Getters
  double get width => _width;
  double get height => _height;
  double get depth => _depth;
  bool get isIntersecting => _isIntersecting;

  // Position getters
  double? get positionX => _positionX;
  double? get positionY => _positionY;
  double? get positionZ => _positionZ;

  // Setters with notification
  set width(double value) {
    _width = value;
    notifyListeners();
  }

  set height(double value) {
    _height = value;
    notifyListeners();
  }

  set depth(double value) {
    _depth = value;
    notifyListeners();
  }

  set isIntersecting(bool value) {
    _isIntersecting = value;
    notifyListeners();
  }

  // Position setters
  set positionX(double? value) {
    _positionX = value;
    notifyListeners();
  }

  set positionY(double? value) {
    _positionY = value;
    notifyListeners();
  }

  set positionZ(double? value) {
    _positionZ = value;
    notifyListeners();
  }

  // Convert to map for platform channel
  Map<String, dynamic> toMap() {
    return {
      'width': _width,
      'height': _height,
      'depth': _depth,
      'positionX': _positionX,
      'positionY': _positionY,
      'positionZ': _positionZ,
    };
  }
}
