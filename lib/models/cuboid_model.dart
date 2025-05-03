import 'package:flutter/foundation.dart';

class CuboidModel extends ChangeNotifier {
  double _width = 70.0; // cm - default wheelchair width
  double _height = 150.0; // cm - default height with person
  double _depth = 120.0; // cm - default wheelchair length
  bool _isIntersecting = false;

  // Getters
  double get width => _width;
  double get height => _height;
  double get depth => _depth;
  bool get isIntersecting => _isIntersecting;

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

  // Convert to map for platform channel
  Map<String, dynamic> toMap() {
    return {
      'width': _width,
      'height': _height,
      'depth': _depth,
    };
  }
}
