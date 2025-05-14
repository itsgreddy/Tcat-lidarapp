import 'dart:async';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../models/cuboid_model.dart';
import '../services/ar_service.dart';
import '../widgets/dimension_slider.dart';

class ARScreen extends StatefulWidget {
  const ARScreen({Key? key}) : super(key: key);

  @override
  State<ARScreen> createState() => _ARScreenState();
}

class _ARScreenState extends State<ARScreen> {
  final ARService _arService = ARService();
  StreamSubscription<bool>? _intersectionSubscription;
  bool _arInitialized = false;
  bool isColliding = false;

  // Add status message to show path planning status
  String _statusMessage = "Initialize AR to begin";

  @override
  void initState() {
    super.initState();
    _initializeAR();

    // Passive listener to log intersection status
    ARService().intersectionStream.listen((status) {
      print("Intersection status: $status");
      setState(() {
        isColliding = status;
      });
    });
  }

  Future<void> _initializeAR() async {
    final success = await _arService.initializeAR();
    if (success) {
      setState(() {
        _arInitialized = true;
        _statusMessage = "AR initialized. Use the 'Plan Path' button in AR view.";
      });

      // Main intersection binding to model
      _intersectionSubscription = _arService.intersectionStream.listen((isIntersecting) {
        final model = Provider.of<CuboidModel>(context, listen: false);
        model.isIntersecting = isIntersecting;
      });
    } else {
      setState(() {
        _statusMessage = "Failed to initialize AR. Check device compatibility.";
      });
    }
  }

  @override
  void dispose() {
    _intersectionSubscription?.cancel();
    _arService.disposeAR();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Accessibility Scanner'),
      ),
      body: Column(
        children: [
          // AR View placeholder (in real app, rendered by native code)
          Expanded(
            child: Stack(
              children: [
                Container(
                  width: double.infinity,
                  color: Colors.black,
                  child: Center(
                    child: !_arInitialized
                        ? const CircularProgressIndicator(color: Colors.white)
                        : const Text(
                            'LiDAR Scanning Active',
                            style: TextStyle(color: Colors.white),
                          ),
                  ),
                ),

                // Add path planning instructions overlay
                if (_arInitialized)
                  Positioned(
                    bottom: 100,
                    left: 0,
                    right: 0,
                    child: Container(
                      padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 8),
                      color: Colors.black54,
                      child: Text(
                        _statusMessage,
                        style: const TextStyle(color: Colors.white),
                        textAlign: TextAlign.center,
                      ),
                    ),
                  ),
              ],
            ),
          ),

          // Additional intersection banner
          Container(
            width: double.infinity,
            height: 20,
            color: isColliding ? Colors.red : Colors.green,
          ),

          // Status indicator
          Consumer<CuboidModel>(
            builder: (context, model, child) {
              return Container(
                width: double.infinity,
                color: model.isIntersecting ? Colors.red : Colors.green,
                padding: const EdgeInsets.symmetric(vertical: 8),
                child: Center(
                  child: Text(
                    model.isIntersecting ? '🟥 BLOCKED' : '🟩 PASSABLE',
                    style: const TextStyle(
                      color: Colors.white,
                      fontWeight: FontWeight.bold,
                      fontSize: 18,
                    ),
                  ),
                ),
              );
            },
          ),

          // Updated sliders for cuboid dimensions and position
          Padding(
            padding: const EdgeInsets.all(16.0),
            child: Consumer<CuboidModel>(
              builder: (context, model, child) {
                return Column(  // Changed from Row to Column for better layout on smaller screens
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    // Dimension sliders
                    Text('Size', style: TextStyle(fontWeight: FontWeight.bold)),
                    const SizedBox(height: 8),
                    Row(
                      children: [
                        Expanded(
                          child: DimensionSlider(
                            label: 'W',
                            value: model.width,
                            min: 40,
                            max: 120,
                            onChanged: (value) {
                              model.width = value;
                              _arService.updateCuboidDimensions(model);
                            },
                          ),
                        ),
                      ],
                    ),
                    Row(
                      children: [
                        Expanded(
                          child: DimensionSlider(
                            label: 'H',
                            value: model.height,
                            min: 100,
                            max: 200,
                            onChanged: (value) {
                              model.height = value;
                              _arService.updateCuboidDimensions(model);
                            },
                          ),
                        ),
                      ],
                    ),
                    Row(
                      children: [
                        Expanded(
                          child: DimensionSlider(
                            label: 'D',
                            value: model.depth,
                            min: 80,
                            max: 150,
                            onChanged: (value) {
                              model.depth = value;
                              _arService.updateCuboidDimensions(model);
                            },
                          ),
                        ),
                      ],
                    ),
                        
                    const SizedBox(height: 16),
                        
                    // Position sliders
                    Text('Position', style: TextStyle(fontWeight: FontWeight.bold)),
                    const SizedBox(height: 8),
                    Row(
                      children: [
                        Expanded(
                          child: DimensionSlider(
                            label: 'X',
                            value: model.positionX ?? 0.0,
                            min: -3.0,
                            max: 3.0,
                            divisions: 60, // 6.0 range with 0.1 increments
                            onChanged: (value) {
                              model.positionX = value;
                              _arService.updateCuboidPosition(model);
                            },
                          ),
                        ),
                      ],
                    ),
                    Row(
                      children: [
                        Expanded(
                          child: DimensionSlider(
                            label: 'Y',
                            value: model.positionY ?? 0.0,
                            min: -2.0,
                            max: 2.0,
                            divisions: 40, // 4.0 range with 0.1 increments
                            onChanged: (value) {
                              model.positionY = value;
                              _arService.updateCuboidPosition(model);
                            },
                          ),
                        ),
                      ],
                    ),
                    Row(
                      children: [
                        Expanded(
                          child: DimensionSlider(
                            label: 'Z',
                            value: model.positionZ ?? -1.5,
                            min: -5.0,
                            max: 0.0,
                            divisions: 50, // 5.0 range with 0.1 increments
                            onChanged: (value) {
                              model.positionZ = value;
                              _arService.updateCuboidPosition(model);
                            },
                          ),
                        ),
                      ],
                    ),
                  ],
                );
              },
            ),
          ),
        ],
      ),
    );
  }
}
