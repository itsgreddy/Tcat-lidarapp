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
      });

      // Main intersection binding to model
      _intersectionSubscription = _arService.intersectionStream.listen((isIntersecting) {
        final model = Provider.of<CuboidModel>(context, listen: false);
        model.isIntersecting = isIntersecting;
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
            child: Container(
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

          // Sliders for cuboid dimensions
          Padding(
            padding: const EdgeInsets.all(16.0),
            child: Consumer<CuboidModel>(
              builder: (context, model, child) {
                return Column(
                  children: [
                    DimensionSlider(
                      label: 'Width',
                      value: model.width,
                      min: 40,
                      max: 120,
                      onChanged: (value) {
                        model.width = value;
                        _arService.updateCuboidDimensions(model);
                      },
                    ),
                    DimensionSlider(
                      label: 'Height',
                      value: model.height,
                      min: 100,
                      max: 200,
                      onChanged: (value) {
                        model.height = value;
                        _arService.updateCuboidDimensions(model);
                      },
                    ),
                    DimensionSlider(
                      label: 'Depth',
                      value: model.depth,
                      min: 80,
                      max: 150,
                      onChanged: (value) {
                        model.depth = value;
                        _arService.updateCuboidDimensions(model);
                      },
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
