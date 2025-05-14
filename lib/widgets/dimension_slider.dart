import 'package:flutter/material.dart';

class DimensionSlider extends StatelessWidget {
  final String label;
  final double value;
  final double min;
  final double max;
  final ValueChanged<double> onChanged;
  final int? divisions; // Make divisions optional

  const DimensionSlider({
    Key? key,
    required this.label,
    required this.value,
    required this.min,
    required this.max,
    required this.onChanged,
    this.divisions, // Default is null
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    // Calculate sensible divisions based on min and max if not provided
    // For position sliders with decimal values, we want more granular control
    final bool isPositionSlider = label == 'X' || label == 'Y' || label == 'Z';
    final int calculatedDivisions = isPositionSlider ? 
        ((max - min) * 10).round() : // 0.1 increments for position
        (max - min).round();          // 1.0 increments for dimensions
    
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4.0),
      child: Row(
        children: [
          SizedBox(
            width: 20,
            child: Text(label),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: Slider(
              value: value.clamp(min, max),
              min: min,
              max: max,
              divisions: divisions ?? calculatedDivisions, // Use provided divisions or calculated
              label: isPositionSlider ? value.toStringAsFixed(1) : value.round().toString(),
              onChanged: onChanged,
            ),
          ),
          SizedBox(
            width: 50,
            child: Text(
              isPositionSlider ? value.toStringAsFixed(1) : value.round().toString(),
              textAlign: TextAlign.end,
            ),
          ),
        ],
      ),
    );
  }
}
