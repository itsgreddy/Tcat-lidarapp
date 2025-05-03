import 'package:flutter/material.dart';

class DimensionSlider extends StatelessWidget {
  final String label;
  final double value;
  final double min;
  final double max;
  final Function(double) onChanged;
  final String unit;

  const DimensionSlider({
    Key? key,
    required this.label,
    required this.value,
    required this.min,
    required this.max,
    required this.onChanged,
    this.unit = 'cm',
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Text('$label:', style: const TextStyle(fontWeight: FontWeight.bold)),
            Text('${value.toStringAsFixed(1)} $unit')
          ],
        ),
        Slider(
          value: value,
          min: min,
          max: max,
          divisions: ((max - min) ~/ 5).toInt(),
          onChanged: onChanged,
        ),
      ],
    );
  }
}
