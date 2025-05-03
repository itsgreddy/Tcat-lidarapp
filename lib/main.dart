import 'package:flutter/material.dart';
// Try a more specific import
import 'package:provider/single_child_widget.dart';
import 'package:provider/provider.dart';
import 'models/cuboid_model.dart';
import 'screens/ar_screen.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return ChangeNotifierProvider(
      create: (context) => CuboidModel(),
      child: MaterialApp(
        title: 'LiDAR Accessibility Scanner',
        theme: ThemeData(
          primarySwatch: Colors.blue,
          useMaterial3: true,
        ),
        home: const ARScreen(),
      ),
    );
  }
}
