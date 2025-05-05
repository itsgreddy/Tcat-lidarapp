# tcat

LiDAR-based accessibility scanner for mapping indoor obstacles and measuring clearances.

## Features

- Live LiDAR scanning via ARKit/ARCore plugin  
- Adjustable cuboid in 3D to model object clearances  
- Collision detection: red overlay when blocked, green when passable  
- Lock/unlock cuboid to world vs. camera reference  
- Flutter UI with sliders for Width/Height/Depth  

## Controls

- Top status bar: 🟩 PASSABLE / 🟥 BLOCKED  
- Sliders (bottom):  
  - W: cuboid width (cm)  
  - H: cuboid height (cm)  
  - D: cuboid depth (cm)  
- Lock/Unlock button (top-right): fix cuboid in world space or follow camera  
- Pan & pinch gestures on AR view to move/scale cuboid  

## Getting Started

1. `flutter pub get`  
2. Connect iOS/Android device with LiDAR/AR support  
3. `flutter run`  

## Architecture

- Flutter UI in `lib/`  
  - `ARScreen` widget hosts controls and status  
  - `DimensionSlider`, `CuboidModel`, `ARService`  
- Native AR plugin  
  - iOS: `SwiftArLidarPlugin.swift`, `ARManager.swift`  
  - Android: `ArLidarPlugin.kt`  
- Channels  
  - `MethodChannel('com.tcat/ar_lidar')` for init + dimension updates  
  - `EventChannel('com.tcat/ar_lidar_events')` for collision stream  

## Permissions

- iOS:  
  - `NSCameraUsageDescription` in `Info.plist`  
  - ARKit + LiDAR entitlement  

## Contributing

1. Fork & clone  
2. Create a feature branch  
3. Commit & push  
4. Open a PR
