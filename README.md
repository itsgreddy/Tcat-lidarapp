# tcat

LiDAR-based accessibility scanner for mapping indoor obstacles and measuring clearances.

## Features

- Live LiDAR scanning via ARKit/ARCore plugin  
- Adjustable cuboid in 3D to model object clearances  
- Collision detection: red overlay when blocked, green when passable  
- Lock/unlock cuboid to world vs. camera reference  
- Flutter UI with sliders for Width/Height/Depth  
- Path planning with obstacle avoidance
- Real-time path recalculation when obstacles detected

## Controls

- Top status bar: 🟩 PASSABLE / 🟥 BLOCKED  
- Sliders (bottom):  
  - W: cuboid width (cm)  
  - H: cuboid height (cm)  
  - D: cuboid depth (cm)  
- Lock/Unlock button (top-right): fix cuboid in world space or follow camera  
- Pan & pinch gestures on AR view to move/scale cuboid
- **Path Planning**:
  - Tap "Plan Path" button in top-left corner of AR view
  - First tap places the start point (green sphere)
  - Second tap places the end point (blue sphere)
  - The app will show a green path if clear, red if blocked
  - If path is blocked, it will attempt to find an alternative route
  - Export button appears when path is created

## Getting Started

1. `flutter pub get`  
2. Connect iOS/Android device with LiDAR/AR support  
3. Configure code signing:
   - Open Xcode: `open ios/Runner.xcworkspace`
   - Select "Runner" project in the navigator
   - Select "Runner" target
   - Go to "Signing & Capabilities" tab
   - Check "Automatically manage signing"
   - Select your personal team
   - Change Bundle Identifier if needed (e.g., "com.yourname.tcat")
4. `flutter run`  

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
  - Developer account for device testing

## Contributing

1. Fork & clone  
2. Create a feature branch  
3. Commit & push  
4. Open a PR
