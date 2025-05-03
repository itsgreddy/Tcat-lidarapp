# tcat

LiDAR-based accessibility scanner for mapping indoor obstacles and measuring clearances.

## Table of Contents

- [Features](#features)  
- [Getting Started](#getting-started)  
  - [Prerequisites](#prerequisites)  
  - [Installation](#installation)  
  - [Running the App](#running-the-app)  
- [Architecture](#architecture)  
- [Permissions](#permissions)  
- [Contributing](#contributing)  
- [License](#license)  

## Features

- Initialize the LiDAR-based AR engine  
- Measure and update cuboid dimensions in real time  
- Stream spatial and object-detection events via EventChannel  
- Responsive UI with Flutter ScreenUtil  
- State management using Provider  

## Getting Started

### Prerequisites

- Flutter SDK (>=2.19.0 <3.0.0)  
- Xcode 12+ (for iOS builds)  
- Android Studio or SDK (for Android builds)  
- A LiDAR-equipped iOS device (e.g., iPhone 12 Pro or later)  

### Installation

```bash
git clone https://github.com/your-org/Tcat-lidarapp.git
cd tcat
flutter pub get
```

### Running the App

- iOS:  
  ```bash
  cd ios
  pod install
  cd ..
  flutter run --debug -d ios
  ```
- Android:  
  ```bash
  flutter run --debug -d android
  ```

## Architecture

- **Dart ↔ Native** communication via  
  - `MethodChannel('com.tcat/ar_lidar')` for commands  
  - `EventChannel('com.tcat/ar_lidar_events')` for real-time streams  
- Plugin code lives under `android/src/.../ArLidarPlugin.kt` and `ios/Classes/SwiftArLidarPlugin.swift`  
- UI/state management in `lib/` using Provider and ScreenUtil  

## Permissions

- **iOS**:  
  - `NSCameraUsageDescription` in `Info.plist`  
  - LiDAR access implied by ARKit entitlement  
- **Android**:  
  - `CAMERA` permission in `AndroidManifest.xml`  

## Contributing

1. Fork the repository  
2. Create a feature branch  
3. Commit your changes  
4. Open a pull request  

Please follow the existing lint rules (`analysis_options.yaml`) and write tests where applicable.

## License

MIT © Your Name or Organization
