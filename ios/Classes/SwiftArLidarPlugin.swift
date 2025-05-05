import Flutter
import UIKit
import ARKit

public class SwiftArLidarPlugin: NSObject, FlutterPlugin, FlutterStreamHandler {
  public static func register(with registrar: FlutterPluginRegistrar) {
    let methodChannel = FlutterMethodChannel(name: "com.tcat/ar_lidar",
                                             binaryMessenger: registrar.messenger())
    let eventChannel = FlutterEventChannel(name: "com.tcat/ar_lidar_events",
                                           binaryMessenger: registrar.messenger())
    let instance = SwiftArLidarPlugin()
    registrar.addMethodCallDelegate(instance, channel: methodChannel)
    eventChannel.setStreamHandler(instance)
  }

  public func handle(_ call: FlutterMethodCall, result: @escaping FlutterResult) {
    switch call.method {
    case "initializeAR":
      // return whether ARKit (and LiDAR) is supported on this device
      result(ARWorldTrackingConfiguration.isSupported)
    case "updateCuboidDimensions":
      guard let args = call.arguments as? [String: Any],
            let width = args["width"] as? Double,
            let height = args["height"] as? Double,
            let depth = args["depth"] as? Double else {
        result(FlutterError(code: "INVALID_ARGS", message: nil, details: nil))
        return
      }
      // TODO: wire into your AR engine
      result(nil)
    default:
      result(FlutterMethodNotImplemented)
    }
  }

  // FlutterStreamHandler
  public func onListen(withArguments arguments: Any?,
                       eventSink events: @escaping FlutterEventSink) -> FlutterError? {
    // TODO: start sending events via `events(...)`
    return nil
  }

  public func onCancel(withArguments arguments: Any?) -> FlutterError? {
    // TODO: stop sending events
    return nil
  }
}
