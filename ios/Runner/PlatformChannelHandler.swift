import Flutter
import UIKit

class PlatformChannelHandler {
    private let methodChannel: FlutterMethodChannel
    private let eventChannel: FlutterEventChannel
    private var eventSink: FlutterEventSink?
    
    // Reference to our AR view controller
    private var arViewController: ARViewController?

    init(binaryMessenger: FlutterBinaryMessenger) {
        // Initialize MethodChannel
        self.methodChannel = FlutterMethodChannel(name: "com.tcat/ar_lidar", binaryMessenger: binaryMessenger)
        
        // Initialize EventChannel
        self.eventChannel = FlutterEventChannel(name: "com.tcat/ar_lidar_events", binaryMessenger: binaryMessenger)
        
        // Register stream handler
        self.eventChannel.setStreamHandler(StreamHandler(setEventSink: { [weak self] sink in
            self?.eventSink = sink
        }))
        
        // Register method handlers
        self.setupMethodHandlers()
    }

    private func setupMethodHandlers() {
        methodChannel.setMethodCallHandler { [weak self] (call, result) in
            guard let self = self else { return }

            switch call.method {
            case "initializeAR":
                self.initializeAR(result: result)
                
            case "updateCuboidDimensions":
                guard let args = call.arguments as? [String: Any],
                      let width = args["width"] as? Double,
                      let height = args["height"] as? Double,
                      let depth = args["depth"] as? Double else {
                    result(FlutterError(code: "INVALID_ARGUMENTS", message: "Invalid cuboid dimensions", details: nil))
                    return
                }
                self.arViewController?.updateCuboidDimensions(width: Float(width), height: Float(height), depth: Float(depth))
                result(nil)
                
            case "disposeAR":
                self.disposeAR()
                result(nil)
                
            default:
                result(FlutterMethodNotImplemented)
            }
        }
    }

    private func initializeAR(result: @escaping FlutterResult) {
        // Find the current root view controller (iOS13+ vs earlier)
        let rootVC: UIViewController?
        if #available(iOS 13.0, *) {
            rootVC = UIApplication.shared.connectedScenes
                .compactMap { ($0 as? UIWindowScene)?.windows.first }
                .first?.rootViewController
        } else {
            rootVC = UIApplication.shared.keyWindow?.rootViewController
        }
        guard let root = rootVC else {
            result(FlutterError(code: "NO_ROOT_VC", message: "No root view controller found", details: nil))
            return
        }

        // Instantiate the AR view controller
        let arVC = ARViewController()
        arVC.modalPresentationStyle = .fullScreen

        // Provide intersection callback to send events to Flutter
        arVC.onIntersectionUpdate = { [weak self] isIntersecting in
            DispatchQueue.main.async {
                self?.eventSink?(isIntersecting)
            }
        }

        self.arViewController = arVC

        // Present the AR view
        root.present(arVC, animated: true) {
            result(true)
        }
    }

    private func disposeAR() {
        arViewController?.dismiss(animated: true)
        arViewController = nil
    }
}

// MARK: - StreamHandler

class StreamHandler: NSObject, FlutterStreamHandler {
    private let setEventSink: (FlutterEventSink?) -> Void

    init(setEventSink: @escaping (FlutterEventSink?) -> Void) {
        self.setEventSink = setEventSink
    }

    func onListen(withArguments arguments: Any?, eventSink events: @escaping FlutterEventSink) -> FlutterError? {
        setEventSink(events)
        return nil
    }

    func onCancel(withArguments arguments: Any?) -> FlutterError? {
        setEventSink(nil)
        return nil
    }
}
