import Flutter
import UIKit

class PlatformChannelHandler {
    private let methodChannel: FlutterMethodChannel
    private let eventChannel: FlutterEventChannel
    private var eventSink: FlutterEventSink?

    private var arViewController: ARViewController?

    init(binaryMessenger: FlutterBinaryMessenger) {
        methodChannel = FlutterMethodChannel(name: "com.tcat/ar_lidar", binaryMessenger: binaryMessenger)
        eventChannel = FlutterEventChannel(name: "com.tcat/ar_lidar_events", binaryMessenger: binaryMessenger)

        eventChannel.setStreamHandler(StreamHandler(setEventSink: { [weak self] sink in
            self?.eventSink = sink
        }))

        setupMethodHandlers()
    }

    func register() {
        print("✅ PlatformChannelHandler registered.")
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
        guard let rootViewController = UIApplication.shared.windows.first?.rootViewController else {
            result(FlutterError(code: "NO_ROOT_VC", message: "Could not get root view controller", details: nil))
            return
        }

        let arVC = ARViewController()
        arVC.modalPresentationStyle = .fullScreen

        arVC.onIntersectionUpdate = { [weak self] isIntersecting in
            DispatchQueue.main.async {
                self?.eventSink?(isIntersecting)
            }
        }

        arViewController = arVC
        rootViewController.present(arVC, animated: true) {
            result(true)
        }
    }

    private func disposeAR() {
        arViewController?.dismiss(animated: true)
        arViewController = nil
    }
}

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
