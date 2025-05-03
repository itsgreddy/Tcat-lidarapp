import UIKit
import Flutter

@main
@objc class AppDelegate: FlutterAppDelegate {
    var channelHandler: PlatformChannelHandler?  // ✅ Strong reference retained

    override func application(
        _ application: UIApplication,
        didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
    ) -> Bool {
        GeneratedPluginRegistrant.register(with: self)

        if let controller = window?.rootViewController as? FlutterViewController {
            let handler = PlatformChannelHandler(binaryMessenger: controller.binaryMessenger)
            handler.register()
            channelHandler = handler // ✅ Retain handler instance to avoid deallocation
        }

        return super.application(application, didFinishLaunchingWithOptions: launchOptions)
    }
}
