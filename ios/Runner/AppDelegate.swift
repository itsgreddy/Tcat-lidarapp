import UIKit
import Flutter

// Declare the class here so it's visible to the AppDelegate
class PlatformChannelHandler {
    init(binaryMessenger: FlutterBinaryMessenger) {
        // Placeholder - the real implementation is in PlatformChannelHandler.swift
        // This ensures the compiler finds a definition of the class
    }
}

@main
@objc class AppDelegate: FlutterAppDelegate {
    private var channelHandler: PlatformChannelHandler?
    
    override func application(
        _ application: UIApplication,
        didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
    ) -> Bool {
        // Make sure we register plugins before setting up channel handler
        GeneratedPluginRegistrant.register(with: self)
        
        let controller = window?.rootViewController as! FlutterViewController
        
        // Initialize platform channel handler
        channelHandler = PlatformChannelHandler(binaryMessenger: controller.binaryMessenger)
        
        return super.application(application, didFinishLaunchingWithOptions: launchOptions)
    }
}
