package com.tcat.ar_lidar

import io.flutter.embedding.engine.plugins.FlutterPlugin
import io.flutter.plugin.common.MethodCall
import io.flutter.plugin.common.MethodChannel
import io.flutter.plugin.common.EventChannel

class ArLidarPlugin: FlutterPlugin, MethodChannel.MethodCallHandler {
  private lateinit var channel: MethodChannel
  private lateinit var eventChannel: EventChannel

  override fun onAttachedToEngine(binding: FlutterPlugin.FlutterPluginBinding) {
    channel = MethodChannel(binding.binaryMessenger, "com.tcat/ar_lidar")
    channel.setMethodCallHandler(this)

    eventChannel = EventChannel(binding.binaryMessenger, "com.tcat/ar_lidar_events")
    eventChannel.setStreamHandler(object: EventChannel.StreamHandler {
      override fun onListen(arguments: Any?, events: EventChannel.EventSink?) {
        // TODO: start emitting AR events
      }
      override fun onCancel(arguments: Any?) {
        // TODO: stop emitting
      }
    })
  }

  override fun onMethodCall(call: MethodCall, result: MethodChannel.Result) {
    when (call.method) {
      "initializeAR" -> {
        // TODO: initialize your AR engine
        result.success(null)
      }
      "updateCuboidDimensions" -> {
        val width = (call.argument<Double>("width") ?: 0.0).toFloat()
        val height = (call.argument<Double>("height") ?: 0.0).toFloat()
        val depth = (call.argument<Double>("depth") ?: 0.0).toFloat()
        // TODO: wire into your AR engine
        result.success(null)
      }
      else -> result.notImplemented()
    }
  }

  override fun onDetachedFromEngine(binding: FlutterPlugin.FlutterPluginBinding) {
    channel.setMethodCallHandler(null)
    eventChannel.setStreamHandler(null)
  }
}
