import ARKit
import RealityKit
import Combine

class ARManager: NSObject {
    var arView: ARView?
    private var updateSubscription: Cancellable?

    private var cuboidEntity: ModelEntity?
    private var cuboidWidth: Float = 0.7
    private var cuboidHeight: Float = 1.5
    private var cuboidDepth: Float = 1.2

    private var isIntersecting = false
    private var intersectionCallback: ((Bool) -> Void)?

    private let arConfiguration: ARWorldTrackingConfiguration = {
        let configuration = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            configuration.sceneReconstruction = .mesh
        }
        configuration.planeDetection = []
        return configuration
    }()

    func initialize(arView: ARView, callback: @escaping (Bool) -> Void) {
        guard ARWorldTrackingConfiguration.isSupported else { return }
        self.arView = arView
        self.intersectionCallback = callback

        arView.session.run(arConfiguration)
        createCuboid()

        // Subscribe to scene updates for real-time intersection checks
        updateSubscription = arView.scene.subscribe(to: SceneEvents.Update.self) { [weak self] _ in
            self?.checkIntersection()
        }
    }

    func updateCuboidDimensions(width: Float, height: Float, depth: Float) {
        cuboidWidth = width / 100
        cuboidHeight = height / 100
        cuboidDepth = depth / 100

        if let anchor = cuboidEntity?.parent as? AnchorEntity {
            arView?.scene.removeAnchor(anchor)
        }
        createCuboid()
    }

    func cleanup() {
        updateSubscription?.cancel()
        arView?.session.pause()
        cuboidEntity = nil
        arView = nil
    }

    var currentCuboidEntity: ModelEntity? {
        return cuboidEntity
    }

    private func createCuboid() {
        guard let arView = arView else { return }

        let boxMesh = MeshResource.generateBox(size: [cuboidWidth, cuboidHeight, cuboidDepth])
        cuboidEntity = ModelEntity(mesh: boxMesh,
                                   materials: [SimpleMaterial(color: .green,
                                                             roughness: 0.3,
                                                             isMetallic: false)])
        // Anchor to the camera so it moves with the device
        let anchor = AnchorEntity(.camera)
        anchor.addChild(cuboidEntity!)
        cuboidEntity?.setPosition([0, -cuboidHeight/2, -1.5], relativeTo: anchor)
        arView.scene.addAnchor(anchor)
    }

    private func checkIntersection() {
        guard let arView = arView, let cuboidEntity = cuboidEntity else { return }
        let meshAnchors = arView.session.currentFrame?.anchors.compactMap { $0 as? ARMeshAnchor } ?? []
        let cuboidBounds = cuboidEntity.visualBounds(relativeTo: nil)
        var found = false

        for meshAnchor in meshAnchors {
            let geo = meshAnchor.geometry
            let buffer = geo.vertices.buffer
            let stride = geo.vertices.stride
            let offset = Int(geo.vertices.offset)

            for i in 0..<geo.vertices.count {
                let ptr = buffer.contents().advanced(by: offset + i * stride)
                let vertex = ptr.bindMemory(to: SIMD3<Float>.self, capacity: 1).pointee
                let worldPos = meshAnchor.transform * simd_float4(vertex, 1)
                if cuboidBounds.contains(SIMD3(worldPos.x, worldPos.y, worldPos.z)) {
                    found = true
                    break
                }
            }
            if found { break }
        }

        if found != isIntersecting {
            isIntersecting = found
            let color = found ? UIColor.red.withAlphaComponent(0.6)
                              : UIColor.green.withAlphaComponent(0.6)
            cuboidEntity.model?.materials = [SimpleMaterial(color: color,
                                                             roughness: 0.3,
                                                             isMetallic: false)]
            intersectionCallback?(found)
        }
    }
}
