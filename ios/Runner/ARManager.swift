import ARKit
import RealityKit

class ARManager: NSObject {
    // AR view for rendering
    var arView: ARView?
    
    // Cuboid entity
    private var cuboidEntity: ModelEntity?
    private var cuboidWidth: Float = 0.7 // 70cm default
    private var cuboidHeight: Float = 1.5 // 150cm default
    private var cuboidDepth: Float = 1.2 // 120cm default
    
    // Intersection state
    private var isIntersecting = false
    private var intersectionCallback: ((Bool) -> Void)?
    
    // Configuration
    private let arConfiguration: ARWorldTrackingConfiguration = {
        let configuration = ARWorldTrackingConfiguration()
        
        // Check if device supports LiDAR
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            configuration.sceneReconstruction = .mesh
        }
        
        configuration.planeDetection = [.horizontal, .vertical]
        return configuration
    }()
    
    // MARK: - Public Methods
    
    func initialize(arView: ARView, callback: @escaping (Bool) -> Void) {

        guard ARWorldTrackingConfiguration.isSupported else {
            print("ARWorldTrackingConfiguration not supported on this device.")
            return
        }

        self.arView = arView
        self.intersectionCallback = callback
        
        // Start AR session
        arView.session.run(arConfiguration)
        
        // Create cuboid
        createCuboid()
        
        // Set up intersection checking timer
        Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
            self?.checkIntersection()
        }
    }
    
    func updateCuboidDimensions(width: Float, height: Float, depth: Float) {
        cuboidWidth = width / 100 // Convert to meters
        cuboidHeight = height / 100
        cuboidDepth = depth / 100
        
        // Update cuboid size
        if let cuboidEntity = cuboidEntity {
            arView?.scene.removeAnchor(cuboidEntity.anchor!)
            createCuboid()
        }
    }
    
    func cleanup() {
        arView?.session.pause()
        cuboidEntity = nil
        arView = nil
    }
    
    // MARK: - Private Methods
    
    private func createCuboid() {
        guard let arView = arView else { return }
        
        // Create cuboid mesh with dimensions
        let boxMesh = MeshResource.generateBox(size: [cuboidWidth, cuboidHeight, cuboidDepth])
        let material = SimpleMaterial(color: .green, roughness: 0.3, isMetallic: true)
        cuboidEntity = ModelEntity(mesh: boxMesh, materials: [material])
        
        // Position the cuboid 1 meter in front of the camera
        let anchor = AnchorEntity(.camera)
        anchor.addChild(cuboidEntity!)
        cuboidEntity?.setPosition([0, -cuboidHeight/2, -1.5], relativeTo: anchor) // Position at floor level, 1.5m ahead
        
        // Make the cuboid semi-transparent for better visibility
        cuboidEntity?.model?.materials = [SimpleMaterial(
            color: .green.withAlphaComponent(0.6),
            roughness: 0.3,
            isMetallic: false
        )]
        
        arView.scene.addAnchor(anchor)
    }
    
    private func checkIntersection() {
        guard let arView = arView, let cuboidEntity = cuboidEntity else { return }
        
        // Get the current LiDAR mesh anchors from the session
        let meshAnchors = arView.session.currentFrame?.anchors.compactMap { $0 as? ARMeshAnchor }
        
        // Calculate the world bounds of our cuboid
        let cuboidBounds = cuboidEntity.visualBounds(relativeTo: nil)
        
        // Flag to track if any intersection is found
        var foundIntersection = false
        
        // Check each mesh anchor for intersection
        for anchor in meshAnchors ?? [] {
            // Get the mesh geometry
            let meshGeometry = anchor.geometry
            
            // Create vertices array for intersection testing
            var vertices: [SIMD3<Float>] = []
            for i in 0..<meshGeometry.vertices.count {
                let vertex = meshGeometry.vertices[i]
                // Transform vertex to world coordinates
                let vertexWorldPosition = anchor.transform * simd_float4(vertex, 1)
                vertices.append(SIMD3<Float>(vertexWorldPosition.x, vertexWorldPosition.y, vertexWorldPosition.z))
            }
            
            // Check if any vertex is inside our cuboid bounds
            for vertex in vertices {
                if cuboidBounds.contains(vertex) {
                    foundIntersection = true
                    break
                }
            }
            
            if foundIntersection {
                break
            }
        }
        
        // Update intersection state if changed
        if foundIntersection != isIntersecting {
            isIntersecting = foundIntersection
            
            // Update cuboid color based on intersection
            let newMaterial = SimpleMaterial(
                color: isIntersecting ? UIColor.red.withAlphaComponent(0.6) : UIColor.green.withAlphaComponent(0.6),
                roughness: 0.3,
                isMetallic: false
            )
            cuboidEntity.model?.materials = [newMaterial]
            
            // Call the callback
            intersectionCallback?(isIntersecting)
        }
    }
}
