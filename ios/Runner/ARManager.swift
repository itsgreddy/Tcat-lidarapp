import ARKit
import RealityKit
import Combine

// Occupancy grid cell state
enum CellState: Int {
    case unknown = 0
    case free = 1
    case occupied = 2
}

// A* node for pathfinding
class PathNode: Hashable, Comparable {
    var position: (Int, Int)
    var gScore: Float = Float.infinity
    var fScore: Float = Float.infinity
    var cameFrom: PathNode?
    
    init(position: (Int, Int)) {
        self.position = position
    }
    
    static func == (lhs: PathNode, rhs: PathNode) -> Bool {
        return lhs.position.0 == rhs.position.0 && lhs.position.1 == rhs.position.1
    }
    
    func hash(into hasher: inout Hasher) {
        hasher.combine(position.0)
        hasher.combine(position.1)
    }
    
    static func < (lhs: PathNode, rhs: PathNode) -> Bool {
        return lhs.fScore < rhs.fScore
    }
}

class ARManager: NSObject {
    var arView: ARView?
    private var updateSubscription: Cancellable?

    private var cuboidEntity: ModelEntity?
    private var cuboidWidth: Float = 0.7
    private var cuboidHeight: Float = 1.5
    private var cuboidDepth: Float = 1.2
    private var cuboidBuffer: Float = 0.1  // Additional buffer around the cuboid

    private var cuboidPositionX: Float = 0.0
    private var cuboidPositionY: Float = 0.0
    private var cuboidPositionZ: Float = -1.5

    private var isIntersecting = false
    private var intersectionCallback: ((Bool) -> Void)?

    private var anchorEntity: AnchorEntity? // track current anchor
    
    // Path planning properties
    private var occupancyGrid: [[CellState]] = []
    private var gridResolution: Float = 0.05 // 5cm resolution
    private var gridOrigin: SIMD3<Float> = SIMD3<Float>(0, 0, 0)
    private var gridSize: (width: Int, height: Int) = (0, 0)
    private var startPoint: SIMD3<Float>?
    private var goalPoint: SIMD3<Float>?
    private var currentPath: [SIMD3<Float>] = []
    private var pathEntity: AnchorEntity? // Changed from Entity to AnchorEntity
    private var pathMaterial = SimpleMaterial(color: .green, roughness: 0.3, isMetallic: false)
    private var groundPlaneY: Float?
    private var pathUpdateTimer: Timer?

    // MARK: - Performance Improvements

    // Update path update timer settings
    private var pathUpdateInterval: TimeInterval = 2.0 // Increase to 2 seconds (was 1.0)
    private var lastPathValidationTime: TimeInterval = 0
    private var validationThrottleInterval: TimeInterval = 1.0 

    // Update mesh processing settings
    private var lastGridUpdateTime: TimeInterval = 0
    private var gridUpdateInterval: TimeInterval = 1.0 // Increase to 1.0 seconds (was 0.5)
    private var lastCollisionCheckTime: TimeInterval = 0
    private var collisionCheckInterval: TimeInterval = 0.2 // Increase to 0.2 seconds (was 0.1)
    private var isProcessingPath: Bool = false

    // Path animation properties
    private var isFollowingPath: Bool = false
    private var pathAnimationTimer: Timer?
    private var currentPathIndex: Int = 0
    private var pathCompletionCallback: ((Bool) -> Void)?

    // Add collision side detection properties
    private enum CuboidSide: String {
        case front, back, left, right, top, bottom, none
    }

    private var collisionSides: Set<CuboidSide> = []
    private var lastReportedCollisionSide: CuboidSide = .none

    private let arConfiguration: ARWorldTrackingConfiguration = {
        let configuration = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            configuration.sceneReconstruction = .mesh
        }
        configuration.planeDetection = [.horizontal]
        return configuration
    }()

    private var groundVisualizer: AnchorEntity?

    func initialize(arView: ARView, callback: @escaping (Bool) -> Void) {
        guard ARWorldTrackingConfiguration.isSupported else { return }
        self.arView = arView
        self.intersectionCallback = callback

        arView.session.run(arConfiguration)
        createCuboid()
        
        // Start by detecting ground plane
        detectGroundPlane()

        // Subscribe to scene updates for real-time intersection checks
        updateSubscription = arView.scene.subscribe(to: SceneEvents.Update.self) { [weak self] _ in
            self?.throttledCheckIntersection()
        }
        
        // Reduce frequency of path update timer to improve performance
        pathUpdateTimer = Timer.scheduledTimer(withTimeInterval: pathUpdateInterval, repeats: true) { [weak self] _ in
            guard let self = self else { return }
            
            // Add throttling check
            let currentTime = Date().timeIntervalSince1970
            guard currentTime - self.lastPathValidationTime >= self.validationThrottleInterval else { return }
            self.lastPathValidationTime = currentTime
            
            // Do validation on a background queue
            DispatchQueue.global(qos: .utility).async {
                self.validatePath()
            }
        }
    }
    
    private func detectGroundPlane() {
        // Wait for horizontal plane detection in ARKit
        if let arView = arView {
            DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) { [weak self] in
                let planes = arView.session.currentFrame?.anchors.compactMap { $0 as? ARPlaneAnchor } ?? []
                if let floorPlane = planes.filter({ $0.alignment == .horizontal }).first {
                    self?.groundPlaneY = floorPlane.transform.columns.3.y
                    self?.initializeOccupancyGrid()
                } else {
                    self?.detectGroundPlane() // Try again if no plane found
                }
            }
        }
    }
    
    private func initializeOccupancyGrid() {
        guard let frame = arView?.session.currentFrame else { return }
        
        // Create a grid sized around the current camera view
        let cameraTransform = frame.camera.transform
        let cameraPosition = SIMD3<Float>(cameraTransform.columns.3.x,
                                         cameraTransform.columns.3.y,
                                         cameraTransform.columns.3.z)
        
        // Establish grid origin in world space - 10m x 10m grid centered around camera
        let gridWorldSize: Float = 10.0 // 10 meters
        gridOrigin = SIMD3<Float>(cameraPosition.x - gridWorldSize/2,
                                 groundPlaneY ?? 0.0,
                                 cameraPosition.z - gridWorldSize/2)
        
        // Calculate grid dimensions
        gridSize.width = Int(gridWorldSize / gridResolution)
        gridSize.height = Int(gridWorldSize / gridResolution)
        
        // Initialize all cells as unknown
        occupancyGrid = Array(repeating: Array(repeating: .unknown, count: gridSize.height), count: gridSize.width)
        
        // Update grid with initial scan data
        updateOccupancyGrid()
    }
    
    private func updateOccupancyGrid() {
        guard let groundY = groundPlaneY,
              let frame = arView?.session.currentFrame else { return }
        
        // Throttle grid updates to prevent frequent main thread blocking
        let currentTime = Date().timeIntervalSince1970
        if currentTime - lastGridUpdateTime < gridUpdateInterval {
            return
        }
        lastGridUpdateTime = currentTime
        
        // Get all mesh anchors
        let meshAnchors = frame.anchors.compactMap { $0 as? ARMeshAnchor }
        
        // Create a temporary grid for new obstacles
        var newObstacles = Array(repeating: Array(repeating: false, count: gridSize.height), count: gridSize.width)
        
        // Process mesh vertices and update grid
        for meshAnchor in meshAnchors {
            let vertexStride = meshAnchor.geometry.vertices.stride
            let buffer = meshAnchor.geometry.vertices.buffer
            let offset = Int(meshAnchor.geometry.vertices.offset)
            let anchorTransform = meshAnchor.transform
            
            // Process even fewer vertices (every 10th instead of every 5th)
            for i in stride(from: 0, to: meshAnchor.geometry.vertices.count, by: 10) {
                let ptr = buffer.contents().advanced(by: offset + i * vertexStride)
                let vertex = ptr.bindMemory(to: SIMD3<Float>.self, capacity: 1).pointee
                let worldVertex = anchorTransform * simd_float4(vertex, 1)
                let worldPoint = SIMD3<Float>(worldVertex.x, worldVertex.y, worldVertex.z)
                
                // Check if vertex is an obstacle (< 0.4m above ground)
                let height = worldPoint.y - groundY
                let isObstacle = height < 0.4 && height > 0.02
                
                if isObstacle {
                    // Convert world point to grid coordinates
                    let gridX = Int((worldPoint.x - gridOrigin.x) / gridResolution)
                    let gridZ = Int((worldPoint.z - gridOrigin.z) / gridResolution)
                    
                    // Check if within grid bounds
                    if gridX >= 0 && gridX < gridSize.width && gridZ >= 0 && gridZ < gridSize.height {
                        // Mark obstacle and buffer zone
                        let bufferCells = Int(ceil((cuboidWidth/2 + cuboidBuffer) / gridResolution))
                        for dx in -bufferCells...bufferCells {
                            for dz in -bufferCells...bufferCells {
                                let nx = gridX + dx
                                let nz = gridZ + dz
                                if nx >= 0 && nx < gridSize.width && nz >= 0 && nz < gridSize.height {
                                    newObstacles[nx][nz] = true
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // Update the main grid with new obstacles
        for x in 0..<gridSize.width {
            for z in 0..<gridSize.height {
                if newObstacles[x][z] {
                    occupancyGrid[x][z] = .occupied
                } else if occupancyGrid[x][z] != .occupied {
                    occupancyGrid[x][z] = .free
                }
            }
        }
    }

    func setPathPoints(start: SIMD3<Float>, goal: SIMD3<Float>) {
        // Add debug logging to troubleshoot
        print("ARManager received path points:")
        print("  - Start: \(start)")
        print("  - Goal: \(goal)")
        
        // Check if both points have valid positions
        if start == SIMD3<Float>(0, 0, 0) || goal == SIMD3<Float>(0, 0, 0) {
            print("Warning: Invalid path points (0,0,0)")
            return
        }
        
        // Additional validation to ensure we don't have NaN values
        if start.x.isNaN || start.y.isNaN || start.z.isNaN ||
           goal.x.isNaN || goal.y.isNaN || goal.z.isNaN {
            print("Warning: Path points contain NaN values")
            return
        }
        
        startPoint = start
        goalPoint = goal
        
        // Remove any existing path
        if let pathEntity = pathEntity {
            arView?.scene.removeAnchor(pathEntity) // Now works with AnchorEntity
            self.pathEntity = nil
        }
        
        // Plan the path
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            self?.planPath()
        }
    }
    
    private func planPath() {
        guard !isProcessingPath else { return } // Prevent concurrent path planning
        isProcessingPath = true
        
        guard let start = startPoint, let goal = goalPoint else { 
            isProcessingPath = false
            return 
        }
        
        // Update the occupancy grid
        updateOccupancyGrid()
        
        // First attempt a straight line (convex hull heuristic)
        let straightLinePath = [start, goal]
        
        // Check if the straight path is valid
        if isPathValid(straightLinePath) {
            currentPath = straightLinePath
            DispatchQueue.main.async { [weak self] in
                self?.visualizePath(path: straightLinePath, isBlocked: false)
                self?.isProcessingPath = false
            }
            return
        }
        
        // If straight path fails, use A* algorithm with computation limits
        if let astarPath = findAStar(from: start, to: goal, maxIterations: 1000) {
            let thinned = thinPath(astarPath)
            currentPath = thinned
            DispatchQueue.main.async { [weak self] in
                self?.visualizePath(path: thinned, isBlocked: false)
                self?.isProcessingPath = false
            }
        } else {
            // No path found - visualize the blocked straight path
            currentPath = straightLinePath
            DispatchQueue.main.async { [weak self] in
                self?.visualizePath(path: straightLinePath, isBlocked: true)
                self?.isProcessingPath = false
            }
        }
    }
    
    // MARK: - Performance Improvements

    private func findAStar(from startWorld: SIMD3<Float>, to goalWorld: SIMD3<Float>, maxIterations: Int = 500) -> [SIMD3<Float>]? {
        // Convert world coordinates to grid coordinates
        let startGridX = Int((startWorld.x - gridOrigin.x) / gridResolution)
        let startGridZ = Int((startWorld.z - gridOrigin.z) / gridResolution)
        let goalGridX = Int((goalWorld.x - gridOrigin.x) / gridResolution)
        let goalGridZ = Int((goalWorld.z - gridOrigin.z) / gridResolution)
        
        // Check if start and goal are within grid bounds
        guard startGridX >= 0 && startGridX < gridSize.width && startGridZ >= 0 && startGridZ < gridSize.height &&
              goalGridX >= 0 && goalGridX < gridSize.width && goalGridZ >= 0 && goalGridZ < gridSize.height else {
            return nil
        }
        
        // Check if start or goal are occupied
        if occupancyGrid[startGridX][startGridZ] == .occupied || occupancyGrid[goalGridX][goalGridZ] == .occupied {
            return nil
        }
        
        // Create nodes
        let startNode = PathNode(position: (startGridX, startGridZ))
        let goalNode = PathNode(position: (goalGridX, goalGridZ))
        
        // Initialize scores
        startNode.gScore = 0
        startNode.fScore = heuristic(from: startNode.position, to: goalNode.position)
        
        // Open set (nodes to evaluate)
        var openSet = Set<PathNode>()
        openSet.insert(startNode)
        
        // Grid of nodes for lookup
        var nodeGrid = Array(repeating: Array<PathNode?>(repeating: nil, count: gridSize.height), count: gridSize.width)
        nodeGrid[startGridX][startGridZ] = startNode
        
        // A* main loop with iteration limit
        var iterations = 0
        while !openSet.isEmpty && iterations < maxIterations {
            iterations += 1
            // Find the node with the lowest fScore
            let current = openSet.min(by: { $0.fScore < $1.fScore })!
            openSet.remove(current)
            
            // Check if we reached the goal
            if current.position == goalNode.position {
                return reconstructPath(current)
            }
            
            // Examine neighbors
            let directions = [
                (0, 1), (1, 0), (0, -1), (-1, 0),  // Cardinal directions
                (1, 1), (-1, 1), (1, -1), (-1, -1) // Diagonal directions
            ]
            
            for (dx, dz) in directions {
                let neighborX = current.position.0 + dx
                let neighborZ = current.position.1 + dz
                
                // Check bounds
                guard neighborX >= 0 && neighborX < gridSize.width && neighborZ >= 0 && neighborZ < gridSize.height else {
                    continue
                }
                
                // Skip occupied cells
                if occupancyGrid[neighborX][neighborZ] == .occupied {
                    continue
                }
                
                // Create or get neighbor node
                let neighbor: PathNode
                if let existingNode = nodeGrid[neighborX][neighborZ] {
                    neighbor = existingNode
                } else {
                    neighbor = PathNode(position: (neighborX, neighborZ))
                    nodeGrid[neighborX][neighborZ] = neighbor
                }
                
                // Calculate tentative gScore (diagonal movement costs more)
                let moveCost: Float = (dx != 0 && dz != 0) ? 1.414 : 1.0  // √2 for diagonal
                let tentativeGScore = current.gScore + moveCost
                
                // If this path is better than previous
                if tentativeGScore < neighbor.gScore {
                    neighbor.cameFrom = current
                    neighbor.gScore = tentativeGScore
                    neighbor.fScore = tentativeGScore + heuristic(from: neighbor.position, to: goalNode.position)
                    
                    if !openSet.contains(neighbor) {
                        openSet.insert(neighbor)
                    }
                }
            }
        }
        
        // No path found or hit iteration limit
        return nil
    }
    
    private func heuristic(from: (Int, Int), to: (Int, Int)) -> Float {
        let dx = Float(to.0 - from.0)
        let dy = Float(to.1 - from.1)
        return sqrt(dx*dx + dy*dy)
    }
    
    private func reconstructPath(_ goalNode: PathNode) -> [SIMD3<Float>] {
        var path: [SIMD3<Float>] = []
        var current = goalNode
        
        // Make sure ground Y level is used
        let yPosition = groundPlaneY ?? gridOrigin.y
        
        while let node = current.cameFrom {
            // Convert grid coordinates back to world coordinates
            let worldX = gridOrigin.x + (Float(current.position.0) + 0.5) * gridResolution
            let worldZ = gridOrigin.z + (Float(current.position.1) + 0.5) * gridResolution
            
            // Use ground level for Y position - no additional offset
            path.append(SIMD3<Float>(worldX, yPosition, worldZ))
            current = node
        }
        
        // Add start point
        let worldX = gridOrigin.x + (Float(current.position.0) + 0.5) * gridResolution
        let worldZ = gridOrigin.z + (Float(current.position.1) + 0.5) * gridResolution
        path.append(SIMD3<Float>(worldX, yPosition, worldZ))
        
        // Reverse to get start->goal order
        return path.reversed()
    }
    
    private func thinPath(_ path: [SIMD3<Float>]) -> [SIMD3<Float>] {
        if path.count <= 2 {
            return path
        }
        
        var result: [SIMD3<Float>] = [path[0]]
        var i = 1
        
        while i < path.count - 1 {
            let prev = result.last!
            let next = path[i+1]
            
            if lineOfSight(from: prev, to: next) {
                i += 1
            } else {
                result.append(path[i])
                i += 1
            }
        }
        
        result.append(path.last!)
        return result
    }
    
    private func lineOfSight(from start: SIMD3<Float>, to end: SIMD3<Float>) -> Bool {
        let startGridX = Int((start.x - gridOrigin.x) / gridResolution)
        let startGridZ = Int((start.z - gridOrigin.z) / gridResolution)
        let endGridX = Int((end.x - gridOrigin.x) / gridResolution)
        let endGridZ = Int((end.z - gridOrigin.z) / gridResolution)
        
        var x = startGridX
        var z = startGridZ
        let dx = abs(endGridX - startGridX)
        let dz = abs(endGridZ - startGridZ)
        let sx = startGridX < endGridX ? 1 : -1
        let sz = startGridZ < endGridZ ? 1 : -1
        var err = dx - dz
        
        while x != endGridX || z != endGridZ {
            if x >= 0 && x < gridSize.width && z >= 0 && z < gridSize.height {
                if occupancyGrid[x][z] == .occupied {
                    return false
                }
            }
            
            let e2 = 2 * err
            if e2 > -dz {
                err -= dz
                x += sx
            }
            if e2 < dx {
                err += dx
                z += sz
            }
        }
        
        return true
    }
    
    private func visualizePath(path: [SIMD3<Float>], isBlocked: Bool) {
        if let pathEntity = pathEntity {
            arView?.scene.removeAnchor(pathEntity)
        }
        
        guard let arView = arView, path.count >= 2 else { 
            print("Cannot visualize path: invalid path or no ARView")
            return 
        }
        
        // Debug print path points to help diagnose issues
        print("Visualizing path with \(path.count) points")
        
        // Use the actual Y coordinates of the path points - don't force to ground level
        let pathPoints = path
        
        let lineSegments = createPath(points: pathPoints)
        
        // Make path much more visible with stronger colors and translucency
        let color = isBlocked ? 
            UIColor(red: 1.0, green: 0.2, blue: 0.2, alpha: 0.9) :  // Bright red
            UIColor(red: 0.2, green: 1.0, blue: 0.2, alpha: 0.9)    // Bright green
            
        let material = SimpleMaterial(
            color: color,
            roughness: 0.1,
            isMetallic: true
        )
        
        let pathMeshEntity = ModelEntity(mesh: lineSegments, materials: [material])
        
        // Add glow effect by slightly scaling the path
        pathMeshEntity.scale = [1.05, 1.05, 1.05]
        
        // Ensure the path is anchored in world space not camera space
        let anchor = AnchorEntity(world: .zero)
        anchor.addChild(pathMeshEntity)
        
        // No additional height adjustment needed here now
        
        arView.scene.addAnchor(anchor)
        pathEntity = anchor
        pathMaterial = material
        
        print("Path visualization complete")
    }

    private func createPath(points: [SIMD3<Float>]) -> MeshResource {
        // Make the path thicker for better 3D visibility
        let lineWidth: Float = 0.05  // 5cm radius (10cm diameter)
        
        var vertices: [SIMD3<Float>] = []
        var triangleIndices: [UInt32] = []
        var normals: [SIMD3<Float>] = []
        var vertexIndex: UInt32 = 0
        
        // Create a 3D tube along the path
        let numSides = 8  // Number of sides for our tube (octagonal)
        
        for i in 0..<points.count-1 {
            let start = points[i]
            let end = points[i+1]
            
            // Create direction vector
            let direction = normalize(end - start)
            let distance = length(end - start)
            
            // Create perpendicular vectors for the tube cross-section
            let up = SIMD3<Float>(0, 1, 0)
            var right = normalize(cross(direction, up))
            if length(right) < 0.1 {
                // If direction is parallel to up, use a different reference vector
                right = SIMD3<Float>(1, 0, 0)
            }
            let upVector = normalize(cross(right, direction))
            
            // Create vertices for start and end rings
            for point in [start, end] {
                let baseIndex = vertices.count
                
                // Create a ring of vertices around this point
                for s in 0..<numSides {
                    let angle = Float(s) * (2.0 * Float.pi / Float(numSides))
                    
                    // Calculate position around the tube
                    let xOffset = cos(angle) * lineWidth
                    let yOffset = sin(angle) * lineWidth
                    
                    let vertex = point + right * xOffset + upVector * yOffset
                    vertices.append(vertex)
                    
                    // Outward-pointing normal
                    let normal = normalize(SIMD3<Float>(xOffset, yOffset, 0))
                    normals.append(normal)
                }
                
                // If this is not the first segment, connect to previous ring
                if i > 0 || point != start {
                    let prevRingStart = baseIndex - numSides
                    
                    for s in 0..<numSides {
                        let nextS = (s + 1) % numSides
                        
                        // Add two triangles to form a quad
                        triangleIndices.append(UInt32(prevRingStart + s))
                        triangleIndices.append(UInt32(baseIndex + s))
                        triangleIndices.append(UInt32(baseIndex + nextS))
                        
                        triangleIndices.append(UInt32(prevRingStart + s))
                        triangleIndices.append(UInt32(baseIndex + nextS))
                        triangleIndices.append(UInt32(prevRingStart + nextS))
                    }
                }
            }
        }
        
        // Create mesh descriptor with positions and normals
        var meshDescriptor = MeshDescriptor()
        meshDescriptor.positions = MeshBuffer(vertices)
        meshDescriptor.normals = MeshBuffer(normals)
        meshDescriptor.primitives = .triangles(triangleIndices)
        
        // Handle potential mesh generation errors
        do {
            return try MeshResource.generate(from: [meshDescriptor])
        } catch {
            print("Error generating path mesh: \(error)")
            
            // Create a simple fallback mesh
            return MeshResource.generateBox(size: 0.05)
        }
    }

    private func validatePath() {
        guard !currentPath.isEmpty, let start = startPoint, let goal = goalPoint else { return }
        
        // Skip validation if currently following the path
        if isFollowingPath {
            return
        }
        
        // Update grid less often by calling this inside validatePath
        DispatchQueue.global(qos: .utility).async { [weak self] in
            guard let self = self else { return }
            self.updateOccupancyGrid()
            
            if !self.isPathValid(self.currentPath) {
                DispatchQueue.main.async {
                    self.visualizePath(path: self.currentPath, isBlocked: true)
                }
                
                // Attempt to find new path in the background
                if let newPath = self.findAStar(from: start, to: goal) {
                    let thinned = self.thinPath(newPath)
                    self.currentPath = thinned
                    
                    DispatchQueue.main.async {
                        self.visualizePath(path: thinned, isBlocked: false)
                      }
                }
            }
        }
    }
    
    private func isPathValid(_ path: [SIMD3<Float>]) -> Bool {
        // Early exit if path is only one point or empty
        if path.count <= 1 {
            return true
        }
        
        // Optimize by only checking every other segment in long paths
        let strideValue = path.count > 10 ? 2 : 1  // Changed variable name from 'stride' to 'strideValue'
        
        for i in Swift.stride(from: 0, to: path.count-1, by: strideValue) {  // Use Swift.stride to disambiguate
            if !lineOfSight(from: path[i], to: path[i+1]) {
                // Calculate path direction to determine which side is likely blocked
                if path.count >= 2 {
                    let pathDirection = normalize(path[1] - path[0])
                    
                    // Determine which side is most likely blocked based on path direction
                    if abs(pathDirection.z) > abs(pathDirection.x) {
                        // Path is primarily along Z axis
                        if pathDirection.z > 0 {
                            lastReportedCollisionSide = .front
                        } else {
                            lastReportedCollisionSide = .back
                        }
                    } else {
                        // Path is primarily along X axis
                        if pathDirection.x > 0 {
                            lastReportedCollisionSide = .right
                        } else {
                            lastReportedCollisionSide = .left
                        }
                    }
                    
                    print("Path blocked on side: \(lastReportedCollisionSide.rawValue)")
                }
                return false
            }
        }
        return true
    }
    
    // Add getter for collision info
    func getLastCollisionSide() -> String {
        return lastReportedCollisionSide.rawValue
    }
    
    func exportPathToGeoJSON() -> String? {
        guard !currentPath.isEmpty else { return nil }
        
        var geojson = """
        {
          "type": "FeatureCollection",
          "features": [
            {
              "type": "Feature",
              "geometry": {
                "type": "LineString",
                "coordinates": [
        """
        
        let coordStrings = currentPath.map { point in
            "        [\(point.x), \(point.z), \(point.y)]"
        }
        geojson += coordStrings.joined(separator: ",\n")
        
        geojson += """
                ]
              },
              "properties": {
                "name": "accessibility_path",
                "clearance_width": \(cuboidWidth),
                "clearance_height": \(cuboidHeight)
              }
            }
          ]
        }
        """
        
        return geojson
    }
    
    func setCuboid(width: Float, height: Float, depth: Float) {
        cuboidWidth = width / 100.0
        cuboidHeight = height / 100.0
        cuboidDepth = depth / 100.0
        
        updateCuboidDimensions(width: width, height: height, depth: depth)
        
        if let start = startPoint, let goal = goalPoint {
            setPathPoints(start: start, goal: goal)
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

    func updateCuboidPosition(x: Float, y: Float, z: Float) {
        // Store previous position for calculating deltas
        let prevX = cuboidPositionX
        let prevY = cuboidPositionY
        let prevZ = cuboidPositionZ
        
        // Update stored position values
        cuboidPositionX = x
        cuboidPositionY = y
        cuboidPositionZ = z
        
        if let cuboid = cuboidEntity {
            if let anchor = anchorEntity {
                // If locked to world, update position with reduced movement
                if anchor.anchoring.target != .camera {
                    // Calculate position change with reduced movement
                    let worldSpaceScaleFactor: Float = 0.1 // Scale down movements by 10x when locked
                    
                    // Apply scaled deltas to current world position
                    let deltaX = (x - prevX) * worldSpaceScaleFactor
                    let deltaY = (y - prevY) * worldSpaceScaleFactor
                    let deltaZ = (z - prevZ) * worldSpaceScaleFactor
                    
                    // Apply incremental changes to the anchor position
                    anchor.transform.translation += SIMD3<Float>(deltaX, deltaY, deltaZ)
                } else {
                    // When attached to camera, update relative to camera
                    cuboid.setPosition(SIMD3<Float>(x, y - cuboidHeight/2, z), relativeTo: anchor)
                }
            }
        }
    }

    func cleanup() {
        pathUpdateTimer?.invalidate()
        pathUpdateTimer = nil
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
        if let existing = anchorEntity {
            if let entity = cuboidEntity {
                existing.removeChild(entity)
            }
            arView.scene.removeAnchor(existing)
        }
        let boxMesh = MeshResource.generateBox(size: [cuboidWidth, cuboidHeight, cuboidDepth])
        cuboidEntity = ModelEntity(mesh: boxMesh,
                                   materials: [SimpleMaterial(color: .green,
                                                             roughness: 0.3,
                                                             isMetallic: false)])
        let camAnchor = AnchorEntity(.camera)
        camAnchor.addChild(cuboidEntity!)
        cuboidEntity!.setPosition([cuboidPositionX, cuboidPositionY - cuboidHeight/2, cuboidPositionZ], relativeTo: camAnchor)
        arView.scene.addAnchor(camAnchor)
        anchorEntity = camAnchor
    }

    func lockCuboid() {
        guard let arView = arView,
              let entity = cuboidEntity,
              let oldAnchor = anchorEntity else { return }
        
        // Get current world position
        let worldPos = entity.position(relativeTo: nil)
        
        // Clean up old anchor
        oldAnchor.removeChild(entity)
        arView.scene.removeAnchor(oldAnchor)
        
        // Create new world anchor at the current position
        let worldAnchor = AnchorEntity()
        worldAnchor.transform.translation = worldPos
        worldAnchor.addChild(entity)
        entity.transform = Transform()
        
        arView.scene.addAnchor(worldAnchor)
        anchorEntity = worldAnchor
        
        // Reset slider position values to zero since we're now working with deltas
        // in world space mode rather than absolute positions
        cuboidPositionX = 0
        cuboidPositionY = 0
        cuboidPositionZ = 0
    }

    func unlockCuboid() {
        guard let arView = arView,
              let entity = cuboidEntity,
              let oldAnchor = anchorEntity else { return }
        let worldPos = entity.position(relativeTo: nil)
        oldAnchor.removeChild(entity)
        arView.scene.removeAnchor(oldAnchor)
        let camAnchor = AnchorEntity(.camera)
        camAnchor.addChild(entity)
        entity.setPosition(worldPos, relativeTo: camAnchor)
        arView.scene.addAnchor(camAnchor)
        anchorEntity = camAnchor
    }

    private func throttledCheckIntersection() {
        let currentTime = Date().timeIntervalSince1970
        if currentTime - lastCollisionCheckTime >= collisionCheckInterval {
            lastCollisionCheckTime = currentTime
            
            // Move collision check to background thread
            DispatchQueue.global(qos: .userInteractive).async { [weak self] in
                self?.checkIntersection()
            }
        }
    }

    private func checkIntersection() {
        guard let arView = arView, let cuboidEntity = cuboidEntity else { return }
        let meshAnchors = arView.session.currentFrame?.anchors.compactMap { $0 as? ARMeshAnchor } ?? []
        let cuboidBounds = cuboidEntity.visualBounds(relativeTo: nil)
        var found = false
        
        // Reset collision sides
        collisionSides.removeAll()
        
        // Get cuboid transform in world space
        let cuboidTransform = cuboidEntity.transformMatrix(relativeTo: nil)
        let cuboidPosition = cuboidEntity.position(relativeTo: nil)
        
        // Get cuboid dimensions and half-extents
        let halfWidth = cuboidWidth / 2
        let halfHeight = cuboidHeight / 2
        let halfDepth = cuboidDepth / 2
        
        // Sample fewer vertices for more efficient collision detection
        for meshAnchor in meshAnchors {
            let geo = meshAnchor.geometry
            let buffer = geo.vertices.buffer
            let vertexStride = geo.vertices.stride
            let offset = Int(geo.vertices.offset)

            // Sample every 12th vertex instead of every 8th
            for i in Swift.stride(from: 0, to: geo.vertices.count, by: 12) {
                let ptr = buffer.contents().advanced(by: offset + i * vertexStride)
                let vertex = ptr.bindMemory(to: SIMD3<Float>.self, capacity: 1).pointee
                let worldPos = meshAnchor.transform * simd_float4(vertex, 1)
                let worldPoint = SIMD3(worldPos.x, worldPos.y, worldPos.z)
                
                // Check if this point is inside the cuboid bounds
                if cuboidBounds.contains(worldPoint) {
                    found = true
                    
                    // Determine which side(s) of the cuboid this point is closest to
                    let relativePoint = worldPoint - cuboidPosition
                    
                    // Convert to cuboid's local space
                    let localX = abs(relativePoint.x)
                    let localY = abs(relativePoint.y)
                    let localZ = abs(relativePoint.z)
                    
                    // Calculate distance to each face as a percentage of half-extent
                    let distToRight = halfWidth - localX
                    let distToLeft = halfWidth + localX
                    let distToTop = halfHeight - localY
                    let distToBottom = halfHeight + localY
                    let distToFront = halfDepth - localZ
                    let distToBack = halfDepth + localZ
                    
                    // Find the closest face - the one with smallest distance
                    let minDist = min(distToRight, min(distToLeft, min(distToTop, min(distToBottom, min(distToFront, distToBack)))))
                    
                    if minDist == distToRight { collisionSides.insert(.right) }
                    else if minDist == distToLeft { collisionSides.insert(.left) }
                    else if minDist == distToTop { collisionSides.insert(.top) }
                    else if minDist == distToBottom { collisionSides.insert(.bottom) }
                    else if minDist == distToFront { collisionSides.insert(.front) }
                    else if minDist == distToBack { collisionSides.insert(.back) }
                }
            }
            
            if found { break }
        }

        // Report collision status on main thread
        if found != isIntersecting {
            isIntersecting = found
            let color = found ? UIColor.red.withAlphaComponent(0.6) : UIColor.green.withAlphaComponent(0.6)
            
            DispatchQueue.main.async { [weak self] in
                guard let self = self else { return }
                self.cuboidEntity?.model?.materials = [SimpleMaterial(
                    color: color,
                    roughness: 0.3,
                    isMetallic: false
                )]
                
                // Report collision with sides info
                if found && !self.collisionSides.isEmpty {
                    self.lastReportedCollisionSide = self.collisionSides.first ?? .none
                    print("Collision detected on side: \(self.lastReportedCollisionSide.rawValue)")
                }
                
                self.intersectionCallback?(found)
            }
        }
    }

    func followPath(completion: @escaping (Bool) -> Void) {
        guard !currentPath.isEmpty, !isFollowingPath else {
            completion(false)
            return
        }
        
        // Cancel any existing animation
        pathAnimationTimer?.invalidate()
        
        // Set up for new animation
        isFollowingPath = true
        currentPathIndex = 0
        pathCompletionCallback = completion
        
        // If we're starting from a camera-relative cuboid, let's lock it first
        if let anchor = anchorEntity, anchor.anchoring.target == .camera {
            lockCuboid()
        }
        
        // ALWAYS teleport the cuboid to the beginning of the path, regardless of where it currently is
        if let startPoint = currentPath.first {
            // Place cuboid at start position
            // Calculate Y position considering cuboid height - use actual Y coordinate
            let cuboidY = startPoint.y + (cuboidHeight / 2) // Set Y so bottom is at start point Y
            
            print("Moving cuboid to path start position: \(startPoint.x), \(cuboidY), \(startPoint.z)")
            moveToWorldPosition(SIMD3<Float>(startPoint.x, cuboidY, startPoint.z))
            
            // Check if we're already intersecting at the start
            if isIntersecting {
                // Can't move, we're already blocked
                isFollowingPath = false
                pathCompletionCallback?(false)
                return
            }
            
            // Start animation timer with a brief delay to ensure everything is positioned correctly
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.2) { [weak self] in
                self?.pathAnimationTimer = Timer.scheduledTimer(withTimeInterval: 0.05, repeats: true) { [weak self] _ in
                    self?.animateAlongPath()
                }
            }
        } else {
            // No valid start point
            isFollowingPath = false
            pathCompletionCallback?(false)
        }
    }

    private func animateAlongPath() {
        guard isFollowingPath, currentPathIndex < currentPath.count - 1 else {
            // We've reached the end of the path
            stopPathAnimation(success: true)
            return
        }
        
        // Get current and next waypoint
        let currentWaypoint = currentPath[currentPathIndex]
        let nextWaypoint = currentPath[currentPathIndex + 1]
        
        // Calculate step size (5cm per step)
        let stepSize: Float = 0.05
        let pathVector = nextWaypoint - currentWaypoint
        let pathDirection = normalize(pathVector)
        let stepVector = pathDirection * stepSize
        
        // Calculate new position
        let currentPos = cuboidEntity?.position(relativeTo: nil) ?? SIMD3<Float>(0, 0, 0)
        let distanceToNext = length(nextWaypoint - SIMD3<Float>(currentPos.x, nextWaypoint.y, currentPos.z))
        
        if distanceToNext < stepSize {
            // We've reached the next waypoint
            currentPathIndex += 1
            return
        }
        
        // Calculate new cuboid position - maintain current Y to stay grounded
        let newPosX = currentPos.x + stepVector.x
        let newPosZ = currentPos.z + stepVector.z
        
        // Move the cuboid, keeping Y position the same to stay grounded
        moveToWorldPosition(SIMD3<Float>(newPosX, currentPos.y, newPosZ))
        
        // Check for collision after moving
        if isIntersecting {
            // We hit an obstacle, stop animation
            stopPathAnimation(success: false)
        }
    }

    private func stopPathAnimation(success: Bool) {
        pathAnimationTimer?.invalidate()
        pathAnimationTimer = nil
        isFollowingPath = false
        
        // Reset path index to ensure next run starts from beginning
        currentPathIndex = 0
        
        // Call completion handler
        pathCompletionCallback?(success)
        pathCompletionCallback = nil
    }

    private func moveToWorldPosition(_ worldPosition: SIMD3<Float>) {
        guard let entity = cuboidEntity, let anchor = anchorEntity else { return }
        
        // Update the anchor's position in world space
        anchor.transform.translation = worldPosition
    }

    func cancelPathFollowing() {
        stopPathAnimation(success: false)   
    }
    
    // Add accessor method for groundPlaneY
    func getGroundPlaneY() -> Float? {
        return groundPlaneY
    }
    
    // Ground visualization functions
    private func showGroundVisualization() {
        guard let groundY = groundPlaneY, groundVisualizer == nil, let arView = arView else { return }
        
        // Create a grid material
        let material = SimpleMaterial(
            color: UIColor.white.withAlphaComponent(0.3), 
            roughness: 0.4, 
            isMetallic: false
        )
        
        // Create a large flat grid (5m x 5m)
        let gridMesh = MeshResource.generatePlane(width: 5, depth: 5)
        let gridEntity = ModelEntity(mesh: gridMesh, materials: [material])
        
        // Place at ground level
        let anchor = AnchorEntity(world: [0, groundY + 0.001, 0]) // Slightly above ground to avoid z-fighting
        anchor.addChild(gridEntity)
        
        // Add grid lines
        for x in stride(from: -2.5, through: 2.5, by: 0.5) {
            let lineX = ModelEntity(
                mesh: MeshResource.generatePlane(width: 0.01, depth: 5),
                materials: [SimpleMaterial(color: .white.withAlphaComponent(0.5), roughness: 0.5, isMetallic: false)]
            )
            lineX.position = [Float(x), 0.002, 0]
            anchor.addChild(lineX)
            
            let lineZ = ModelEntity(
                mesh: MeshResource.generatePlane(width: 5, depth: 0.01),
                materials: [SimpleMaterial(color: .white.withAlphaComponent(0.5), roughness: 0.5, isMetallic: false)]
            )
            lineZ.position = [0, 0.002, Float(x)]
            anchor.addChild(lineZ)
        }
        
        arView.scene.addAnchor(anchor)
        groundVisualizer = anchor
    }
    
    func toggleGroundVisualization() -> Bool {
        if let visualizer = groundVisualizer, let arView = arView {
            arView.scene.removeAnchor(visualizer)
            groundVisualizer = nil
            return false // Visualization turned off
        } else {
            showGroundVisualization()
            return true // Visualization turned on
        }
    }
}
