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
    
    // K-paths visualization properties
    private var kPathEntities: [AnchorEntity] = []
    private var allKPaths: [[SIMD3<Float>]] = []
    private var selectedPathIndex: Int = 0
    private var pathUpdateTimer: Timer?

    // Wall detection properties
    private var meshVertexCache: [(position: SIMD3<Float>, normal: SIMD3<Float>)] = []
    private var lastMeshUpdateTime: TimeInterval = 0
    private let meshCacheUpdateInterval: TimeInterval = 2.0 // Update mesh cache every 2 seconds

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

    // Wall visualization properties
    private var wallVisualizationEntities: [AnchorEntity] = []
    private var isWallVisualizationEnabled: Bool = false
    
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
        
        // Update mesh vertex cache for wall detection
        updateMeshVertexCache(meshAnchors: meshAnchors)
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
        
        // Clear any existing k-paths
        clearAllKPaths()
        
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
            // No straight path: draw blocked straight line, then recalc A* and draw curved path
            currentPath = straightLinePath
            DispatchQueue.main.async { [weak self] in
                self?.visualizePath(path: straightLinePath, isBlocked: true)
            }
            DispatchQueue.global(qos: .userInitiated).async { [weak self] in
                guard let self = self else { return }
                self.updateOccupancyGrid()
                if let astarPath = self.findAStar(from: start, to: goal, maxIterations: 2000) {
                    let smoothed = self.thinPath(astarPath)
                    self.currentPath = smoothed
                    DispatchQueue.main.async {
                        print("PlanPath: A* found alternative path with \(smoothed.count) waypoints")
                        self.visualizePath(path: smoothed, isBlocked: false)
                    }
                } else {
                    print("PlanPath: A* failed to find alternative path")
                }
                self.isProcessingPath = false
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

        // Clamp all waypoint Y values to the ground plane (or lowest path Y)
        let yLevel = groundPlaneY ?? path.map { $0.y }.min() ?? 0
        let pathPoints = path.map { SIMD3<Float>($0.x, yLevel, $0.z) }
        
        // Debug: log path points
        print("visualizePath: pathPoints = \(pathPoints)")
         // Render each segment as a thin cylinder instead of custom mesh tubing
         let color = isBlocked ? UIColor(red: 1.0, green: 0.2, blue: 0.2, alpha: 0.9)
                             : UIColor(red: 0.2, green: 1.0, blue: 0.9, alpha: 0.9)
         let material = SimpleMaterial(color: color, roughness: 0.1, isMetallic: true)
         let anchor = AnchorEntity(world: .zero)
         // Draw horizontal path segments at ground level
         let thickness: Float = 0.02  // 2cm height
         for i in 0..<pathPoints.count-1 {
            let start = pathPoints[i]
            let end = pathPoints[i+1]
            // Horizontal direction and length
            let dx = end.x - start.x, dz = end.z - start.z
            let length = sqrt(dx*dx + dz*dz)
            guard length > 0 else { continue }
            let dirFlat = normalize(SIMD3<Float>(dx, 0, dz))
            
            // Create unit box and scale to segment dimensions
            let segment = ModelEntity(mesh: MeshResource.generateBox(size: [1,1,1]),
                                      materials: [material])
            segment.scale = SIMD3<Float>(length, thickness, thickness)
            
            // Position at segment midpoint on ground
            let mid = (start + end) * 0.5
            segment.position = SIMD3<Float>(mid.x, yLevel + thickness/2, mid.z)
            
            // Rotate from local X axis to direction in XZ plane
            let xAxis = SIMD3<Float>(1, 0, 0)
            segment.orientation = simd_quatf(from: xAxis, to: dirFlat)
            
            anchor.addChild(segment)
         }
         arView.scene.addAnchor(anchor)
         pathEntity = anchor
         pathMaterial = material
         
        print("Path visualization complete: rendered \(pathPoints.count-1) segments")
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

    // Clear any visualized path
    func clearPath() {
        if let anchor = pathEntity {
            arView?.scene.removeAnchor(anchor)
            pathEntity = nil
        }
    }
    
    // Clear all k-paths visualization
    func clearAllKPaths() {
        for pathEntity in kPathEntities {
            arView?.scene.removeAnchor(pathEntity)
        }
        kPathEntities.removeAll()
        allKPaths.removeAll()
        selectedPathIndex = 0
    }
    
    // Visualize multiple k-paths with different colors
    func visualizeKPaths(k: Int = 3) {
        // Clear existing k-paths
        clearAllKPaths()
        
        guard let start = startPoint, let goal = goalPoint else {
            print("k-paths visualization failed: start or goal point not set")
            return
        }
        
        print("Visualizing k-paths with k=\(k)")
        
        // Get the k-paths
        let paths = planKPaths(k: k)
        allKPaths = paths
        
        guard !paths.isEmpty else {
            print("No paths found for k-paths visualization")
            return
        }
        
        // Define colors for different paths
        let pathColors: [UIColor] = [
            UIColor(red: 0.2, green: 1.0, blue: 0.2, alpha: 0.9),  // Bright green (primary)
            UIColor(red: 1.0, green: 0.6, blue: 0.0, alpha: 0.9),  // Orange (alternative 1)
            UIColor(red: 0.2, green: 0.6, blue: 1.0, alpha: 0.9),  // Blue (alternative 2)
            UIColor(red: 1.0, green: 0.2, blue: 0.8, alpha: 0.9),  // Pink (alternative 3)
            UIColor(red: 0.8, green: 0.2, blue: 1.0, alpha: 0.9),  // Purple (alternative 4)
        ]
        
        // Visualize each path with different color and thickness
        for (index, path) in paths.enumerated() {
            let colorIndex = index % pathColors.count
            let color = pathColors[colorIndex]
            let thickness: Float = index == 0 ? 0.025 : 0.015  // Primary path thicker
            
            let pathEntity = createKPathVisualization(path: path, color: color, thickness: thickness, pathIndex: index)
            if let entity = pathEntity {
                kPathEntities.append(entity)
                arView?.scene.addAnchor(entity)
            }
        }
        
        // Set the first (shortest) path as current for following
        if let firstPath = paths.first {
            currentPath = firstPath
            selectedPathIndex = 0
        }
        
        print("K-paths visualization complete: showing \(paths.count) paths")
        for (index, path) in paths.enumerated() {
            print("  Path \(index + 1): \(path.count) waypoints, length: \(String(format: "%.2f", calculatePathLength(path)))m")
            
            // Add wall proximity analysis for each path
            analyzePathWallProximity(path: path, pathIndex: index + 1)
        }
    }
    
    // Create visualization for a single k-path
    private func createKPathVisualization(path: [SIMD3<Float>], color: UIColor, thickness: Float, pathIndex: Int) -> AnchorEntity? {
        guard let arView = arView, path.count >= 2 else { 
            print("Cannot visualize k-path: invalid path or no ARView")
            return nil
        }

        // Clamp all waypoint Y values to the ground plane
        let yLevel = groundPlaneY ?? path.map { $0.y }.min() ?? 0
        let pathPoints = path.map { SIMD3<Float>($0.x, yLevel, $0.z) }
        
        let material = SimpleMaterial(color: color, roughness: 0.1, isMetallic: true)
        let anchor = AnchorEntity(world: .zero)
        
        // Draw horizontal path segments at ground level
        for i in 0..<pathPoints.count-1 {
            let start = pathPoints[i]
            let end = pathPoints[i+1]
            
            // Horizontal direction and length
            let dx = end.x - start.x, dz = end.z - start.z
            let length = sqrt(dx*dx + dz*dz)
            guard length > 0 else { continue }
            let dirFlat = normalize(SIMD3<Float>(dx, 0, dz))
            
            // Create unit box and scale to segment dimensions
            let segment = ModelEntity(mesh: MeshResource.generateBox(size: [1,1,1]),
                                      materials: [material])
            segment.scale = SIMD3<Float>(length, thickness, thickness)
            
            // Position at segment midpoint on ground
            let mid = (start + end) * 0.5
            segment.position = SIMD3<Float>(mid.x, yLevel + thickness/2, mid.z)
            
            // Rotate from local X axis to direction in XZ plane
            let xAxis = SIMD3<Float>(1, 0, 0)
            segment.orientation = simd_quatf(from: xAxis, to: dirFlat)
            
            anchor.addChild(segment)
        }
        
        // Add path label at start point
        addPathLabel(to: anchor, at: pathPoints.first ?? SIMD3<Float>.zero, pathIndex: pathIndex, yLevel: yLevel)
        
        return anchor
    }
    
    // Add a small label/marker to identify each path
    private func addPathLabel(to anchor: AnchorEntity, at position: SIMD3<Float>, pathIndex: Int, yLevel: Float) {
        // Create a small sphere as path identifier
        let sphereMesh = MeshResource.generateSphere(radius: 0.02)
        let labelColor = pathIndex == 0 ? UIColor.white : UIColor.yellow
        let material = SimpleMaterial(color: labelColor, roughness: 0.1, isMetallic: true)
        
        let labelEntity = ModelEntity(mesh: sphereMesh, materials: [material])
        labelEntity.position = SIMD3<Float>(position.x, yLevel + 0.05, position.z) // Slightly above path
        
        anchor.addChild(labelEntity)
    }
    
    // Select a specific path from k-paths for following
    func selectPathForFollowing(pathIndex: Int) -> Bool {
        guard pathIndex >= 0 && pathIndex < allKPaths.count else {
            print("Invalid path index: \(pathIndex), available paths: \(allKPaths.count)")
            return false
        }
        
        selectedPathIndex = pathIndex
        currentPath = allKPaths[pathIndex]
        
        print("Selected path \(pathIndex + 1) for following (\(currentPath.count) waypoints)")
        return true
    }
    
    // Get information about available k-paths
    func getKPathsInfo() -> [(index: Int, length: Float, waypoints: Int)] {
        return allKPaths.enumerated().map { index, path in
            (index: index, length: calculatePathLength(path), waypoints: path.count)
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
    
    // MARK: - K-Paths Planning Algorithm
    
    /* K-PATHS ALGORITHM IMPROVEMENTS:
     * 
     * Problem: Original implementation was finding only 1 path because the blocking
     * mechanism was too aggressive (2-cell radius around entire path).
     * 
     * Solutions implemented:
     * 1. PROGRESSIVE BLOCKING: Start with exact path blocking (radius 0) for first alternative,
     *    then use minimal radius (1) for subsequent paths
     * 2. STRATEGIC PARTIAL BLOCKING: Instead of blocking every cell, block every 3rd cell
     *    with spacing to allow A* to find alternative routes around blocked sections
     * 3. REDUCED SIMILARITY THRESHOLD: Changed from 80% to 60% to allow more path diversity
     * 4. SMARTER GRID RESTORATION: Backup and restore original grid state after all searches
     * 
     * This allows A* to find meaningful alternative routes while still ensuring paths
     * are sufficiently different from each other.
     */
    
    /// Returns up to k non-identical, collision-free paths between startPoint and goalPoint
    /// Paths are sorted by total length (shortest first)
    public func planKPaths(k: Int = 3) -> [[SIMD3<Float>]] {
        guard let start = startPoint, let goal = goalPoint else {
            print("k-paths planning failed: start or goal point not set")
            return []
        }
        
        print("k-paths planning: searching for up to \(k) paths from \(start) to \(goal)")
        
        // Update the occupancy grid first and create a backup
        updateOccupancyGrid()
        let originalGrid = occupancyGrid // Create backup of original grid state
        
        var foundPaths: [[SIMD3<Float>]] = []
        
        for pathIndex in 0..<k {
            print("k-paths planning: searching for path \(pathIndex + 1)/\(k)")
            
            // Try to find a path with current grid state
            if let rawPath = findAStar(from: start, to: goal, maxIterations: 2000) {
                let thinned = thinPath(rawPath)
                foundPaths.append(thinned)
                print("k-paths planning: found path \(pathIndex + 1) with \(thinned.count) waypoints, length: \(String(format: "%.2f", calculatePathLength(thinned)))m")
                
                // Progressive blocking strategy - start minimal and increase as needed
                let blockRadius = pathIndex == 0 ? 0 : min(1, pathIndex - 1)
                print("k-paths: using block radius \(blockRadius) for path \(pathIndex + 1)")
                blockPathInGrid(thinned, blockRadius: blockRadius)
            } else {
                print("k-paths planning: no more paths found at iteration \(pathIndex + 1)")
                break
            }
        }
        
        // Restore original grid state
        occupancyGrid = originalGrid
        
        // Remove duplicate paths (paths that are too similar)
        let uniquePaths = removeSimilarPaths(foundPaths)
        
        // Sort paths by length (shortest first)
        let sortedPaths = uniquePaths.sorted { path1, path2 in
            calculatePathLength(path1) < calculatePathLength(path2)
        }
        
        print("k-paths planning produced \(sortedPaths.count) unique paths from \(foundPaths.count) found paths")
        for (index, path) in sortedPaths.enumerated() {
            print("  Path \(index + 1): \(path.count) waypoints, length: \(String(format: "%.2f", calculatePathLength(path)))m")
        }
        
        return sortedPaths
    }
    
    /// Block a path in the occupancy grid with a specified radius using strategic partial blocking
    private func blockPathInGrid(_ path: [SIMD3<Float>], blockRadius: Int) {
        let pathCells = convertPathToGridCells(path)
        
        // Use different blocking strategies based on radius
        if blockRadius == 0 {
            // Only block exact path cells for first alternative
            for (gridX, gridZ) in pathCells {
                if gridX >= 0 && gridX < gridSize.width && gridZ >= 0 && gridZ < gridSize.height {
                    if occupancyGrid[gridX][gridZ] == .free || occupancyGrid[gridX][gridZ] == .unknown {
                        occupancyGrid[gridX][gridZ] = .occupied
                    }
                }
            }
        } else {
            // Block every 3rd cell with the specified radius for subsequent alternatives
            let blockingStep = max(3, blockRadius + 2) // Increase spacing between blocked sections
            
            for i in stride(from: 0, to: pathCells.count, by: blockingStep) {
                let (gridX, gridZ) = pathCells[i]
                
                // Block cells in a radius around selected path cells
                for dx in -blockRadius...blockRadius {
                    for dz in -blockRadius...blockRadius {
                        let blockX = gridX + dx
                        let blockZ = gridZ + dz
                        
                        if blockX >= 0 && blockX < gridSize.width && blockZ >= 0 && blockZ < gridSize.height {
                            // Only block free cells, don't overwrite actual obstacles
                            if occupancyGrid[blockX][blockZ] == .free || occupancyGrid[blockX][blockZ] == .unknown {
                                occupancyGrid[blockX][blockZ] = .occupied
                            }
                        }
                    }
                }
            }
        }
        
        print("k-paths: blocked path cells with radius \(blockRadius), using \(blockRadius == 0 ? "exact path" : "strategic partial") blocking")
    }
    
    /// Remove paths that are too similar to each other
    private func removeSimilarPaths(_ paths: [[SIMD3<Float>]]) -> [[SIMD3<Float>]] {
        guard paths.count > 1 else { return paths }
        
        var uniquePaths: [[SIMD3<Float>]] = []
        
        for currentPath in paths {
            var isUnique = true
            
            for existingPath in uniquePaths {
                if pathsSimilar(currentPath, existingPath, threshold: 0.6) { // Reduced from 80% to 60% to allow more diversity
                    isUnique = false
                    break
                }
            }
            
            if isUnique {
                uniquePaths.append(currentPath)
            }
        }
        
        return uniquePaths
    }
    
    /// Check if two paths are too similar
    private func pathsSimilar(_ path1: [SIMD3<Float>], _ path2: [SIMD3<Float>], threshold: Float) -> Bool {
        // Convert both paths to grid cells for comparison
        let cells1 = Set(convertPathToGridCells(path1).map { "\($0.0),\($0.1)" })
        let cells2 = Set(convertPathToGridCells(path2).map { "\($0.0),\($0.1)" })
        
        let intersection = cells1.intersection(cells2)
        let union = cells1.union(cells2)
        
        let similarity = Float(intersection.count) / Float(union.count)
        return similarity > threshold
    }
    
    /// Convert a world-space path to grid cell coordinates
    private func convertPathToGridCells(_ path: [SIMD3<Float>]) -> [(Int, Int)] {
        var cells: [(Int, Int)] = []
        
        for i in 0..<path.count-1 {
            let start = path[i]
            let end = path[i+1]
            
            // Get all grid cells along this segment using Bresenham's line algorithm
            let startGridX = Int((start.x - gridOrigin.x) / gridResolution)
            let startGridZ = Int((start.z - gridOrigin.z) / gridResolution)
            let endGridX = Int((end.x - gridOrigin.x) / gridResolution)
            let endGridZ = Int((end.z - gridOrigin.z) / gridResolution)
            
            let lineCells = bresenhamLine(from: (startGridX, startGridZ), to: (endGridX, endGridZ))
            cells.append(contentsOf: lineCells)
        }
        
        // Remove duplicates
        var uniqueCells: [(Int, Int)] = []
        var seenCells: Set<String> = []
        
        for cell in cells {
            let key = "\(cell.0),\(cell.1)"
            if !seenCells.contains(key) {
                seenCells.insert(key)
                uniqueCells.append(cell)
            }
        }
        
        return uniqueCells
    }
    
    /// Simple Bresenham's line algorithm to get all grid cells along a line
    private func bresenhamLine(from start: (Int, Int), to end: (Int, Int)) -> [(Int, Int)] {
        var cells: [(Int, Int)] = []
        
        let dx = abs(end.0 - start.0)
        let dz = abs(end.1 - start.1)
        let sx = start.0 < end.0 ? 1 : -1
        let sz = start.1 < end.1 ? 1 : -1
        var err = dx - dz
        
        var currentX = start.0
        var currentZ = start.1
        
        while true {
            cells.append((currentX, currentZ))
            
            if currentX == end.0 && currentZ == end.1 {
                break
            }
            
            let e2 = 2 * err
            if e2 > -dz {
                err -= dz
                currentX += sx
            }
            if e2 < dx {
                err += dx
                currentZ += sz
            }
        }
        
        return cells
    }
    
    /// Calculate the total length of a path
    private func calculatePathLength(_ path: [SIMD3<Float>]) -> Float {
        guard path.count > 1 else { return 0.0 }
        
        var totalLength: Float = 0.0
        for i in 0..<path.count-1 {
            let segment = path[i+1] - path[i]
            totalLength += length(segment)
        }
        return totalLength
    }

    // MARK: - Testing Methods
    
    /// Test method to verify k-paths functionality
    public func testKPaths() {
        guard let start = startPoint, let goal = goalPoint else {
            print("Test k-paths: No start/goal points set")
            return
        }
        
        print("=== Testing K-Paths Functionality ===")
        
        // Test with k=3
        let paths = planKPaths(k: 3)
        
        print("Test completed. Found \(paths.count) paths:")
        for (index, path) in paths.enumerated() {
            print("  Test Path \(index + 1): \(path.count) waypoints, length: \(String(format: "%.2f", calculatePathLength(path)))m")
        }
        
        print("=== End K-Paths Test ===")
    }

    // MARK: - Wall Detection Helpers

    /// Update cached mesh vertices and normals for wall detection
    private func updateMeshVertexCache(meshAnchors: [ARMeshAnchor]) {
        let now = Date().timeIntervalSince1970
        guard now - lastMeshUpdateTime >= meshCacheUpdateInterval else { return }
        lastMeshUpdateTime = now
        meshVertexCache.removeAll()
        
        for anchor in meshAnchors {
            let geometry = anchor.geometry
            let vBuffer = geometry.vertices.buffer
            let nBuffer = geometry.normals.buffer
            let vStride = geometry.vertices.stride
            let nStride = geometry.normals.stride
            let vOffset = Int(geometry.vertices.offset)
            let nOffset = Int(geometry.normals.offset)
            
            // Sample every 10th vertex
            for i in stride(from: 0, to: geometry.vertices.count, by: 10) {
                let vPtr = vBuffer.contents().advanced(by: vOffset + i * vStride)
                let vertex = vPtr.bindMemory(to: SIMD3<Float>.self, capacity: 1).pointee
                let worldV4 = anchor.transform * simd_float4(vertex, 1)
                let worldPos = SIMD3<Float>(worldV4.x, worldV4.y, worldV4.z)
                
                let nPtr = nBuffer.contents().advanced(by: nOffset + i * nStride)
                let normal = nPtr.bindMemory(to: SIMD3<Float>.self, capacity: 1).pointee
                let worldN4 = anchor.transform * simd_float4(normal, 0)
                let worldNormal = normalize(SIMD3<Float>(worldN4.x, worldN4.y, worldN4.z))
                
                meshVertexCache.append((position: worldPos, normal: worldNormal))
            }
        }
    }

    /// Returns true if any mesh vertex near worldPos has a near-vertical normal (wall)
    func isWall(at worldPos: SIMD3<Float>) -> Bool {
        let threshold: Float = 0.15
        let thresh2 = threshold * threshold
        for entry in meshVertexCache {
            let d2 = simd_length_squared(entry.position - worldPos)
            if d2 <= thresh2 && abs(entry.normal.y) < 0.3 {
                return true
            }
        }
        return false
    }

    /// Samples radially around worldPos and returns distance to nearest wall
    /// Always returns a valid distance - uses comprehensive search if initial radial search fails
    func distanceToNearestWall(from worldPos: SIMD3<Float>, searchRadius: Float = 0.5) -> Float {
        // First try the fast radial sampling approach
        let directions: [SIMD3<Float>] = (0..<8).map { i in
            let theta = Float(i) * (2 * .pi / 8)
            return SIMD3<Float>(cos(theta), 0, sin(theta))
        }
        let steps = 10 // Increased from 5 for better accuracy
        var minDist = Float.infinity
        
        for dir in directions {
            for step in 1...steps {
                let t = searchRadius * Float(step) / Float(steps)
                let sample = worldPos + dir * t
                if isWall(at: sample) {
                    minDist = min(minDist, t)
                    break
                }
            }
        }
        
        // If no wall found in radial search, do comprehensive distance calculation
        if minDist == Float.infinity {
            minDist = findNearestWallDistance(from: worldPos, maxSearchRadius: searchRadius * 4)
        }
        
        // Ensure we never return infinity - if still no walls found, return the max search distance
        return minDist == Float.infinity ? searchRadius * 4 : minDist
    }
    
    /// Comprehensive wall distance calculation that searches through all cached wall vertices
    /// Returns the actual minimum distance to any wall vertex
    private func findNearestWallDistance(from worldPos: SIMD3<Float>, maxSearchRadius: Float) -> Float {
        var minDistance = Float.infinity
        
        // Check distance to all wall vertices in cache
        for entry in meshVertexCache {
            // Only consider vertices that are actually walls (vertical surfaces)
            if abs(entry.normal.y) < 0.3 {
                let distance = simd_length(entry.position - worldPos)
                if distance <= maxSearchRadius {
                    minDistance = min(minDistance, distance)
                }
            }
        }
        
        // If still no walls found within max search radius, return the max search radius
        // This ensures we never return infinity
        return minDistance == Float.infinity ? maxSearchRadius : minDistance
    }
    
    /// Visualize detected walls in the scene
    func toggleWallVisualization() -> Bool {
        if isWallVisualizationEnabled {
            // Turn off wall visualization
            clearWallVisualization()
            isWallVisualizationEnabled = false
            return false
        } else {
            // Turn on wall visualization
            visualizeDetectedWalls()
            isWallVisualizationEnabled = true
            return true
        }
    }
    
    /// Clear all wall visualization
    private func clearWallVisualization() {
        for entity in wallVisualizationEntities {
            arView?.scene.removeAnchor(entity)
        }
        wallVisualizationEntities.removeAll()
    }
    
    /// Visualize walls as red wireframe or colored surfaces
    private func visualizeDetectedWalls() {
        guard let arView = arView else { return }
        clearWallVisualization()
        
        // Create wall markers at detected wall positions
        for (index, entry) in meshVertexCache.enumerated() {
            // Only show every 20th wall vertex to avoid clutter
            guard index % 20 == 0 && abs(entry.normal.y) < 0.3 else { continue }
            
            // Create a small red cube to mark wall positions
            let wallMarker = ModelEntity(
                mesh: MeshResource.generateBox(size: [0.02, 0.02, 0.02]),
                materials: [SimpleMaterial(color: .red, roughness: 0.3, isMetallic: false)]
            )
            
            let anchor = AnchorEntity(world: entry.position)
            anchor.addChild(wallMarker)
            arView.scene.addAnchor(anchor)
            wallVisualizationEntities.append(anchor)
        }
        
        print("Wall visualization: showing \(wallVisualizationEntities.count) wall markers")
    }
    
    /// Test wall detection at the cuboid's current position
    /// Always returns meaningful distance values
    func testWallDetectionAtCuboid() -> (isNearWall: Bool, distance: Float) {
        guard let cuboidPos = cuboidEntity?.position(relativeTo: nil) else {
            // If no cuboid, return a reasonable default distance
            return (false, 2.0) // 2 meters as default "no wall nearby" distance
        }
        
        let isWall = isWall(at: cuboidPos)
        let distance = distanceToNearestWall(from: cuboidPos)
        
        print("Wall detection test at cuboid position \(cuboidPos):")
        print("  - Is wall detected: \(isWall)")
        print("  - Distance to nearest wall: \(String(format: "%.2f", distance))m")
        
        return (isWall, distance)
    }
    
    /// Get distances from start and end points to nearest walls
    /// Always returns meaningful distance values (never null/infinity)
    func getStartEndWallDistances() -> (startDistance: Float, endDistance: Float, startIsWall: Bool, endIsWall: Bool) {
        var startDistance: Float = 2.0 // Default distance if no start point
        var endDistance: Float = 2.0   // Default distance if no end point
        var startIsWall = false
        var endIsWall = false
        
        // Check start point
        if let start = startPoint {
            startDistance = distanceToNearestWall(from: start)
            startIsWall = isWall(at: start)
            print("Start point wall analysis:")
            print("  - Position: \(start)")
            print("  - Is wall detected: \(startIsWall)")
            print("  - Distance to nearest wall: \(String(format: "%.2f", startDistance))m")
        }
        
        // Check end point
        if let goal = goalPoint {
            endDistance = distanceToNearestWall(from: goal)
            endIsWall = isWall(at: goal)
            print("End point wall analysis:")
            print("  - Position: \(goal)")
            print("  - Is wall detected: \(endIsWall)")
            print("  - Distance to nearest wall: \(String(format: "%.2f", endDistance))m")
        }
        
        return (startDistance: startDistance, endDistance: endDistance, startIsWall: startIsWall, endIsWall: endIsWall)
    }
    
    /// Get distance from any arbitrary point to nearest walls
    /// Always returns meaningful distance values (never null/infinity)
    func getWallDistanceFromPoint(_ point: SIMD3<Float>) -> (distance: Float, isWall: Bool) {
        let distance = distanceToNearestWall(from: point)
        let isWall = isWall(at: point)
        
        print("Wall analysis for point \(point):")
        print("  - Is wall detected: \(isWall)")
        print("  - Distance to nearest wall: \(String(format: "%.2f", distance))m")
        
        return (distance: distance, isWall: isWall)
    }
    
    /// Analyze how close a path gets to walls
    private func analyzePathWallProximity(path: [SIMD3<Float>], pathIndex: Int) {
        var minWallDistance = Float.infinity
        var maxWallDistance: Float = 0
        var wallContactPoints: [SIMD3<Float>] = []
        
        // Sample every few waypoints to check wall proximity
        let sampleStep = max(1, path.count / 10) // Sample ~10 points along path
        
        for i in stride(from: 0, to: path.count, by: sampleStep) {
            let waypoint = path[i]
            let wallDistance = distanceToNearestWall(from: waypoint)
            
            // Since distanceToNearestWall now always returns a valid distance (never infinity)
            minWallDistance = min(minWallDistance, wallDistance)
            maxWallDistance = max(maxWallDistance, wallDistance)
            
            // If very close to wall (< 20cm), mark as potential contact point
            if wallDistance < 0.2 {
                wallContactPoints.append(waypoint)
            }
        }
        
        // Report analysis - minWallDistance will always be valid now
        if minWallDistance != Float.infinity {
            print("    Wall Analysis - Min distance: \(String(format: "%.2f", minWallDistance))m, Contact points: \(wallContactPoints.count)")
            
            // Visual warning for paths too close to walls
            if minWallDistance < 0.1 {
                print("    ⚠️  WARNING: Path \(pathIndex) passes very close to walls!")
            }
        } else {
            // This case should never happen now, but keep as fallback
            print("    Wall Analysis - No walls detected near path")
        }
    }
}
