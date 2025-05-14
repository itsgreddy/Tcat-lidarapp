import UIKit
import ARKit
import RealityKit
import simd
import ObjectiveC  // Add this import for objc_setAssociatedObject

class ARViewController: UIViewController {
    // AR View to display the LiDAR scan and cuboid
    private lazy var arView: ARView = {
        let view = ARView(frame: .zero)
        view.translatesAutoresizingMaskIntoConstraints = false
        return view
    }()
    
    // AR manager to handle the LiDAR and cuboid logic
    private lazy var arManager: ARManager = {
        return ARManager()
    }()
    
    // Intersection callback
    var onIntersectionUpdate: ((Bool) -> Void)?
    
    private let statusView: UIView = {
        let v = UIView()
        v.translatesAutoresizingMaskIntoConstraints = false
        v.backgroundColor = .green
        return v
    }()

    // Dimension sliders
    private let widthSlider: UISlider = {
        let s = UISlider()
        s.minimumValue = 50
        s.maximumValue = 200
        s.value = 70
        s.translatesAutoresizingMaskIntoConstraints = false
        return s
    }()
    
    private let heightSlider: UISlider = {
        let s = UISlider()
        s.minimumValue = 100
        s.maximumValue = 250
        s.value = 150
        s.translatesAutoresizingMaskIntoConstraints = false
        return s
    }()
    
    private let depthSlider: UISlider = {
        let s = UISlider()
        s.minimumValue = 50
        s.maximumValue = 200
        s.value = 120
        s.translatesAutoresizingMaskIntoConstraints = false
        return s
    }()
    
    // Position sliders
    private let posXSlider: UISlider = {
        let s = UISlider()
        s.minimumValue = -3.0
        s.maximumValue = 3.0
        s.value = 0.0
        s.translatesAutoresizingMaskIntoConstraints = false
        return s
    }()
    
    private let posYSlider: UISlider = {
        let s = UISlider()
        s.minimumValue = -2.0
        s.maximumValue = 2.0
        s.value = 0.0
        s.translatesAutoresizingMaskIntoConstraints = false
        return s
    }()
    
    private let posZSlider: UISlider = {
        let s = UISlider()
        s.minimumValue = -5.0
        s.maximumValue = 0.0
        s.value = -1.5
        s.translatesAutoresizingMaskIntoConstraints = false
        return s
    }()

    private var isLocked = false

    private let lockButton: UIButton = {
        let b = UIButton(type: .system)
        b.translatesAutoresizingMaskIntoConstraints = false
        b.setTitle("Lock", for: .normal)
        return b
    }()
    
    // Path planning UI elements
    private var pathPlanningMode = false
    private var startPointEntity: ModelEntity?
    private var goalPointEntity: ModelEntity?
    private var exportButton: UIButton?
    private var pathPlanButton: UIButton?
    private var meshButton: UIButton?
    private var followPathButton: UIButton?
    private var isFollowingPath = false
    
    // Add these new properties to store positions directly
    private var startMarkerPosition = SIMD3<Float>(0, 0, 0)
    private var goalMarkerPosition = SIMD3<Float>(0, 0, 0)

    // MARK: - Lifecycle Methods
    
    override func viewDidLoad() {
        super.viewDidLoad()
        setupARView()
        setupGestures()
        setupUI()
        setupPathPlanningUI()
        requestCameraPermission()
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        arManager.cleanup()
    }
    
    // MARK: - Setup Methods
    
    private func setupARView() {
        view.addSubview(arView)
        
        NSLayoutConstraint.activate([
            arView.topAnchor.constraint(equalTo: view.topAnchor),
            arView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            arView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            arView.bottomAnchor.constraint(equalTo: view.bottomAnchor)
        ])
        
        // Start with debug options disabled
        // User can toggle them with the "Show Mesh" button
        arView.debugOptions = []
        
        // Enable mesh anchors in the configuration when we initialize AR
        // This ensures the mesh data is available even if visualization is off
        if let config = arView.session.configuration as? ARWorldTrackingConfiguration {
            if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
                config.sceneReconstruction = .mesh
            }
            arView.session.run(config)
        }
    }
    
    private func setupGestures() {
        let pan = UIPanGestureRecognizer(target: self, action: #selector(handlePan(_:)))
        arView.addGestureRecognizer(pan)
        
        let pinch = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
        arView.addGestureRecognizer(pinch)
        
        let tap = UITapGestureRecognizer(target: self, action: #selector(handleTap(_:)))
        arView.addGestureRecognizer(tap)
    }
    
    private func setupUI() {
        // Status bar at top
        view.addSubview(statusView)
        NSLayoutConstraint.activate([
            statusView.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            statusView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            statusView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            statusView.heightAnchor.constraint(equalToConstant: 20)
        ])

        // Create two columns of sliders
        let leftStack = UIStackView(arrangedSubviews: [
            labeledSlider("W", widthSlider),
            labeledSlider("H", heightSlider),
            labeledSlider("D", depthSlider)
        ])
        leftStack.axis = .vertical
        leftStack.spacing = 8
        leftStack.translatesAutoresizingMaskIntoConstraints = false
        
        let rightStack = UIStackView(arrangedSubviews: [
            labeledSlider("X", posXSlider),
            labeledSlider("Y", posYSlider),
            labeledSlider("Z", posZSlider)
        ])
        rightStack.axis = .vertical
        rightStack.spacing = 8
        rightStack.translatesAutoresizingMaskIntoConstraints = false
        
        // Create a horizontal stack to hold both columns
        let mainStack = UIStackView(arrangedSubviews: [leftStack, rightStack])
        mainStack.axis = .horizontal
        mainStack.spacing = 16
        mainStack.distribution = .fillEqually
        mainStack.translatesAutoresizingMaskIntoConstraints = false
        
        view.addSubview(mainStack)
        NSLayoutConstraint.activate([
            mainStack.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -16),
            mainStack.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 16),
            mainStack.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16)
        ])

        // Slider actions
        [widthSlider, heightSlider, depthSlider].forEach {
            $0.addTarget(self, action: #selector(dimensionSliderChanged(_:)), for: .valueChanged)
        }
        
        [posXSlider, posYSlider, posZSlider].forEach {
            $0.addTarget(self, action: #selector(positionSliderChanged(_:)), for: .valueChanged)
        }

        // Add lock/unlock button
        view.addSubview(lockButton)
        NSLayoutConstraint.activate([
            lockButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
            lockButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16)
        ])
        lockButton.addTarget(self, action: #selector(toggleLock(_:)), for: .touchUpInside)
    }
    
    private func setupPathPlanningUI() {
        // Path planning mode button - Make it more prominent
        let planButton = UIButton(type: .system)
        planButton.setTitle("Plan Path", for: .normal)
        planButton.backgroundColor = UIColor.systemBlue
        planButton.setTitleColor(.white, for: .normal)
        planButton.layer.cornerRadius = 8
        planButton.translatesAutoresizingMaskIntoConstraints = false
        planButton.titleLabel?.font = UIFont.boldSystemFont(ofSize: 16)
        view.addSubview(planButton)
        
        NSLayoutConstraint.activate([
            planButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
            planButton.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            planButton.widthAnchor.constraint(equalToConstant: 110),
            planButton.heightAnchor.constraint(equalToConstant: 44)
        ])
        
        planButton.addTarget(self, action: #selector(togglePathPlanningMode), for: .touchUpInside)
        self.pathPlanButton = planButton
        
        // Export path button - Also make it more visible
        let exportBtn = UIButton(type: .system)
        exportBtn.setTitle("Export Path", for: .normal)
        exportBtn.backgroundColor = UIColor.systemGreen
        exportBtn.setTitleColor(.white, for: .normal)
        exportBtn.layer.cornerRadius = 8
        exportBtn.translatesAutoresizingMaskIntoConstraints = false
        exportBtn.titleLabel?.font = UIFont.boldSystemFont(ofSize: 16)
        exportBtn.isHidden = true
        view.addSubview(exportBtn)
        
        NSLayoutConstraint.activate([
            exportBtn.topAnchor.constraint(equalTo: planButton.bottomAnchor, constant: 8),
            exportBtn.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            exportBtn.widthAnchor.constraint(equalToConstant: 110),
            exportBtn.heightAnchor.constraint(equalToConstant: 44)
        ])
        
        exportBtn.addTarget(self, action: #selector(exportPath), for: .touchUpInside)
        self.exportButton = exportBtn
        
        // Add debug/mesh visibility toggle button
        let meshBtn = UIButton(type: .system)
        meshBtn.setTitle("Show Mesh", for: .normal)
        meshBtn.backgroundColor = UIColor.systemPurple
        meshBtn.setTitleColor(.white, for: .normal)
        meshBtn.layer.cornerRadius = 8
        meshBtn.translatesAutoresizingMaskIntoConstraints = false
        meshBtn.titleLabel?.font = UIFont.boldSystemFont(ofSize: 16)
        view.addSubview(meshBtn)
        
        NSLayoutConstraint.activate([
            meshBtn.topAnchor.constraint(equalTo: exportBtn.bottomAnchor, constant: 8),
            meshBtn.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            meshBtn.widthAnchor.constraint(equalToConstant: 110),
            meshBtn.heightAnchor.constraint(equalToConstant: 44)
        ])
        
        meshBtn.addTarget(self, action: #selector(toggleMeshVisibility), for: .touchUpInside)
        self.meshButton = meshBtn
        
        // Add follow path button
        let followBtn = UIButton(type: .system)
        followBtn.setTitle("Follow Path", for: .normal)
        followBtn.backgroundColor = UIColor.systemYellow
        followBtn.setTitleColor(.white, for: .normal)
        followBtn.layer.cornerRadius = 8
        followBtn.translatesAutoresizingMaskIntoConstraints = false
        followBtn.titleLabel?.font = UIFont.boldSystemFont(ofSize: 16)
        followBtn.isHidden = true  // Initially hidden until path is created
        view.addSubview(followBtn)
        
        NSLayoutConstraint.activate([
            followBtn.topAnchor.constraint(equalTo: meshBtn.bottomAnchor, constant: 8),
            followBtn.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            followBtn.widthAnchor.constraint(equalToConstant: 110),
            followBtn.heightAnchor.constraint(equalToConstant: 44)
        ])
        
        followBtn.addTarget(self, action: #selector(toggleFollowPath), for: .touchUpInside)
        self.followPathButton = followBtn
    }

    private func labeledSlider(_ label: String, _ slider: UISlider) -> UIView {
        let lbl = UILabel()
        lbl.text = label
        lbl.widthAnchor.constraint(equalToConstant: 20).isActive = true
        let hstack = UIStackView(arrangedSubviews: [lbl, slider])
        hstack.axis = .horizontal
        hstack.spacing = 8
        return hstack
    }

    // MARK: - Action Handlers
    
    @objc private func dimensionSliderChanged(_ s: UISlider) {
        arManager.setCuboid(
            width: widthSlider.value,
            height: heightSlider.value,
            depth: depthSlider.value
        )
    }
    
    @objc private func positionSliderChanged(_ s: UISlider) {
        arManager.updateCuboidPosition(
            x: posXSlider.value,
            y: posYSlider.value,
            z: posZSlider.value
        )
    }

    @objc private func toggleLock(_ sender: UIButton) {
        if isLocked {
            arManager.unlockCuboid()
            sender.setTitle("Lock", for: .normal)
        } else {
            arManager.lockCuboid()
            sender.setTitle("Unlock", for: .normal)
        }
        isLocked.toggle()
    }
    
    @objc private func toggleMeshVisibility(_ sender: UIButton) {
        // Toggle between showing mesh and hiding mesh
        if arView.debugOptions.contains(.showSceneUnderstanding) {
            // Currently showing, so hide it
            arView.debugOptions = []
            sender.setTitle("Show Mesh", for: .normal)
            sender.backgroundColor = UIColor.systemPurple
        } else {
            // Currently hidden, so show it
            arView.debugOptions = [.showSceneUnderstanding, .showFeaturePoints]
            sender.setTitle("Hide Mesh", for: .normal)
            sender.backgroundColor = UIColor.systemOrange
        }
    }
    
    @objc private func toggleFollowPath(_ sender: UIButton) {
        if !isFollowingPath {
            // Start following path
            sender.setTitle("Cancel", for: .normal)
            sender.backgroundColor = UIColor.systemOrange
            isFollowingPath = true
            
            // Start the cuboid moving along the path
            arManager.followPath { [weak self] success in
                guard let self = self else { return }
                DispatchQueue.main.async {
                    // Animation finished
                    self.followPathButton?.setTitle("Follow Path", for: .normal)
                    self.followPathButton?.backgroundColor = UIColor.systemYellow
                    self.isFollowingPath = false
                    
                    // Show result to user
                    let title = success ? "Path Complete" : "Path Blocked"
                    let message = success ? 
                        "The cuboid successfully reached the destination." :
                        "The cuboid's path was blocked by an obstacle."
                    
                    let alert = UIAlertController(
                        title: title,
                        message: message,
                        preferredStyle: .alert
                    )
                    alert.addAction(UIAlertAction(title: "OK", style: .default))
                    self.present(alert, animated: true)
                }
            }
        } else {
            // Cancel path following
            sender.setTitle("Follow Path", for: .normal)
            sender.backgroundColor = UIColor.systemYellow
            isFollowingPath = false
            arManager.cancelPathFollowing()
        }
    }

    @objc private func handlePan(_ gesture: UIPanGestureRecognizer) {
        guard let entity = arManager.currentCuboidEntity else { return }
        let translation = gesture.translation(in: arView)
        let dx = Float(translation.x) * 0.001
        let dy = Float(-translation.y) * 0.001
        entity.transform.translation += SIMD3(dx, dy, 0)
        gesture.setTranslation(.zero, in: arView)
    }
    
    @objc private func handlePinch(_ gesture: UIPinchGestureRecognizer) {
        guard let entity = arManager.currentCuboidEntity else { return }
        let s = Float(gesture.scale)
        entity.scale *= SIMD3(repeating: s)
        gesture.scale = 1
    }
    
    @objc private func handleTap(_ gesture: UITapGestureRecognizer) {
        let location = gesture.location(in: arView)
        handlePathPlanningTap(location)
    }
    
    private func requestCameraPermission() {
        AVCaptureDevice.requestAccess(for: .video) { [weak self] granted in
            DispatchQueue.main.async {
                if granted {
                    self?.initializeAR()
                } else {
                    // Show alert about missing camera permission
                    let alert = UIAlertController(
                        title: "Camera Access Required",
                        message: "This app needs camera access to use LiDAR scanning features.",
                        preferredStyle: .alert
                    )
                    alert.addAction(UIAlertAction(title: "OK", style: .default))
                    self?.present(alert, animated: true)
                }
            }
        }
    }
    
    private func initializeAR() {
        arManager.initialize(arView: arView) { [weak self] isIntersecting in
            DispatchQueue.main.async {
                self?.statusView.backgroundColor = isIntersecting ? .red : .green
                self?.onIntersectionUpdate?(isIntersecting)
            }
        }
    }
    
    // MARK: - Path Planning
    
    @objc private func togglePathPlanningMode() {
        pathPlanningMode = !pathPlanningMode
        
        if pathPlanningMode {
            // Enter path planning mode
            pathPlanButton?.setTitle("Cancel", for: .normal)
            pathPlanButton?.backgroundColor = UIColor.systemRed
            
            // Show instructions
            let alert = UIAlertController(
                title: "Path Planning",
                message: "Tap to place start point, then tap again to place destination.",
                preferredStyle: .alert
            )
            alert.addAction(UIAlertAction(title: "OK", style: .default))
            present(alert, animated: true)
            
            // Clear existing points
            removePathPoints()
        } else {
            // Exit path planning mode
            pathPlanButton?.setTitle("Plan Path", for: .normal)
            pathPlanButton?.backgroundColor = UIColor.systemBlue
            exportButton?.isHidden = true
            followPathButton?.isHidden = true
            
            // Clear path and points
            removePathPoints()
        }
    }
    
    private func removePathPoints() {
        if let entity = startPointEntity, let parent = entity.parent {
            parent.removeChild(entity)
            startPointEntity = nil
            startMarkerPosition = SIMD3<Float>(0, 0, 0) // Reset stored position
        }
        
        if let entity = goalPointEntity, let parent = entity.parent {
            parent.removeChild(entity)
            goalPointEntity = nil
            goalMarkerPosition = SIMD3<Float>(0, 0, 0) // Reset stored position
        }
    }
    
    private func handlePathPlanningTap(_ location: CGPoint) {
        guard pathPlanningMode else { return }
        
        // Use a more robust raycast query to ensure we get hits
        guard let raycast = arView.ray(through: location) else {
            print("Failed to create ray from touch point")
            return
        }
        
        var raycastQuery = ARRaycastQuery(
            origin: raycast.origin,
            direction: raycast.direction,
            allowing: .estimatedPlane,
            alignment: .any
        )
        
        // Fallbacks for raycast in case first attempt fails
        let raycastMethods: [ARRaycastQuery.Target] = [
            .estimatedPlane,
            .existingPlaneInfinite,
            .existingPlaneGeometry
        ]
        
        var raycastResult: ARRaycastResult?
        
        // Try different raycast methods until we get a hit
        for method in raycastMethods {
            if let ray = arView.ray(through: location) {
                raycastQuery = ARRaycastQuery(
                    origin: ray.origin,
                    direction: ray.direction,
                    allowing: method,
                    alignment: .any
                )
                
                if let result = arView.session.raycast(raycastQuery).first {
                    raycastResult = result
                    break
                }
            }
        }
        
        // If all raycasts failed, try to place at a reasonable default distance
        if raycastResult == nil {
            let defaultDistance: Float = 1.5 // 1.5 meters in front
            guard let ray = arView.ray(through: location) else {
                print("Failed to create ray for default position")
                return
            }
            let defaultPosition = ray.origin + ray.direction * defaultDistance
            
            if startPointEntity == nil {
                startPointEntity = createMarkerEntity(color: .systemGreen, position: defaultPosition)
                startMarkerPosition = defaultPosition
                print("Placed start marker at default position: \(defaultPosition)")
                return
            } else if goalPointEntity == nil {
                goalPointEntity = createMarkerEntity(color: .systemBlue, position: defaultPosition)
                goalMarkerPosition = defaultPosition
                print("Placed goal marker at default position: \(defaultPosition)")
                
                // Use stored positions directly
                print("Using positions: start=\(startMarkerPosition), goal=\(goalMarkerPosition)")
                arManager.setPathPoints(start: startMarkerPosition, goal: goalMarkerPosition)
                exportButton?.isHidden = false
                followPathButton?.isHidden = false
                return
            }
        }
        
        guard let hitResult = raycastResult else {
            print("Failed to place marker - no raycast hit")
            return
        }
        
        let worldPosition = hitResult.worldTransform.columns.3
        let position = SIMD3<Float>(worldPosition.x, worldPosition.y, worldPosition.z)
        
        print("Raycast hit at position: \(position)")
        
        if startPointEntity == nil {
            // Place start point
            startPointEntity = createMarkerEntity(color: .systemGreen, position: position)
            startMarkerPosition = position
            print("Placed start marker")
        } else if goalPointEntity == nil {
            // Place goal point
            goalPointEntity = createMarkerEntity(color: .systemBlue, position: position)
            goalMarkerPosition = position
            print("Placed goal marker")
            
            // Use stored positions for path planning
            print("Using positions: start=\(startMarkerPosition), goal=\(goalMarkerPosition)")
            arManager.setPathPoints(start: startMarkerPosition, goal: goalMarkerPosition)
            exportButton?.isHidden = false
            followPathButton?.isHidden = false
        } else {
            // Replace both points
            print("Removing existing path points")
            removePathPoints()
            startPointEntity = createMarkerEntity(color: .systemGreen, position: position)
            startMarkerPosition = position
            print("Placed new start marker")
            followPathButton?.isHidden = true
        }
    }
    
    private func createMarkerEntity(color: UIColor, position: SIMD3<Float>) -> ModelEntity? {
        // Create a sphere to mark the point - much larger radius for better visibility
        let sphereMesh = MeshResource.generateSphere(radius: 0.1) // 10cm sphere
        
        // Create a more vibrant material
        let material = SimpleMaterial(
            color: color.withAlphaComponent(0.9), 
            roughness: 0.1, 
            isMetallic: true
        )
        
        let entity = ModelEntity(mesh: sphereMesh, materials: [material])
        
        // Add to AR scene
        let anchor = AnchorEntity(world: position)
        anchor.addChild(entity)
        arView.scene.addAnchor(anchor)
        
        // Visual feedback - add a pulsing animation using RealityKit's animation system
        let scaleUp = Transform(scale: SIMD3<Float>(1.2, 1.2, 1.2))
        let scaleNormal = Transform(scale: SIMD3<Float>(1.0, 1.0, 1.0))
        
        // Create animation for the pulse effect
        entity.transform.scale = [1.0, 1.0, 1.0]
        
        // Use simple animations instead of the complex sequence
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
            entity.move(to: scaleUp, relativeTo: entity.parent, duration: 0.5, timingFunction: .easeInOut)
            
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                entity.move(to: scaleNormal, relativeTo: entity.parent, duration: 0.5, timingFunction: .easeInOut)
            }
        }
        
        return entity
    }
    
    @objc private func exportPath() {
        guard let geojson = arManager.exportPathToGeoJSON() else {
            let alert = UIAlertController(
                title: "Export Failed",
                message: "No valid path to export.",
                preferredStyle: .alert
            )
            alert.addAction(UIAlertAction(title: "OK", style: .default))
            present(alert, animated: true)
            return
        }
        
        // In a real app, you'd save this to a file or send it somewhere
        // For demo purposes, we'll show it in an alert
        let alert = UIAlertController(
            title: "Path Exported",
            message: "GeoJSON data ready for TurtleBot.\n\n\(geojson.prefix(200))...",
            preferredStyle: .alert
        )
        alert.addAction(UIAlertAction(title: "Copy", style: .default) { _ in
            UIPasteboard.general.string = geojson
        })
        alert.addAction(UIAlertAction(title: "OK", style: .default))
        present(alert, animated: true)
    }
    
    // MARK: - Public Methods
    
    func updateCuboidDimensions(width: Float, height: Float, depth: Float) {
        arManager.updateCuboidDimensions(width: width, height: height, depth: depth)
        // Update sliders to match
        widthSlider.value = width
        heightSlider.value = height
        depthSlider.value = depth
    }

    func updateCuboidPosition(x: Float, y: Float, z: Float) {
        arManager.updateCuboidPosition(x: x, y: y, z: z)
        // Update sliders to match
        posXSlider.value = x
        posYSlider.value = y
        posZSlider.value = z
    }
    
    // Add accessor method for ARManager
    func getARManager() -> ARManager? {
        return arManager
    }
}
