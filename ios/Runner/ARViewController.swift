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
        s.minimumValue = 10
        s.maximumValue = 100
        s.value = 70  // Default width: 70cm
        s.translatesAutoresizingMaskIntoConstraints = false
        return s
    }()
    
    private let heightSlider: UISlider = {
        let s = UISlider()
        s.minimumValue = 10
        s.maximumValue = 100
        s.value = 100  // Default height: 100cm
        s.translatesAutoresizingMaskIntoConstraints = false
        return s
    }()
    
    private let depthSlider: UISlider = {
        let s = UISlider()
        s.minimumValue = 10
        s.maximumValue = 100
        s.value = 100  // Default depth: 100cm
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

    // Add properties to store cuboid state when switching to path planning mode
    private var savedCuboidWidth: Float = 10.0
    private var savedCuboidHeight: Float = 20.0
    private var savedCuboidDepth: Float = 10.0
    private var savedCuboidX: Float = 0.0
    private var savedCuboidY: Float = 0.0
    private var savedCuboidZ: Float = -1.5
    
    // Add label for the controls section to update with mode
    private var controlsLabel: UILabel?

    // Add a property to track last path interaction time
    private var lastPathInteractionTime: TimeInterval = 0
    private let pathInteractionThrottle: TimeInterval = 1.0 // 1 second minimum between path interactions

    // Add a property to track which point we're currently editing
    private var editingStartPoint = false
    private var editPointToggleButton: UIButton?

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
        
        // Add a label to indicate current slider mode
        let modeLabel = UILabel()
        modeLabel.text = "Cuboid Controls"
        modeLabel.font = UIFont.boldSystemFont(ofSize: 16)
        modeLabel.textAlignment = .center
        modeLabel.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(modeLabel)
        
        NSLayoutConstraint.activate([
            modeLabel.bottomAnchor.constraint(equalTo: mainStack.topAnchor, constant: -8),
            modeLabel.centerXAnchor.constraint(equalTo: view.centerXAnchor)
        ])
        
        self.controlsLabel = modeLabel
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
        
        // Add a toggle button to switch between editing start and end points
        let editPointBtn = UIButton(type: .system)
        editPointBtn.setTitle("Edit: End Point", for: .normal)
        editPointBtn.backgroundColor = UIColor.systemTeal
        editPointBtn.setTitleColor(.white, for: .normal)
        editPointBtn.layer.cornerRadius = 8
        editPointBtn.translatesAutoresizingMaskIntoConstraints = false
        editPointBtn.titleLabel?.font = UIFont.boldSystemFont(ofSize: 16)
        editPointBtn.isHidden = true // Initially hidden until both points exist
        view.addSubview(editPointBtn)
        
        NSLayoutConstraint.activate([
            editPointBtn.topAnchor.constraint(equalTo: followPathButton!.bottomAnchor, constant: 8),
            editPointBtn.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            editPointBtn.widthAnchor.constraint(equalToConstant: 110),
            editPointBtn.heightAnchor.constraint(equalToConstant: 44)
        ])
        
        editPointBtn.addTarget(self, action: #selector(toggleEditPoint), for: .touchUpInside)
        self.editPointToggleButton = editPointBtn
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
    
    @objc private func handlePan(_ gesture: UIPanGestureRecognizer) {
        guard !pathPlanningMode, let entity = arManager.currentCuboidEntity else { return }
        let translation = gesture.translation(in: arView)
        let dx = Float(translation.x) * 0.001
        let dy = Float(-translation.y) * 0.001
        entity.transform.translation += SIMD3(dx, dy, 0)
        gesture.setTranslation(.zero, in: arView)
    }

    @objc private func handlePinch(_ gesture: UIPinchGestureRecognizer) {
        guard !pathPlanningMode, let entity = arManager.currentCuboidEntity else { return }
        let s = Float(gesture.scale)
        entity.scale *= SIMD3(repeating: s)
        gesture.scale = 1
    }

    @objc private func handleTap(_ gesture: UITapGestureRecognizer) {
        let location = gesture.location(in: arView)
        handlePathPlanningTap(location)
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
        // Prevent rapid repeated taps
        let now = Date().timeIntervalSince1970
        guard now - lastPathInteractionTime > pathInteractionThrottle else { return }
        lastPathInteractionTime = now
        
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
                    
                    // Get collision side info if path was blocked
                    let collisionSide = success ? "" : " (\(self.arManager.getLastCollisionSide()) side)"
                    
                    // Show result to user
                    let title = success ? "Path Complete" : "Path Blocked"
                    let message = success ? 
                        "The cuboid successfully reached the destination." :
                        "The cuboid's path was blocked by an obstacle\(collisionSide)."
                    
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

    @objc private func toggleEditPoint(_ sender: UIButton) {
        editingStartPoint = !editingStartPoint
        
        // Update button title to indicate which point is being edited
        if editingStartPoint {
            sender.setTitle("Edit: Start Pt", for: .normal)
            sender.backgroundColor = UIColor.systemGreen
            
            // Update sliders to match start point position
            if let startPos = startPointEntity?.position(relativeTo: nil) {
                posXSlider.value = startPos.x
                posYSlider.value = startPos.y
                posZSlider.value = startPos.z
            }
        } else {
            sender.setTitle("Edit: End Pt", for: .normal)
            sender.backgroundColor = UIColor.systemTeal
            
            // Update sliders to match goal point position
            if let goalPos = goalPointEntity?.position(relativeTo: nil) {
                posXSlider.value = goalPos.x
                posYSlider.value = goalPos.y
                posZSlider.value = goalPos.z
            }
        }
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

    // Fix the method signature to match what's expected
    @objc private func dimensionSliderChanged(_ sender: UISlider) {
        if pathPlanningMode {
            // In path planning mode, width/height/depth sliders don't apply
        } else {
            // Normal mode - control cuboid dimensions
            arManager.setCuboid(
                width: widthSlider.value,
                height: heightSlider.value,
                depth: depthSlider.value
            )
        }
    }
    
    @objc private func positionSliderChanged(_ sender: UISlider) {
        if pathPlanningMode {
            // In path planning mode, update path point positions
            updatePathPointsFromSliders(sender)
        } else {
            // Normal mode - control cuboid position
            arManager.updateCuboidPosition(
                x: posXSlider.value,
                y: posYSlider.value,
                z: posZSlider.value
            )
        }
    }
    
    private func handlePathPlanningTap(_ location: CGPoint) {
        // Throttle rapid taps
        let now = Date().timeIntervalSince1970
        guard now - lastPathInteractionTime > pathInteractionThrottle/2 else { return }
        lastPathInteractionTime = now
        
        guard pathPlanningMode else { return }
        
        // Use a more robust raycast query to ensure we get hits
        guard let raycast = arView.ray(through: location) else {
            print("Failed to create ray from touch point")
            return
        }
        
        // Raycast against any estimated surfaces (not just ground planes)
        let raycastQuery = ARRaycastQuery(
            origin: raycast.origin,
            direction: raycast.direction,
            allowing: .estimatedPlane, // Use estimated plane for more reliable hits
            alignment: .any
        )
        
        var hitPosition: SIMD3<Float>? = nil
        
        // Try to hit any surface to get the 3D position - keep the actual Y value
        if let result = arView.session.raycast(raycastQuery).first {
            let worldPosition = result.worldTransform.columns.3
            hitPosition = SIMD3<Float>(worldPosition.x, worldPosition.y, worldPosition.z)
        } else {
            // If no hit, use a ray at default distance
            let defaultDistance: Float = 1.5 // 1.5 meters in front
            let rayDirection = normalize(raycast.direction)
            hitPosition = raycast.origin + rayDirection * defaultDistance
        }
        
        // Ensure we have a hit position
        guard let position = hitPosition else {
            print("Failed to determine position")
            return
        }
        
        print("Placing marker at position: \(position)")
        
        if startPointEntity == nil {
            // Place start point at actual 3D position
            startPointEntity = createMarkerEntity(color: .systemGreen, position: position)
            startMarkerPosition = position
            print("Placed start marker")
        } else if goalPointEntity == nil {
            // Place goal point at actual 3D position
            goalPointEntity = createMarkerEntity(color: .systemBlue, position: position)
            goalMarkerPosition = position
            print("Placed goal marker")
            
            // Both points exist now, show the edit toggle button
            editPointToggleButton?.isHidden = false
            
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
        
        // Update slider values to match the tapped position (including Y)
        posXSlider.value = position.x
        posYSlider.value = position.y
        posZSlider.value = position.z
    }

    // Update method to allow editing both start and end points
    private func updatePathPointsFromSliders(_ sender: UISlider? = nil) {
        // Use the Y slider value for point height - don't enforce ground plane
        let pointY = posYSlider.value
        let position = SIMD3<Float>(posXSlider.value, pointY, posZSlider.value)
        
        // Determine which point to update based on whether we have a start point
        if startPointEntity == nil {
            // Create start point at slider position
            startPointEntity = createMarkerEntity(color: .systemGreen, position: position)
            startMarkerPosition = position
            print("Created start marker at position: \(position)")
        } else if goalPointEntity == nil {
            // Create goal point at slider position
            goalPointEntity = createMarkerEntity(color: .systemBlue, position: position)
            goalMarkerPosition = position
            print("Created goal marker at position: \(position)")
            
            // Both points exist now, show the edit toggle button
            editPointToggleButton?.isHidden = false
            
            // Plan path between the points
            arManager.setPathPoints(start: startMarkerPosition, goal: goalMarkerPosition)
            exportButton?.isHidden = false
            followPathButton?.isHidden = false
        } else {
            // Both points exist, update based on which one we're editing
            if sender == nil || sender == posXSlider || sender == posYSlider || sender == posZSlider {
                if editingStartPoint {
                    // Update start point
                    if let entity = startPointEntity, let parent = entity.parent {
                        parent.removeChild(entity)
                        startPointEntity = createMarkerEntity(color: .systemGreen, position: position)
                        startMarkerPosition = position
                    }
                } else {
                    // Update goal point
                    if let entity = goalPointEntity, let parent = entity.parent {
                        parent.removeChild(entity)
                        goalPointEntity = createMarkerEntity(color: .systemBlue, position: position)
                        goalMarkerPosition = position
                    }
                }
                
                // Update path with the new points
                arManager.setPathPoints(start: startMarkerPosition, goal: goalMarkerPosition)
            }
        }
    }

    // MARK: - Path Planning
    
    @objc private func togglePathPlanningMode() {
        // Add throttling here too
        let now = Date().timeIntervalSince1970
        guard now - lastPathInteractionTime > pathInteractionThrottle else { return }
        lastPathInteractionTime = now
        
        pathPlanningMode = !pathPlanningMode
        
        if pathPlanningMode {
            // Enter path planning mode
            pathPlanButton?.setTitle("Exit Path Mode", for: .normal)
            pathPlanButton?.backgroundColor = UIColor.systemRed
            
            // Save current cuboid settings
            savedCuboidWidth = widthSlider.value
            savedCuboidHeight = heightSlider.value
            savedCuboidDepth = depthSlider.value
            savedCuboidX = posXSlider.value
            savedCuboidY = posYSlider.value
            savedCuboidZ = posZSlider.value
            
            // Update slider labels and purpose
            controlsLabel?.text = "Path Point Controls"
            
            // Initialize sliders to camera/center position for placing points
            if let cameraPosition = arView.session.currentFrame?.camera.transform.columns.3 {
                posXSlider.value = cameraPosition.x
                posYSlider.value = cameraPosition.y // Use actual camera Y position
                posZSlider.value = cameraPosition.z
            }
            
            // Disable dimension sliders since they don't apply to path points
            widthSlider.isEnabled = false
            heightSlider.isEnabled = false
            depthSlider.isEnabled = false
            
            // Enable Y slider to allow height adjustment for path points
            posYSlider.isEnabled = true
            
            // Show instructions
            let alert = UIAlertController(
                title: "Path Planning Mode",
                message: "Use the X, Y and Z position sliders to place path points. First point is start, second is destination.",
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
            
            // Restore slider functionality
            controlsLabel?.text = "Cuboid Controls"
            
            // Re-enable all sliders
            widthSlider.isEnabled = true
            heightSlider.isEnabled = true
            depthSlider.isEnabled = true
            posYSlider.isEnabled = true
            
            // Restore cuboid settings
            widthSlider.value = savedCuboidWidth
            heightSlider.value = savedCuboidHeight
            depthSlider.value = savedCuboidDepth
            posXSlider.value = savedCuboidX
            posYSlider.value = savedCuboidY
            posZSlider.value = savedCuboidZ
            
            // Apply restored values to cuboid
            arManager.setCuboid(
                width: savedCuboidWidth,
                height: savedCuboidHeight,
                depth: savedCuboidDepth
            )
            
            arManager.updateCuboidPosition(
                x: savedCuboidX, 
                y: savedCuboidY, 
                z: savedCuboidZ
            )
            
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
        
        // Hide edit toggle since there are no points
        editPointToggleButton?.isHidden = true
        editingStartPoint = false
        editPointToggleButton?.setTitle("Edit: End Pt", for: .normal)
        editPointToggleButton?.backgroundColor = UIColor.systemTeal
    }
    
    private func createMarkerEntity(color: UIColor, position: SIMD3<Float>) -> ModelEntity? {
        // Create a sphere to mark the point - smaller radius for better precision
        let sphereMesh = MeshResource.generateSphere(radius: 0.03) // 3cm sphere instead of 10cm
        
        // Create a more vibrant material
        let material = SimpleMaterial(
            color: color.withAlphaComponent(0.9), 
            roughness: 0.1, 
            isMetallic: true
        )
        
        let entity = ModelEntity(mesh: sphereMesh, materials: [material])
        
        // Add to AR scene - ensure position is at ground level
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
