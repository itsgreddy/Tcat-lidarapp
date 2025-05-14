import UIKit
import ARKit
import RealityKit
import simd

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
    // MARK: - Lifecycle Methods= false
    
    override func viewDidLoad() {
        super.viewDidLoad()
        setupARView()wDidLoad() {
        setupGestures()ad()
        setupUI()ew()
        setupPathPlanningUI()
        requestCameraPermission()
    }   setupPathPlanningUI()
        requestCameraPermission()
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        arManager.cleanup()isappear(_ animated: Bool) {
    }   super.viewWillDisappear(animated)
        arManager.cleanup()
    // MARK: - Setup Methods
    
    private func setupARView() {
        view.addSubview(arView)
        ate func setupARView() {
        NSLayoutConstraint.activate([
            arView.topAnchor.constraint(equalTo: view.topAnchor),
            arView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            arView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            arView.bottomAnchor.constraint(equalTo: view.bottomAnchor)r),
        ])  arView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            arView.bottomAnchor.constraint(equalTo: view.bottomAnchor)
        // Start with debug options disabled
        // User can toggle them with the "Show Mesh" button
        arView.debugOptions = []ons disabled
        // User can toggle them with the "Show Mesh" button
        // Enable mesh anchors in the configuration when we initialize AR
        // This ensures the mesh data is available even if visualization is off
        if let config = arView.session.configuration as? ARWorldTrackingConfiguration {
            if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
                config.sceneReconstruction = .meshon as? ARWorldTrackingConfiguration {
            }f ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            arView.session.run(config)tion = .mesh
        }   }
    }       arView.session.run(config)
        }
    private func setupGestures() {
        let pan = UIPanGestureRecognizer(target: self, action: #selector(handlePan(_:)))
        arView.addGestureRecognizer(pan)
        let pan = UIPanGestureRecognizer(target: self, action: #selector(handlePan(_:)))
        let pinch = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
        arView.addGestureRecognizer(pinch)
        let pinch = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
        let tap = UITapGestureRecognizer(target: self, action: #selector(handleTap(_:)))
        arView.addGestureRecognizer(tap)
    }   let tap = UITapGestureRecognizer(target: self, action: #selector(handleTap(_:)))
        arView.addGestureRecognizer(tap)
    private func setupUI() {
        // Status bar at top
        view.addSubview(statusView)
        NSLayoutConstraint.activate([
            statusView.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            statusView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            statusView.trailingAnchor.constraint(equalTo: view.trailingAnchor),opAnchor),
            statusView.heightAnchor.constraint(equalToConstant: 20)ngAnchor),
        ])  statusView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            statusView.heightAnchor.constraint(equalToConstant: 20)
        // Create two columns of sliders
        let leftStack = UIStackView(arrangedSubviews: [
            labeledSlider("W", widthSlider),
            labeledSlider("H", heightSlider),ubviews: [
            labeledSlider("D", depthSlider),
        ])  labeledSlider("H", heightSlider),
        leftStack.axis = .verticalthSlider)
        leftStack.spacing = 8
        leftStack.translatesAutoresizingMaskIntoConstraints = false
        leftStack.spacing = 8
        let rightStack = UIStackView(arrangedSubviews: [nts = false
            labeledSlider("X", posXSlider),
            labeledSlider("Y", posYSlider),edSubviews: [
            labeledSlider("Z", posZSlider),
        ])  labeledSlider("Y", posYSlider),
        rightStack.axis = .verticalSlider)
        rightStack.spacing = 8
        rightStack.translatesAutoresizingMaskIntoConstraints = false
        rightStack.spacing = 8
        // Create a horizontal stack to hold both columnsnts = false
        let mainStack = UIStackView(arrangedSubviews: [leftStack, rightStack])
        mainStack.axis = .horizontal to hold both columns
        mainStack.spacing = 16kView(arrangedSubviews: [leftStack, rightStack])
        mainStack.distribution = .fillEqually
        mainStack.translatesAutoresizingMaskIntoConstraints = false
        mainStack.distribution = .fillEqually
        view.addSubview(mainStack)sizingMaskIntoConstraints = false
        NSLayoutConstraint.activate([
            mainStack.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -16),
            mainStack.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 16),
            mainStack.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16), constant: -16),
        ])  mainStack.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 16),
            mainStack.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16)
        // Slider actions
        [widthSlider, heightSlider, depthSlider].forEach {
            $0.addTarget(self, action: #selector(dimensionSliderChanged(_:)), for: .valueChanged)
        }widthSlider, heightSlider, depthSlider].forEach {
            $0.addTarget(self, action: #selector(dimensionSliderChanged(_:)), for: .valueChanged)
        [posXSlider, posYSlider, posZSlider].forEach {
            $0.addTarget(self, action: #selector(positionSliderChanged(_:)), for: .valueChanged)
        }posXSlider, posYSlider, posZSlider].forEach {
            $0.addTarget(self, action: #selector(positionSliderChanged(_:)), for: .valueChanged)
        // Add lock/unlock button
        view.addSubview(lockButton)
        NSLayoutConstraint.activate([
            lockButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
            lockButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16)
        ])  lockButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
        lockButton.addTarget(self, action: #selector(toggleLock(_:)), for: .touchUpInside)16)
    }   ])
        lockButton.addTarget(self, action: #selector(toggleLock(_:)), for: .touchUpInside)
    private func setupPathPlanningUI() {
        // Path planning mode button - Make it more prominent
        let planButton = UIButton(type: .system)
        planButton.setTitle("Plan Path", for: .normal)ominent
        planButton.backgroundColor = UIColor.systemBlue
        planButton.setTitleColor(.white, for: .normal)
        planButton.layer.cornerRadius = 8lor.systemBlue
        planButton.translatesAutoresizingMaskIntoConstraints = false
        planButton.titleLabel?.font = UIFont.boldSystemFont(ofSize: 16)
        view.addSubview(planButton)sizingMaskIntoConstraints = false
        planButton.titleLabel?.font = UIFont.boldSystemFont(ofSize: 16)
        NSLayoutConstraint.activate([
            planButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
            planButton.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            planButton.widthAnchor.constraint(equalToConstant: 110),youtGuide.topAnchor, constant: 8),
            planButton.heightAnchor.constraint(equalToConstant: 44)ngAnchor, constant: 20),
        ])  planButton.widthAnchor.constraint(equalToConstant: 110),
            planButton.heightAnchor.constraint(equalToConstant: 44)
        planButton.addTarget(self, action: #selector(togglePathPlanningMode), for: .touchUpInside)
        self.pathPlanButton = planButton
        planButton.addTarget(self, action: #selector(togglePathPlanningMode), for: .touchUpInside)
        // Export path button - Also make it more visible
        let exportBtn = UIButton(type: .system)
        exportBtn.setTitle("Export Path", for: .normal)le
        exportBtn.backgroundColor = UIColor.systemGreen
        exportBtn.setTitleColor(.white, for: .normal)l)
        exportBtn.layer.cornerRadius = 8lor.systemGreen
        exportBtn.translatesAutoresizingMaskIntoConstraints = false
        exportBtn.titleLabel?.font = UIFont.boldSystemFont(ofSize: 16)
        exportBtn.isHidden = trueesizingMaskIntoConstraints = false
        view.addSubview(exportBtn) = UIFont.boldSystemFont(ofSize: 16)
        exportBtn.isHidden = true
        NSLayoutConstraint.activate([
            exportBtn.topAnchor.constraint(equalTo: planButton.bottomAnchor, constant: 8),
            exportBtn.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            exportBtn.widthAnchor.constraint(equalToConstant: 110),omAnchor, constant: 8),
            exportBtn.heightAnchor.constraint(equalToConstant: 44)ngAnchor, constant: 20),
        ])  exportBtn.widthAnchor.constraint(equalToConstant: 110),
            exportBtn.heightAnchor.constraint(equalToConstant: 44)
        exportBtn.addTarget(self, action: #selector(exportPath), for: .touchUpInside)
        self.exportButton = exportBtn
        exportBtn.addTarget(self, action: #selector(exportPath), for: .touchUpInside)
        // Add debug/mesh visibility toggle button
        let meshBtn = UIButton(type: .system)
        meshBtn.setTitle("Show Mesh", for: .normal)
        meshBtn.backgroundColor = UIColor.systemPurple
        meshBtn.setTitleColor(.white, for: .normal)
        meshBtn.layer.cornerRadius = 8lor.systemPurple
        meshBtn.translatesAutoresizingMaskIntoConstraints = false
        meshBtn.titleLabel?.font = UIFont.boldSystemFont(ofSize: 16)
        view.addSubview(meshBtn)sizingMaskIntoConstraints = false
        meshBtn.titleLabel?.font = UIFont.boldSystemFont(ofSize: 16)
        NSLayoutConstraint.activate([
            meshBtn.topAnchor.constraint(equalTo: exportBtn.bottomAnchor, constant: 8),
            meshBtn.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 20),
            meshBtn.widthAnchor.constraint(equalToConstant: 110),mAnchor, constant: 8),
            meshBtn.heightAnchor.constraint(equalToConstant: 44)ngAnchor, constant: 20),
        ])  meshBtn.widthAnchor.constraint(equalToConstant: 110),
            meshBtn.heightAnchor.constraint(equalToConstant: 44)
        meshBtn.addTarget(self, action: #selector(toggleMeshVisibility), for: .touchUpInside)
        self.meshButton = meshBtn
    }   meshBtn.addTarget(self, action: #selector(toggleMeshVisibility), for: .touchUpInside)
        self.meshButton = meshBtn
    private func labeledSlider(_ label: String, _ slider: UISlider) -> UIView {
        let lbl = UILabel()button
        lbl.text = labelUIButton(type: .system)
        lbl.widthAnchor.constraint(equalToConstant: 20).isActive = true
        let hstack = UIStackView(arrangedSubviews: [lbl, slider])
        hstack.axis = .horizontalwhite, for: .normal)
        hstack.spacing = 8rnerRadius = 8
        return hstacknslatesAutoresizingMaskIntoConstraints = false
    }   followBtn.titleLabel?.font = UIFont.boldSystemFont(ofSize: 16)
        followBtn.isHidden = true  // Initially hidden until path is created
    // MARK: - Action HandlersBtn)
        
    @objc private func dimensionSliderChanged(_ s: UISlider) {
        arManager.setCuboid(hor.constraint(equalTo: meshBtn.bottomAnchor, constant: 8),
            width: widthSlider.value,onstraint(equalTo: view.leadingAnchor, constant: 20),
            height: heightSlider.value,raint(equalToConstant: 110),
            depth: depthSlider.valueonstraint(equalToConstant: 44)
        ))
    }   
        followBtn.addTarget(self, action: #selector(toggleFollowPath), for: .touchUpInside)
    @objc private func positionSliderChanged(_ s: UISlider) {
        arManager.updateCuboidPosition(
            x: posXSlider.value,
            y: posYSlider.value, label: String, _ slider: UISlider) -> UIView {
            z: posZSlider.value
        )bl.text = label
    }   lbl.widthAnchor.constraint(equalToConstant: 20).isActive = true
        let hstack = UIStackView(arrangedSubviews: [lbl, slider])
    @objc private func toggleLock(_ sender: UIButton) {
        if isLocked {g = 8
            arManager.unlockCuboid()
            sender.setTitle("Lock", for: .normal)
        } else {
            arManager.lockCuboid()
            sender.setTitle("Unlock", for: .normal)
        } private func dimensionSliderChanged(_ s: UISlider) {
        isLocked.toggle()id(
    }       width: widthSlider.value,
            height: heightSlider.value,
    @objc private func toggleMeshVisibility(_ sender: UIButton) {
        // Toggle between showing mesh and hiding mesh
        if arView.debugOptions.contains(.showSceneUnderstanding) {
            // Currently showing, so hide it
            arView.debugOptions = []rChanged(_ s: UISlider) {
            sender.setTitle("Show Mesh", for: .normal)
            sender.backgroundColor = UIColor.systemPurple
        } else {osYSlider.value,
            // Currently hidden, so show it
            arView.debugOptions = [.showSceneUnderstanding, .showFeaturePoints]
            sender.setTitle("Hide Mesh", for: .normal)
            sender.backgroundColor = UIColor.systemOrange
        } private func toggleLock(_ sender: UIButton) {
    }   if isLocked {
            arManager.unlockCuboid()
    @objc private func handlePan(_ gesture: UIPanGestureRecognizer) {
        guard let entity = arManager.currentCuboidEntity else { return }
        let translation = gesture.translation(in: arView)
        let dx = Float(translation.x) * 0.001ormal)
        let dy = Float(-translation.y) * 0.001
        entity.transform.translation += SIMD3(dx, dy, 0)
        gesture.setTranslation(.zero, in: arView)
    }
    @objc private func toggleMeshVisibility(_ sender: UIButton) {
    @objc private func handlePinch(_ gesture: UIPinchGestureRecognizer) {
        guard let entity = arManager.currentCuboidEntity else { return }
        let s = Float(gesture.scale) hide it
        entity.scale *= SIMD3(repeating: s)
        gesture.scale = 1le("Show Mesh", for: .normal)
    }       sender.backgroundColor = UIColor.systemPurple
        } else {
    @objc private func handleTap(_ gesture: UITapGestureRecognizer) {
        let location = gesture.location(in: arView)tanding, .showFeaturePoints]
        handlePathPlanningTap(location), for: .normal)
    }       sender.backgroundColor = UIColor.systemOrange
        }
    private func requestCameraPermission() {
        AVCaptureDevice.requestAccess(for: .video) { [weak self] granted in
            DispatchQueue.main.async {ture: UIPanGestureRecognizer) {
                if granted {rManager.currentCuboidEntity else { return }
                    self?.initializeAR()ation(in: arView)
                } else {ranslation.x) * 0.001
                    // Show alert about missing camera permission
                    let alert = UIAlertController(dy, 0)
                        title: "Camera Access Required",
                        message: "This app needs camera access to use LiDAR scanning features.",
                        preferredStyle: .alert
                    )c handlePinch(_ gesture: UIPinchGestureRecognizer) {
                    alert.addAction(UIAlertAction(title: "OK", style: .default))
                    self?.present(alert, animated: true)
                }ale *= SIMD3(repeating: s)
            }re.scale = 1
        }
    }
    @objc private func handleTap(_ gesture: UITapGestureRecognizer) {
    private func initializeAR() {cation(in: arView)
        arManager.initialize(arView: arView) { [weak self] isIntersecting in
            DispatchQueue.main.async {
                self?.statusView.backgroundColor = isIntersecting ? .red : .green
                self?.onIntersectionUpdate?(isIntersecting)
            }tureDevice.requestAccess(for: .video) { [weak self] granted in
        }   DispatchQueue.main.async {
    }           if granted {
                    self?.initializeAR()
    // MARK: - Path Planning
                    // Show alert about missing camera permission
    @objc private func togglePathPlanningMode() {(
        pathPlanningMode = !pathPlanningModes Required",
                        message: "This app needs camera access to use LiDAR scanning features.",
        if pathPlanningMode {rredStyle: .alert
            // Enter path planning mode
            pathPlanButton?.setTitle("Cancel", for: .normal)", style: .default))
            pathPlanButton?.backgroundColor = UIColor.systemRed
                }
            // Show instructions
            let alert = UIAlertController(
                title: "Path Planning",
                message: "Tap to place start point, then tap again to place destination.",
                preferredStyle: .alert
            )ager.initialize(arView: arView) { [weak self] isIntersecting in
            alert.addAction(UIAlertAction(title: "OK", style: .default))
            present(alert, animated: true)dColor = isIntersecting ? .red : .green
                self?.onIntersectionUpdate?(isIntersecting)
            // Clear existing points
            removePathPoints()
        } else {
            // Exit path planning mode
            pathPlanButton?.setTitle("Plan Path", for: .normal)
            pathPlanButton?.backgroundColor = UIColor.systemBlue
            exportButton?.isHidden = trueMode() {
            PlanningMode = !pathPlanningMode
            // Clear path and points
            removePathPoints()
        }   // Enter path planning mode
    }       pathPlanButton?.setTitle("Cancel", for: .normal)
            pathPlanButton?.backgroundColor = UIColor.systemRed
    private func removePathPoints() {
        if let entity = startPointEntity, let parent = entity.parent {
            parent.removeChild(entity)ler(
            startPointEntity = niling",
        }       message: "Tap to place start point, then tap again to place destination.",
                preferredStyle: .alert
        if let entity = goalPointEntity, let parent = entity.parent {
            parent.removeChild(entity)ion(title: "OK", style: .default))
            goalPointEntity = niled: true)
        }   
    }       // Clear existing points
            removePathPoints()
    private func handlePathPlanningTap(_ location: CGPoint) {
        guard pathPlanningMode else { return }
            pathPlanButton?.setTitle("Plan Path", for: .normal)
        // Use a more robust raycast query to ensure we get hits
        guard let raycast = arView.ray(through: location) else {
            print("Failed to create ray from touch point")
            return
        }   // Clear path and points
            removePathPoints()
        var raycastQuery = ARRaycastQuery(
            origin: raycast.origin,
            direction: raycast.direction,
            allowing: .estimatedPlane,
            alignment: .anyrtPointEntity, let parent = entity.parent {
        )   parent.removeChild(entity)
            startPointEntity = nil
        // Fallbacks for raycast in case first attempt fails
        let raycastMethods: [ARRaycastQuery.Target] = [
            .estimatedPlane,PointEntity, let parent = entity.parent {
            .existingPlaneInfinite,ty)
            .existingPlaneGeometry
        ]
        
        var raycastResult: ARRaycastResult?
        ate func handlePathPlanningTap(_ location: CGPoint) {
        // Try different raycast methods until we get a hit
        for method in raycastMethods {
            if let ray = arView.ray(through: location) {get hits
                raycastQuery = ARRaycastQuery(: location) else {
                    origin: ray.origin, from touch point")
                    direction: ray.direction,
                    allowing: method,
                    alignment: .any
                )stQuery = ARRaycastQuery(
                in: raycast.origin,
                if let result = arView.session.raycast(raycastQuery).first {
                    raycastResult = result
                    breakny
                }
            }
        }/ Fallbacks for raycast in case first attempt fails
        let raycastMethods: [ARRaycastQuery.Target] = [
        // If all raycasts failed, try to place at a reasonable default distance
        if raycastResult == nil {e,
            let defaultDistance: Float = 1.5 // 1.5 meters in front
            guard let ray = arView.ray(through: location) else {
                print("Failed to create ray for default position")
                returnult: ARRaycastResult?
            }
            let defaultPosition = ray.origin + ray.direction * defaultDistance
            method in raycastMethods {
            if startPointEntity == nil {ugh: location) {
                startPointEntity = createMarkerEntity(color: .systemGreen, position: defaultPosition)
                print("Placed start marker at default position")
                returnrection: ray.direction,
            } else if goalPointEntity == nil {
                goalPointEntity = createMarkerEntity(color: .systemBlue, position: defaultPosition)
                print("Placed goal marker at default position")
                
                if let start = startPointEntity?.position, let goal = goalPointEntity?.position {
                    print("Setting path points: start=\(start), goal=\(goal)")
                    arManager.setPathPoints(start: start, goal: goal)
                    exportButton?.isHidden = false
                }
                return
            }
        }/ If all raycasts failed, try to place at a reasonable default distance
        if raycastResult == nil {
        guard let hitResult = raycastResult else {5 meters in front
            print("Failed to place marker - no raycast hit")se {
            returnint("Failed to create ray for default position")
        }       return
            }
        let worldPosition = hitResult.worldTransform.columns.3 defaultDistance
        let position = SIMD3<Float>(worldPosition.x, worldPosition.y, worldPosition.z)
            if startPointEntity == nil {
        print("Raycast hit at position: \(position)")(color: .systemGreen, position: defaultPosition)
                print("Placed start marker at default position")
        if startPointEntity == nil {
            // Place start pointntity == nil {
            startPointEntity = createMarkerEntity(color: .systemGreen, position: position)Position)
            print("Placed start marker")r at default position")
        } else if goalPointEntity == nil {
            // Place goal pointstartPointEntity?.position, let goal = goalPointEntity?.position {
            goalPointEntity = createMarkerEntity(color: .systemBlue, position: position)
            print("Placed goal marker")ints(start: start, goal: goal)
                    exportButton?.isHidden = false
            // Calculate path between pointsen = false
            if let start = startPointEntity?.position, let goal = goalPointEntity?.position {
                print("Setting path points: start=\(start), goal=\(goal)")
                arManager.setPathPoints(start: start, goal: goal)
                exportButton?.isHidden = false
            }
        } else {t hitResult = raycastResult else {
            print("Failed to place marker - no raycast hit")
            return
        }
        
        let worldPosition = hitResult.worldTransform.columns.3
        let position = SIMD3<Float>(worldPosition.x, worldPosition.y, worldPosition.z)
        
        print("Raycast hit at position: \(position)")
        
        if startPointEntity == nil {
            // Place start point
            startPointEntity = createMarkerEntity(color: .systemGreen, position: position)
            print("Placed start marker")
        } else if goalPointEntity == nil {
            // Place goal point
            goalPointEntity = createMarkerEntity(color: .systemBlue, position: position)
            print("Placed goal marker")
            
            // Calculate path between points
            if let start = startPointEntity?.position, let goal = goalPointEntity?.position {
                print("Setting path points: start=\(start), goal=\(goal)")
                arManager.setPathPoints(start: start, goal: goal)
                exportButton?.isHidden = false
                followPathButton?.isHidden = false
            }
        } else {
            // Replace both points
            print("Removing existing path points")
            removePathPoints()
            startPointEntity = createMarkerEntity(color: .systemGreen, position: position)
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
            preferredStyle: .alert    // Add accessor method for ARManager



































































}    }        posZSlider.value = z        posYSlider.value = y        posXSlider.value = x        // Update sliders to match        arManager.updateCuboidPosition(x: x, y: y, z: z)    func updateCuboidPosition(x: Float, y: Float, z: Float) {    }        depthSlider.value = depth        heightSlider.value = height        widthSlider.value = width        // Update sliders to match        arManager.updateCuboidDimensions(width: width, height: height, depth: depth)    func updateCuboidDimensions(width: Float, height: Float, depth: Float) {        // MARK: - Public Methods        }        }            arManager.cancelPathFollowing()            isFollowingPath = false            sender.backgroundColor = UIColor.systemYellow            sender.setTitle("Follow Path", for: .normal)            // Cancel path following        } else {            }                }                    self.present(alert, animated: true)                    alert.addAction(UIAlertAction(title: "OK", style: .default))                    )                        preferredStyle: .alert                        message: message,                        title: title,                    let alert = UIAlertController(                                            "The cuboid's path was blocked by an obstacle."                        "The cuboid successfully reached the destination." :                    let message = success ?                     let title = success ? "Path Complete" : "Path Blocked"                    // Show result to user                                        self.isFollowingPath = false                    self.followPathButton?.backgroundColor = UIColor.systemYellow                    self.followPathButton?.setTitle("Follow Path", for: .normal)                    // Animation finished                DispatchQueue.main.async {                guard let self = self else { return }            arManager.followPath { [weak self] success in            // Start the cuboid moving along the path                        isFollowingPath = true            sender.backgroundColor = UIColor.systemOrange            sender.setTitle("Cancel", for: .normal)            // Start following path        if !isFollowingPath {    @objc private func toggleFollowPath(_ sender: UIButton) {        }        present(alert, animated: true)        alert.addAction(UIAlertAction(title: "OK", style: .default))        })            UIPasteboard.general.string = geojson        alert.addAction(UIAlertAction(title: "Copy", style: .default) { _ in        )    func getARManager() -> ARManager? {
        return arManager
    }
}
