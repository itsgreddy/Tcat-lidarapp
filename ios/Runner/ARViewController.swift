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

    override func viewDidLoad() {
        super.viewDidLoad()
        setupARView()
        setupGestures()
        setupUI()
        requestCameraPermission()
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        arManager.cleanup()
    }
    
    // MARK: - Setup
    
    private func setupARView() {
        view.addSubview(arView)
        
        NSLayoutConstraint.activate([
            arView.topAnchor.constraint(equalTo: view.topAnchor),
            arView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            arView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            arView.bottomAnchor.constraint(equalTo: view.bottomAnchor)
        ])
    }
    
    private func setupGestures() {
        let pan = UIPanGestureRecognizer(target: self, action: #selector(handlePan(_:)))
        arView.addGestureRecognizer(pan)
        let pinch = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
        arView.addGestureRecognizer(pinch)
    }
    
    private func setupUI() {
        // status bar at top
        view.addSubview(statusView)
        NSLayoutConstraint.activate([
            statusView.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            statusView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            statusView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            statusView.heightAnchor.constraint(equalToConstant: 20)
        ])

        // sliders stack at bottom
        let stack = UIStackView(arrangedSubviews: [
            labeledSlider("W", widthSlider),
            labeledSlider("H", heightSlider),
            labeledSlider("D", depthSlider)
        ])
        stack.axis = .vertical
        stack.spacing = 8
        stack.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(stack)
        NSLayoutConstraint.activate([
            stack.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -16),
            stack.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 16),
            stack.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16)
        ])

        // slider actions
        [widthSlider, heightSlider, depthSlider].forEach {
            $0.addTarget(self, action: #selector(sliderChanged(_:)), for: .valueChanged)
        }
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

    @objc private func sliderChanged(_ s: UISlider) {
        arManager.updateCuboidDimensions(
            width: widthSlider.value,
            height: heightSlider.value,
            depth: depthSlider.value
        )
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
    
    // MARK: - Public Methods
    
    func updateCuboidDimensions(width: Float, height: Float, depth: Float) {
        arManager.updateCuboidDimensions(width: width, height: height, depth: depth)
    }
}
