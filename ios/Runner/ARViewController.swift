import UIKit
import ARKit
import RealityKit

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
    
    override func viewDidLoad() {
        super.viewDidLoad()
        setupARView()
        
        // Check camera permission
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
        // Initialize AR with a callback for intersection updates
        arManager.initialize(arView: arView) { [weak self] isIntersecting in
            self?.onIntersectionUpdate?(isIntersecting)
        }
    }
    
    // MARK: - Public Methods
    
    func updateCuboidDimensions(width: Float, height: Float, depth: Float) {
        arManager.updateCuboidDimensions(width: width, height: height, depth: depth)
    }
}
