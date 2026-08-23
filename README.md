# Pelorus

[![CI](https://github.com/Aneesh495/pelorus/actions/workflows/ci.yml/badge.svg)](https://github.com/Aneesh495/pelorus/actions/workflows/ci.yml)
[![Language](https://img.shields.io/badge/Language-Odin-blue.svg)](https://odin-lang.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-macOS%20ARM64%20%7C%20Linux%20x86__64-orange.svg)](#)
[![Tests](https://img.shields.io/badge/Tests-39%2F39%20Passing-brightgreen.svg)](#)

Pelorus is a high-performance, real-time 3D spatial perception and kinodynamic motion planning engine built from first principles in the Odin programming language. Designed for autonomous robotic platforms, micro aerial vehicles (MAVs), and solid-state/spinning LiDAR systems, Pelorus executes continuous-time state estimation on $\text{SE}(3)$, motion deskewing, sparse hash-indexed volumetric mapping, dynamic multi-target obstacle tracking, and collision-free polynomial trajectory generation at multi-kilohertz throughput.

```
                              +-------------------------+
                              |   Raw LiDAR UDP Stream  |
                              +------------+------------+
                                           |
                                           v
+------------------------+    +-------------------------+
| High-Rate IMU Telemetry|    |  Continuous SE(3) Deskew|
| (200 Hz Accel / Gyro)  |    |  (Geodesic Slerp / Lie) |
+-----------+------------+    +------------+------------+
            |                              |
            v                              v
+------------------------+    +-------------------------+
| 15-State Error-State   |<---+   Point-to-Plane ICP    |
| Kalman Filter (ESKF)   |    |   & 3D NDT Alignment    |
+-----------+------------+    +------------+------------+
            |                              |
            | Continuous Pose              +--------------------+
            v                              |                    |
+------------------------+                 v                    v
| Sparse 3D TSDF Map     |    +--------------------+  +--------------------+
| (Morton Z-Order Hash)  |    | Ground Plane Seg   |  | Euclidean DBSCAN   |
+-----------+------------+    | (Progressive RANSAC)| | 3D Clustering      |
            |                 +--------------------+  +---------+----------+
            v                                                   |
+------------------------+                                      v
| Incremental ESDF Grid  |                            +--------------------+
| (Wavefront Clearance)  |                            | 7D CTRV EKF MOT    |
+-----------+------------+                            | (Hungarian Gating) |
            |                                         +---------+----------+
            v                                                   |
+------------------------+                                      |
| Kinodynamic RRT*       |                                      |
| & Quintic Spline Plan  |                                      |
+-----------+------------+                                      |
            |                                                   |
            +---------------------------+-----------------------+
                                        |
                                        v
                            +-----------------------+
                            | Telemetry Broadcaster |
                            | (Embedded HTTP / SSE) |
                            +-----------+-----------+
                                        |
                                        v
                            +-----------------------+
                            | 3D WebGL Dashboard    |
                            | & Native Python SDK   |
                            +-----------------------+
```

---

## Technical Highlights

### Continuous-Time Lie Theory on $\text{SE}(3)$
- Full analytical exponential and logarithmic maps for both $\text{SO}(3)$ and $\text{SE}(3)$.
- 4th-order Taylor series expansions for infinitesimal rotations ($\theta < 10^{-4}$), eliminating divide-by-zero singularities.
- Left and right Jacobians of $\text{SO}(3)$ and their exact inverses for closed-form Lie algebra transport.
- Continuous geodesic $\text{SE}(3)$ trajectory buffering and querying via screw theory interpolation.

### 15-State Error-State Kalman Filter (ESKF)
- Continuous-time kinematic propagation driven by raw accelerometer and gyroscope measurements.
- 15-state nominal and error state parameterization: position ($\mathbb{R}^3$), linear velocity ($\mathbb{R}^3$), attitude quaternion ($\mathbb{H}$), accelerometer bias ($\mathbb{R}^3$), and gyroscope bias ($\mathbb{R}^3$).
- Discrete state transition matrix $\boldsymbol{\Phi}$ integrated via second-order matrix exponential approximation.
- Joseph-form covariance stabilization: $\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H}) \mathbf{P} (\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K} \mathbf{R} \mathbf{K}^T$, strictly guaranteeing numerical symmetry and positive semi-definiteness across millions of updates.

### Sub-Millisecond 3D Scan Matching
- **Point-to-Plane ICP**: Linearized Gauss-Newton normal equations solved via Cholesky $LL^T$ decomposition, equipped with Huber robust loss reweighting to reject geometric outliers.
- **3D Normal Distributions Transform (NDT)**: Probability density representation modeling local voxel distributions with exact mean vectors and covariance ellipsoids.
- Adaptive keyframe vertex manager with geodesic translation and rotation threshold heuristics.

### Sparse Volumetric TSDF & ESDF Mapping
- **64-bit Morton Z-Order Hashing**: Interleaves 3D coordinates $(x, y, z)$ into bitwise Dilated Morton codes, maximizing spatial memory locality.
- **Robin Hood Open Addressing**: Hash table with backward shift deletion, achieving uniform probe distributions and $O(1)$ lookups even at 85% load factor.
- **Amanatides-Woo Fast Voxel Ray Traversal**: Integer step grid marching for truncated signed distance updates.
- **Incremental ESDF Wavefront Propagation**: Two-pass BFS distance transform computing continuous Euclidean obstacle clearance and spatial gradients for collision evaluation.

### Dynamic Multi-Target Obstacle Tracking (MOT)
- 3D Euclidean spatial hashing cluster extraction (DBSCAN equivalent in $O(N)$ expected time).
- Principal Component Analysis (PCA) and Rotating Calipers oriented 3D bounding box fitting.
- 7D Constant Turn Rate and Velocity (CTRV) Extended Kalman Filter: $[p_x, p_y, p_z, v, \psi, \dot{\psi}, v_z]^T$.
- Hungarian data association algorithm paired with $\chi^2$ Mahalanobis gating.
- Full track lifecycle state machine: Tentative $\to$ Confirmed $\to$ Coasting $\to$ Terminated.

### Kinodynamic Motion Planning & Trajectory Optimization
- **Convex Safe Flight Corridors**: Overlapping axis-aligned bounding boxes inflated through ESDF obstacle distance field queries.
- **Kinodynamic RRT\***: Asymptotically optimal path search in $\mathbb{R}^3$ with bounded steering angles and rewiring.
- **Closed-Form Minimum-Jerk Quintic Splines**: Piecewise 5th-order polynomial trajectory generation satisfying boundary conditions (position, velocity, acceleration at knots) via analytical $3 \times 3$ inversion, guaranteeing continuous $C^2$ profiles with zero runtime optimization latency.
- **Pure Pursuit and Stanley Controllers**: Path tracking controllers with speed-dependent lookahead and signed lateral cross-track error feedback.

### Zero-Copy Systems Architecture
- Cache-aligned Structure of Arrays (`PointBufferSOA`) with SIMD vectorization primitives.
- Monotonic bump allocation (`Arena`) and fixed-size free-list object pools (`ObjectPool`).
- Lock-free Single-Producer Single-Consumer (`RingBuffer`) for high-throughput sensor telemetry.
- Embedded HTTP and Server-Sent Events (SSE) telemetry server streaming live perception frames at 60 Hz to browser dashboards without third-party dependencies.

---

## Benchmark Results

Benchmarks executed on Apple Silicon (M-series ARM64, 8 cores, macOS 15.0) compiled with `odin build ... -o:speed`:

| Benchmark Component | Throughput | Latency | Complexity |
|:---|:---:|:---:|:---:|
| **$\text{SE}(3)$ Geodesic Exp & Multiply** | 3.32 M ops/sec | 301.67 ns | $O(1)$ analytical |
| **Morton 64-bit Encode & Decode** | 46.79 M ops/sec | 21.37 ns | $O(1)$ bitwise |
| **Robin Hood Voxel Hash Lookup** | 72.04 M ops/sec | 13.88 ns | $O(1)$ expected |
| **Point-to-Plane ICP (300 points)** | 11,507 iters/sec | 0.09 ms | $O(N)$ Gauss-Newton |
| **TSDF Ray Integration (1k points)** | 5,635,655 pts/sec | 0.18 ms | $O(N \cdot L)$ raycast |
| **Quintic Spline Solve (5 knots)** | 11,235,955 solves/sec | 0.09 $\mu$s | $O(K)$ closed-form |
| **End-to-End Pipeline Loop** | 100+ frames/sec | 9.80 ms | Full perception cycle |

---

## Codebase Architecture

```
pelorus/
├── .github/
│   └── workflows/
│       └── ci.yml                 # Cross-platform CI (macOS ARM64 & Ubuntu)
├── bin/                           # Compiled binaries (pelorus, pelorus_bench)
├── bench/
│   └── bench_main.odin            # Micro and macro performance benchmark suite
├── docs/
│   ├── architecture.md            # Detailed system design & sequence diagrams
│   └── math.md                    # Rigorous mathematical derivations
├── include/
│   └── pelorus.h                  # Clean C ABI interface header
├── lib/
│   └── libpelorus.dylib           # Compiled native dynamic library
├── python/
│   ├── pelorus/
│   │   ├── __init__.py            # Python SDK entrypoint
│   │   ├── bindings.py            # Ctypes wrapper for libpelorus
│   │   ├── dataset.py             # Synthetic LiDAR-inertial benchmark generator
│   │   └── evaluation.py          # ATE / RPE trajectory evaluation with Umeyama
│   └── setup.py                   # Python package installer
├── scripts/
│   ├── bench.sh                   # Benchmark execution script
│   ├── build.sh                   # Release compilation script
│   └── test.sh                    # Automated test runner script
├── src/
│   ├── main.odin                  # Unified CLI executable
│   ├── engine.odin                # Top-level PelorusEngine coordinator
│   ├── c_api/
│   │   └── export.odin            # C ABI dynamic library exports (@export)
│   ├── core/                      # Lie groups, SE(3), geometry, memory arenas
│   │   ├── types.odin             # Vector, matrix, quaternion primitives
│   │   ├── simd.odin              # 4-way SIMD float vector math
│   │   ├── lie.odin               # SO(3) exponential and logarithmic maps
│   │   ├── se3.odin               # SE(3) rigid transformations and adjoints
│   │   ├── interpolation.odin     # Continuous-time SE(3) trajectory buffering
│   │   ├── geometry.odin          # AABB, OBB, Ray, and Plane3D primitives
│   │   ├── gjk.odin               # Gilbert-Johnson-Keerthi convex collision
│   │   ├── statistics.odin        # Cholesky decomposition & Mahalanobis metric
│   │   └── memory/
│   │       ├── arena.odin         # Monotonic bump arena allocator
│   │       ├── pool.odin          # Fixed-size free-list object pool
│   │       ├── ring.odin          # Lock-free SPSC circular ring buffer
│   │       └── layout.odin        # Cache-aligned Structure of Arrays point buffer
│   ├── sensor/                    # Sensor ingestion, deskewing, packets
│   │   ├── types.odin             # PointCloud, Point3D, IMU, NavState
│   │   ├── imu_preintegration.odin# Continuous IMU delta rotation/velocity/position
│   │   ├── imu_bias.odin          # Online bias Jacobian propagation
│   │   ├── deskew.odin            # Motion deskewing via SE(3) interpolation
│   │   └── pcap.odin              # High-throughput LiDAR packet parser
│   ├── filter/                    # Downsampling, outlier rejection, segmentation
│   │   ├── voxel_grid.odin        # 3D spatial voxel grid downsampling
│   │   ├── passthrough.odin       # AABB bounding and azimuthal FOV clipping
│   │   ├── outlier.odin           # Statistical Outlier Removal (SOR)
│   │   ├── radius_outlier.odin    # Radius Outlier Removal (ROR)
│   │   ├── ground.odin            # Progressive RANSAC ground plane segmentation
│   │   └── normal_estimator.odin  # PCA surface normal estimation
│   ├── odometry/                  # State estimation and scan matching
│   │   ├── types.odin             # Scan matching parameters and results
│   │   ├── icp.odin               # Point-to-Plane ICP with Huber loss
│   │   ├── ndt.odin               # 3D Normal Distributions Transform (NDT)
│   │   ├── eskf.odin              # 15-state Error-State Kalman Filter
│   │   ├── eskf_cov.odin          # Joseph-stabilized covariance propagation
│   │   └── keyframe.odin          # Keyframe selection heuristics and graph
│   ├── mapping/                   # Volumetric mapping
│   │   ├── morton.odin            # 64-bit Morton Z-order spatial hashing
│   │   ├── voxel_hash.odin        # Sparse voxel hash table with Robin Hood probing
│   │   ├── tsdf.odin              # Truncated Signed Distance Field integrator
│   │   ├── esdf.odin              # Euclidean Signed Distance Field propagator
│   │   ├── octree.odin            # Hierarchical octree with frustum culling
│   │   └── raycaster.odin         # Amanatides-Woo fast voxel ray traversal
│   ├── tracking/                  # Obstacle clustering and multi-target tracking
│   │   ├── cluster.odin           # 3D Euclidean clustering via spatial hashing
│   │   ├── bbox.odin              # 3D Oriented Bounding Box fitting (PCA)
│   │   ├── kalman3d.odin          # 7D CTRV Extended Kalman Filter
│   │   ├── association.odin       # Hungarian assignment and Mahalanobis gating
│   │   └── tracker.odin           # Multi-target track lifecycle state machine
│   ├── planning/                  # Kinodynamic motion planning
│   │   ├── types.odin             # Kinodynamic limits and trajectory waypoints
│   │   ├── corridor.odin          # Convex safe flight corridor generation
│   │   ├── rrt_star.odin          # Kinodynamic RRT* path planner
│   │   ├── spline.odin            # Closed-form quintic polynomial minimum-jerk
│   │   └── controller.odin        # Pure Pursuit and Stanley path followers
│   ├── io/                        # Serialization and formats
│   │   ├── pcd.odin               # ASCII and Binary PCD point cloud codec
│   │   ├── binary_stream.odin     # Compact zero-copy binary wire protocol
│   │   └── json.odin              # Perception snapshot JSON serializer
│   └── server/                    # Embedded network telemetry
│       └── telemetry.odin         # Non-blocking HTTP and SSE telemetry broadcaster
├── tests/                         # Comprehensive unit and integration test suite
│   ├── test_math.odin             # Lie algebra, SE(3), SIMD, and statistics tests
│   ├── test_memory.odin           # Bump arena, object pool, and ring buffer tests
│   ├── test_sensor.odin           # IMU pre-integration and motion deskewing tests
│   ├── test_filter.odin           # Voxel grid, SOR, ROR, and ground RANSAC tests
│   ├── test_odometry.odin         # ICP, NDT, and ESKF state estimation tests
│   ├── test_mapping.odin          # Morton codec, Robin Hood hash, and TSDF tests
│   ├── test_tracking.odin         # 3D clustering, OBB fitting, and MOT tests
│   ├── test_planning.odin         # RRT*, flight corridors, and quintic spline tests
│   └── test_pipeline.odin         # Full multi-sensor perception pipeline integration test
└── web/                           # Real-time WebGL / Canvas 3D Dashboard
    ├── index.html                 # Full-screen HUD spatial dashboard
    ├── css/
    │   └── style.css              # Dark-mode telemetry styling
    └── js/
        ├── visualizer.js          # 3D point cloud, voxel, and trajectory renderer
        └── ws_client.js           # Server-Sent Events (SSE) telemetry client
```

---

## Quickstart

### Prerequisites
- [Odin Compiler](https://odin-lang.org/) (`dev-2026-11` or later)
- LLVM / Clang (automatically installed with Odin)
- Python 3.8+ (for Python SDK bindings)

### 1. Build from Source
```bash
# Clone the repository
git clone https://github.com/Aneesh495/pelorus.git
cd pelorus

# Compile CLI, benchmarks, and C ABI library
./scripts/build.sh
```

### 2. Run Test Suite
```bash
./scripts/test.sh
```
Executes all 39 unit and end-to-end integration tests across mathematics, memory subsystems, state estimation, mapping, tracking, and kinodynamic planning.

### 3. Run Performance Benchmarks
```bash
./scripts/bench.sh
```

### 4. Launch Live Perception Simulation & WebGL Visualizer
```bash
# Start live perception engine on port 8080
./bin/pelorus live --port 8080
```
Open `http://127.0.0.1:8080` in any web browser to view the real-time 3D dashboard displaying live point clouds, vehicle trajectory, 3D tracked obstacle bounding boxes, and planned spline ribbons.

### 5. Python SDK & Trajectory Evaluation
```bash
# Install local editable Python package
pip install -e python/

# Run synthetic trajectory benchmark with ATE evaluation
python3 -c "
import pelorus
engine = pelorus.PelorusEngine(voxel_size=0.1)
dataset = pelorus.SyntheticDataset(duration_sec=5.0)
data = dataset.generate_circular_loop(radius=8.0, speed=2.5)

for i, scan in enumerate(data['scans']):
    t = data['scan_timestamps'][i]
    pos, quat = engine.process_point_cloud(scan, t)

metrics = pelorus.evaluate_ate(data['gt_positions'], data['gt_positions'])
print(f'Evaluation Complete. ATE RMSE: {metrics.rmse:.4f} m')
"
```

---

## Mathematical Derivations

Detailed proofs and mathematical formulations are provided in [docs/math.md](docs/math.md):
- Exact Baker-Campbell-Hausdorff (BCH) formula approximations on $\text{SO}(3)$ and $\text{SE}(3)$.
- Continuous-time error propagation Jacobians $\mathbf{F}_c$ and $\mathbf{G}_c$ for IMU bias dynamics.
- Iteratively Reweighted Least Squares (IRLS) formulation of Point-to-Plane ICP with Huber loss.
- Wavefront distance transform formulation for Euclidean Signed Distance Fields (ESDF).
- Analytical Euler-Lagrange optimality condition for quintic minimum-jerk boundary value problems.

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
