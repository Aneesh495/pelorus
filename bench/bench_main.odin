package pelorus_bench

import "core:fmt"
import "core:math"
import "core:time"
import core "../src/core"
import filter "../src/filter"
import mapping "../src/mapping"
import odometry "../src/odometry"
import planning "../src/planning"
import sensor "../src/sensor"
import tracking "../src/tracking"

main :: proc() {
	fmt.println("==================================================================")
	fmt.println("  PELORUS: High-Performance Perception & Planning Benchmarks")
	fmt.println("==================================================================\n")

	// 1. SE(3) Lie Algebra Geodesic Operations
	{
		t0 := time.tick_now()
		iters := 200000
		p := core.Vec3{0.1, -0.05, 0.02}
		w := core.Vec3{0.02, 0.01, -0.03}
		xi := core.twist6_make(p, w)
		accum := core.transform3d_identity()

		for _ in 0 ..< iters {
			tf := core.se3_exp(xi)
			accum = core.transform3d_mul(accum, tf)
		}

		dt_ms := time.duration_milliseconds(time.tick_since(t0))
		fmt.printfln("  [SE3 Exp/Multiply]        %8.2f ns/op   (%d iters in %.2f ms)", (dt_ms * 1000000.0) / f64(iters), iters, dt_ms)
	}

	// 2. Morton 64-bit Encoding & Decoding
	{
		t0 := time.tick_now()
		iters := 500000
		dummy: u64 = 0

		for i in 0 ..< iters {
			code := mapping.morton_encode_3d(i32(i & 0x1FFFFF), i32((i >> 5) & 0x1FFFFF), i32((i >> 10) & 0x1FFFFF))
			x, y, z := mapping.morton_decode_3d(code)
			dummy += u64(x + y + z)
		}

		dt_ms := time.duration_milliseconds(time.tick_since(t0))
		fmt.printfln("  [Morton 64-bit Codec]     %8.2f ns/op   (%d iters in %.2f ms)", (dt_ms * 1000000.0) / f64(iters), iters, dt_ms)
	}

	// 3. Voxel Hash Table Robin Hood Lookup
	{
		table: mapping.VoxelHashTable
		mapping.voxel_hash_init(&table, 0.05, 32768)
		defer mapping.voxel_hash_destroy(&table)

		// Pre-populate with 10,000 blocks
		for i in 0 ..< 10000 {
			_ = mapping.voxel_hash_get_or_create_block(&table, i32(i % 100), i32((i / 100) % 100), i32(i % 10))
		}

		t0 := time.tick_now()
		iters := 200000
		found_count := 0

		for i in 0 ..< iters {
			b := mapping.voxel_hash_find_block(&table, i32(i % 100), i32((i / 100) % 100), i32(i % 10))
			if b != nil do found_count += 1
		}

		dt_ms := time.duration_milliseconds(time.tick_since(t0))
		fmt.printfln("  [Voxel Hash Find Block]   %8.2f ns/op   (%d iters in %.2f ms)", (dt_ms * 1000000.0) / f64(iters), iters, dt_ms)
	}

	// 4. Point-to-Plane ICP Registration
	{
		params := odometry.registration_params_default()
		params.max_iterations = 20

		solver: odometry.IcpSolver
		odometry.icp_solver_init(&solver, params, 1000)
		defer odometry.icp_solver_destroy(&solver)

		target: sensor.PointCloud
		source: sensor.PointCloud
		sensor.point_cloud_init(&target, 500)
		sensor.point_cloud_init(&source, 500)
		defer sensor.point_cloud_destroy(&target)
		defer sensor.point_cloud_destroy(&source)

		// Populate 300 points
		for x in -5 ..= 5 {
			for y in -5 ..= 5 {
				pt := core.Vec3{f32(x) * 0.4, f32(y) * 0.4, 0.0}
				sensor.point_cloud_push(&target, pt, 100, 0, 0, core.Vec3{0, 0, 1})
				sensor.point_cloud_push(&source, pt + core.Vec3{0.05, -0.03, 0.01}, 100, 0, 0, core.Vec3{0, 0, 1})
			}
		}

		t0 := time.tick_now()
		align_iters := 100
		for _ in 0 ..< align_iters {
			_ = odometry.icp_align(&solver, &source, &target)
		}
		dt_ms := time.duration_milliseconds(time.tick_since(t0))
		fmt.printfln("  [Point-to-Plane ICP]      %8.2f ms/iter (300 pts, %d runs in %.2f ms)", dt_ms / f64(align_iters), align_iters, dt_ms)
	}

	// 5. TSDF Ray Integration
	{
		table: mapping.VoxelHashTable
		mapping.voxel_hash_init(&table, 0.1, 16384)
		defer mapping.voxel_hash_destroy(&table)

		tsdf_params := mapping.tsdf_params_default(0.1)
		integrator: mapping.TsdfIntegrator
		mapping.tsdf_integrator_init(&integrator, &table, tsdf_params)

		cloud: sensor.PointCloud
		sensor.point_cloud_init(&cloud, 1000)
		defer sensor.point_cloud_destroy(&cloud)

		for i in 0 ..< 1000 {
			angle := f32(i) * 0.01
			dist := 5.0 + f32(i % 10) * 0.2
			sensor.point_cloud_push(&cloud, core.Vec3{math.cos(angle) * dist, math.sin(angle) * dist, 0.5})
		}

		t0 := time.tick_now()
		runs := 50
		for _ in 0 ..< runs {
			mapping.tsdf_integrate_point_cloud(&integrator, &cloud, core.Vec3{0, 0, 0})
		}
		dt_ms := time.duration_milliseconds(time.tick_since(t0))
		pts_per_sec := (f64(runs * 1000) / dt_ms) * 1000.0
		fmt.printfln("  [TSDF Ray Integration]    %8.0f pts/sec (1k pts x %d runs in %.2f ms)", pts_per_sec, runs, dt_ms)
	}

	// 6. Minimum-Jerk Spline Generation & Sampling
	{
		waypoints := []core.Vec3{
			{0.0, 0.0, 0.0},
			{2.0, 1.0, 0.5},
			{5.0, 3.0, 1.0},
			{8.0, 2.0, 1.2},
			{10.0, 5.0, 1.5},
		}

		traj: planning.QuinticTrajectory3D
		planning.quintic_trajectory_init(&traj)
		defer planning.quintic_trajectory_destroy(&traj)

		t0 := time.tick_now()
		traj_runs := 10000
		for _ in 0 ..< traj_runs {
			_ = planning.generate_minimum_jerk_trajectory(
				waypoints,
				core.Vec3{0, 0, 0},
				core.Vec3{0, 0, 0},
				core.Vec3{0, 0, 0},
				core.Vec3{0, 0, 0},
				3.0,
				&traj,
			)
		}
		dt_ms := time.duration_milliseconds(time.tick_since(t0))
		fmt.printfln("  [Quintic Spline Solve]    %8.2f us/solve (5 knots, %d runs in %.2f ms)", (dt_ms * 1000.0) / f64(traj_runs), traj_runs, dt_ms)
	}

	fmt.println("\n==================================================================")
	fmt.println("  Benchmark run finished successfully.")
	fmt.println("==================================================================")
}
