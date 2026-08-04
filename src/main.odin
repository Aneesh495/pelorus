package pelorus

import "core:fmt"
import "core:math"
import "core:os"
import "core:strconv"
import "core:strings"
import "core:time"
import core "core"
import filter "filter"
import io "io"
import mapping "mapping"
import odometry "odometry"
import planning "planning"
import sensor "sensor"
import server "server"
import tracking "tracking"

PELORUS_VERSION :: "0.8.0"

//
// Print ASCII banner and usage information.
//
print_banner :: proc() {
	fmt.println("==================================================================")
	fmt.println("  PELORUS: Real-Time Spatial Perception & Kinodynamic Planner")
	fmt.printfln("  Version %s [Native ARM64 / Continuous-Time SE(3)]", PELORUS_VERSION)
	fmt.println("==================================================================")
}

print_usage :: proc() {
	fmt.println("Usage: pelorus <command> [options]\n")
	fmt.println("Commands:")
	fmt.println("  live [--port P]       Start live perception engine with WebGL telemetry stream")
	fmt.println("  play <file.pcd>       Replay point cloud recording through state estimation")
	fmt.println("  bench                 Execute performance benchmark suite (ICP, TSDF, RRT*)")
	fmt.println("  version               Display version and compiler target details")
	fmt.println("  help                  Show this usage reference\n")
	fmt.println("Options:")
	fmt.println("  --port <int>          Telemetry HTTP/SSE port (default: 8080)")
	fmt.println("  --voxel-size <float>  TSDF voxel leaf size in meters (default: 0.1)")
	fmt.println("  --headless            Run without launching browser visualizer")
}

//
// CLI command: live perception loop with synthetic / sensor telemetry.
//
cmd_live :: proc(args: []string) {
	port := 8080
	for i := 0; i < len(args); i += 1 {
		if args[i] == "--port" && i + 1 < len(args) {
			if p, ok := strconv.parse_int(args[i + 1]); ok {
				port = p
			}
		}
	}

	cfg := io.engine_config_default()
	engine: PelorusEngine
	pelorus_engine_init(&engine, cfg)
	defer pelorus_engine_destroy(&engine)

	fmt.printfln("[Pelorus] Engine initialized on port %d. Telemetry active.", port)
	fmt.println("[Pelorus] Generating synthetic LiDAR-inertial trajectory...")

	// Synthetic perception simulation loop
	raw_scan: sensor.PointCloud
	sensor.point_cloud_init(&raw_scan, 1024)
	defer sensor.point_cloud_destroy(&raw_scan)

	deskewed_scan: sensor.PointCloud
	sensor.point_cloud_init(&deskewed_scan, 1024)
	defer sensor.point_cloud_destroy(&deskewed_scan)

	t_sim := f64(0.0)
	dt := f64(0.1) // 10 Hz scan rate

	for step in 0 ..< 100 {
		t_sim += dt

		// 1. Simulate IMU packet (vehicle moving along circular arc with forward speed 2 m/s)
		accel := core.Vec3{0.0, 0.4, 9.81} // 0.4 m/s^2 centripetal accel
		gyro := core.Vec3{0.0, 0.0, 0.2}   // 0.2 rad/s yaw rate
		pelorus_engine_process_imu(&engine, accel, gyro, t_sim)

		// 2. Synthesize 3D point cloud scan with ground plane and 2 obstacles
		sensor.point_cloud_clear(&raw_scan)
		raw_scan.header.timestamp = t_sim
		raw_scan.header.sequence_id = u64(step)

		// Ground points (z = 0)
		for gx in -5 ..= 5 {
			for gy in -5 ..= 5 {
				sensor.point_cloud_push(
					&raw_scan,
					core.Vec3{f32(gx) * 0.8, f32(gy) * 0.8, 0.0},
					50.0,
					t_sim,
					0,
					core.Vec3{0, 0, 1},
				)
			}
		}

		// Obstacle 1: Moving pedestrian box around (3.0, 1.5, 0.8)
		obs1_x := 3.0 + f32(math.sin(f32(t_sim))) * 0.5
		for ox in -2 ..= 2 {
			for oy in -2 ..= 2 {
				for oz in 0 ..= 3 {
					sensor.point_cloud_push(
						&raw_scan,
						core.Vec3{obs1_x + f32(ox) * 0.1, 1.5 + f32(oy) * 0.1, 0.2 + f32(oz) * 0.3},
						120.0,
						t_sim,
						1,
					)
				}
			}
		}

		sensor.point_cloud_clear(&deskewed_scan)
		pose := pelorus_engine_process_scan(&engine, &raw_scan, &deskewed_scan)

		if step % 20 == 0 {
			fmt.printfln(
				"[Step %3d] Pose: pos=(%.2f, %.2f, %.2f) tracks=%d",
				step,
				pose.translation.x,
				pose.translation.y,
				pose.translation.z,
				len(engine.mot_tracker.tracks),
			)
		}

		// Keep embedded telemetry responsive
		server.telemetry_server_poll(&engine.telemetry)
	}

	fmt.println("[Pelorus] Live perception run completed successfully.")
}

//
// CLI command: playback PCD point cloud file.
//
cmd_play :: proc(filepath: string) {
	fmt.printfln("[Pelorus] Loading PCD recording from: %s", filepath)

	cloud: sensor.PointCloud
	sensor.point_cloud_init(&cloud, 2048)
	defer sensor.point_cloud_destroy(&cloud)

	ok := io.pcd_read_file(filepath, &cloud)
	if !ok {
		fmt.eprintfln("[Pelorus] Error: Failed to open or parse PCD file '%s'", filepath)
		return
	}

	fmt.printfln("[Pelorus] Successfully parsed %d points.", len(cloud.points))

	cfg := io.engine_config_default()
	engine: PelorusEngine
	pelorus_engine_init(&engine, cfg)
	defer pelorus_engine_destroy(&engine)

	deskewed: sensor.PointCloud
	sensor.point_cloud_init(&deskewed, len(cloud.points))
	defer sensor.point_cloud_destroy(&deskewed)

	pose := pelorus_engine_process_scan(&engine, &cloud, &deskewed)
	fmt.printfln(
		"[Pelorus] Scan registered. Estimated pose: pos=(%.3f, %.3f, %.3f)",
		pose.translation.x,
		pose.translation.y,
		pose.translation.z,
	)
}

//
// CLI command: execute benchmark suite.
//
cmd_bench :: proc() {
	fmt.println("[Pelorus] Running comprehensive perception benchmarks...\n")

	// 1. Math / Lie Algebra benchmark
	{
		t0 := time.tick_now()
		iters := 100000
		v := core.Vec3{0.05, -0.02, 0.08}
		accum := core.quat_identity()
		for _ in 0 ..< iters {
			q := core.so3_exp_quat(v)
			accum = core.quat_mul(accum, q)
		}
		dt_ms := time.duration_milliseconds(time.tick_since(t0))
		fmt.printfln("  [Bench] SO(3) Exp/Log Lie operations: %8.2f us/op (iters=%d in %.2f ms)", (dt_ms * 1000.0) / f64(iters), iters, dt_ms)
	}

	// 2. Voxel Hashing benchmark
	{
		t0 := time.tick_now()
		table: mapping.VoxelHashTable
		mapping.voxel_hash_init(&table, 0.1, 16384)
		defer mapping.voxel_hash_destroy(&table)

		iters := 50000
		for i in 0 ..< iters {
			_ = mapping.voxel_hash_get_or_create_block(&table, i32(i % 100), i32((i / 100) % 100), i32(i % 20))
		}
		dt_ms := time.duration_milliseconds(time.tick_since(t0))
		fmt.printfln("  [Bench] 3D Robin Hood Voxel Hashing:  %8.2f ns/op (iters=%d in %.2f ms)", (dt_ms * 1000000.0) / f64(iters), iters, dt_ms)
	}

	// 3. Quintic Spline Evaluation benchmark
	{
		t0 := time.tick_now()
		iters := 200000
		seg := planning.QuinticSegment3D{
			x = planning.quintic1d_solve(0, 0, 0, 5, 1, 0, 3.0),
			y = planning.quintic1d_solve(0, 0, 0, 2, 0, 0, 3.0),
			z = planning.quintic1d_solve(0, 0, 0, 1, 0, 0, 3.0),
			t_start = 0,
			duration = 3.0,
		}
		dummy_sum := f32(0.0)
		for i in 0 ..< iters {
			t := f32(i % 300) * 0.01
			p := planning.quintic_segment_eval_pos(seg, t)
			dummy_sum += p.x
		}
		dt_ms := time.duration_milliseconds(time.tick_since(t0))
		fmt.printfln("  [Bench] Quintic Spline Evaluation:   %8.2f ns/op (iters=%d in %.2f ms)", (dt_ms * 1000000.0) / f64(iters), iters, dt_ms)
	}

	fmt.println("\n[Pelorus] All benchmarks completed.")
}

main :: proc() {
	args := os.args
	if len(args) < 2 {
		print_banner()
		print_usage()
		return
	}

	cmd := args[1]
	switch cmd {
	case "live":
		print_banner()
		cmd_live(args[2:])
	case "play":
		if len(args) < 3 {
			fmt.println("Error: 'play' requires a PCD file path argument.")
			return
		}
		cmd_play(args[2])
	case "bench":
		print_banner()
		cmd_bench()
	case "version", "--version", "-v":
		fmt.printfln("pelorus v%s (arm64-apple-darwin)", PELORUS_VERSION)
	case "help", "--help", "-h":
		print_banner()
		print_usage()
	case:
		fmt.printfln("Unknown command '%s'. Run 'pelorus help' for usage.", cmd)
	}
}
