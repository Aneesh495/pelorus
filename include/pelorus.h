/*
 * Pelorus: High-Performance Real-Time Spatial Perception & Kinodynamic Planning Engine
 * Native C ABI Interface Header
 */

#ifndef PELORUS_H
#define PELORUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * Returns the Pelorus engine semantic version string (e.g. "0.8.0").
 */
const char* pelorus_get_version(void);

/**
 * Creates and initializes an instance of the Pelorus perception engine.
 * @param voxel_size TSDF and filter voxel size in meters (e.g. 0.1).
 * @return Opaque pointer handle to engine instance, or NULL on failure.
 */
void* pelorus_engine_create(float voxel_size);

/**
 * Shuts down and releases all resources allocated for the engine instance.
 * @param handle Opaque pointer handle returned by pelorus_engine_create.
 */
void pelorus_engine_destroy(void* handle);

/**
 * Pushes high-frequency IMU linear acceleration and angular velocity measurements.
 * @param handle Opaque pointer handle to engine.
 * @param ax Linear acceleration X (m/s^2).
 * @param ay Linear acceleration Y (m/s^2).
 * @param az Linear acceleration Z (m/s^2).
 * @param gx Angular velocity X (rad/s).
 * @param gy Angular velocity Y (rad/s).
 * @param gz Angular velocity Z (rad/s).
 * @param timestamp IMU measurement timestamp in seconds.
 */
void pelorus_engine_push_imu(
    void* handle,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    double timestamp
);

/**
 * Ingests a raw LiDAR point cloud, executes motion deskewing, scan matching,
 * TSDF integration, ground segmentation, obstacle clustering, and MOT tracking.
 * @param handle Opaque pointer handle to engine.
 * @param points_xyz Contiguous array of interleaved floats: [x0, y0, z0, x1, y1, z1, ...].
 * @param num_points Total number of 3D points in array.
 * @param timestamp Scan capture timestamp in seconds.
 * @param out_pose_7d Optional float buffer of size 7 to receive [x, y, z, qx, qy, qz, qw].
 * @return 0 on success, negative error code on failure.
 */
int32_t pelorus_engine_process_points(
    void* handle,
    const float* points_xyz,
    int32_t num_points,
    double timestamp,
    float* out_pose_7d
);

/**
 * Plans a collision-free kinodynamic minimum-jerk trajectory to the 3D target.
 * @param handle Opaque pointer handle to engine.
 * @param goal_x Target X position in world frame.
 * @param goal_y Target Y position in world frame.
 * @param goal_z Target Z position in world frame.
 * @param out_waypoints_xyz Output buffer to receive interleaved waypoint positions [x, y, z, ...].
 * @param max_waypoints Maximum number of 3D waypoints to write into buffer.
 * @return Number of waypoints generated, or 0 if no feasible collision-free path exists.
 */
int32_t pelorus_engine_plan_trajectory(
    void* handle,
    float goal_x, float goal_y, float goal_z,
    float* out_waypoints_xyz,
    int32_t max_waypoints
);

/**
 * Returns the current number of active tracked dynamic obstacles.
 * @param handle Opaque pointer handle to engine.
 */
int32_t pelorus_engine_get_obstacle_count(void* handle);

#ifdef __cplusplus
}
#endif

#endif /* PELORUS_H */
