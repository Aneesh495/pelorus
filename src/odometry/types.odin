package pelorus_odometry

import core "../core"
import sensor "../sensor"

//
// Scan matching algorithm selector.
//
RegistrationMethod :: enum u8 {
	PointToPointICP,
	PointToPlaneICP,
	NDT3D,
}

//
// Scan registration convergence parameters and robust loss settings.
//
RegistrationParams :: struct {
	max_iterations:             int, // Maximum optimization iterations (e.g. 30)
	max_correspondence_dist:    f32, // Maximum spatial pairing distance (meters)
	transformation_epsilon:     f32, // Minimum SE(3) step norm for convergence
	euclidean_fitness_epsilon:  f32, // Minimum change in residual error for convergence
	huber_delta:                f32, // Huber robust loss scale threshold (meters)
	outlier_rejection_ratio:    f32, // Fraction of worst residuals to reject
}

//
// Detailed registration outcome and diagnostics.
//
RegistrationResult :: struct {
	transform:          core.Transform3D,
	iterations:         int,
	fitness_score:      f32, // Mean residual error across inliers (meters)
	inlier_count:       int,
	inlier_ratio:       f32,
	converged:          bool,
	is_degenerate:      bool, // True if Fisher information matrix reveals unconstrained direction
	information_matrix: core.Mat6x6, // Approximate Hessian / Information matrix at solution
}

//
// Default registration parameters.
//
registration_params_default :: proc "contextless" () -> RegistrationParams {
	return RegistrationParams{
		max_iterations            = 30,
		max_correspondence_dist   = 1.5,
		transformation_epsilon    = 1e-4,
		euclidean_fitness_epsilon = 1e-5,
		huber_delta               = 0.1,
		outlier_rejection_ratio   = 0.1,
	}
}
