#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_config.h>

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Matrix3f;
using matrix::Vector3f;
using matrix::Dcmf;

class PFAAttPlanner : public ModuleParams
{
public:
	struct PlannerMeta {
		float theta;
		Vector3f k;
		Vector3f sat_force;

		uint8_t bisection_count;
		uint8_t is_feasible;
	} _planner_meta;

	PFAAttPlanner(ModuleParams *parent) : ModuleParams(parent) {
		_planner_meta.k(0) = -1.0f;
		_planner_meta.theta = -1.0f;
	};
	~PFAAttPlanner() = default;

	/**
	 * @brief Calculate the desired attitude and angular velocity
	 * @param in_thrust Desired thrust vector
	 * @param in_R      Reference attitude
	 * @param in_omega  Reference angular velocity
	 * @param out_R_d   A feasible desired attitude
	 * @param out_omega_d A feasible desired angular velocity
	 * @return true if the reference attitude is not feasible and a new attitude
	 * 		is calculated.
	*/
	bool plan(const Vector3f & in_thrust,
		const Dcmf & in_R, const Vector3f & in_omega,
		Dcmf & out_R_d, Vector3f & out_omega_d)
	{
		_sat_force = saturate_by_magnitude(in_thrust);
		_planner_meta.sat_force = _sat_force;
		if (calc_boundary_theta_bisection(in_R, _sat_force, _planned_theta, out_R_d)) {
			// The reference attitude is not feasible
			// TODO: calculate the desired angular velocity
			// out_omega_d = (out_R_d.transpose() * in_R).diagonal() * in_omega;
			_planner_meta.theta = _planned_theta;
			_planner_meta.is_feasible = false;
			return true;
		} else {
			out_R_d = in_R;
			out_omega_d = in_omega;
			_planner_meta.theta = 0;
			_planner_meta.is_feasible = true;
			return false;
		}
	}

	void output_wrench_projection(const Vector3f & in_torque, const Vector3f & in_thrust,
		Vector3f & out_torque, Vector3f & out_thrust)
	{
		// All vectors are expressed in the NWU body frame
		// Minimum z-axis thrust
		out_thrust = in_thrust;
		out_thrust(2) = (out_thrust(2) < 0.1f) ? 0.1f : out_thrust(2);

		// Maximum thrust magnitude
		out_thrust = saturate_by_magnitude(out_thrust);

		// Thrust deflection constraints
		const float z = out_thrust(2);
		const float c_x = func_g2(_param.n_x, _param.sigma_b, z, _param.f_max) + func_g2(_param.n_y, _param.sigma_a, z, _param.f_max);
		const float c_y = func_g2(_param.n_x, _param.sigma_a, z, _param.f_max) + func_g2(_param.n_y, _param.sigma_b, z, _param.f_max);
		const float t = (c_x * c_y) / sqrtf(out_thrust(0) * out_thrust(0) * c_y * c_y + out_thrust(1) * out_thrust(1) * c_x * c_x);

		if (t < 1) {
			out_thrust(0) *= t;
			out_thrust(1) *= t;
		}

		// Maximum torque according to the reachibility at the given thrust
		// const Vector3f _sat_torque = saturate_by_magnitude(in_torque);

		out_torque = in_torque;
	}

protected:
	float _planned_theta{0.0f};
	Vector3f _sat_force{};

	struct vehicle_params {
		float sigma_a;
		float sigma_b;
		float f_max;
		int n;
		int n_x;
		int n_y;
	} _param;

	Vector3f saturate_by_magnitude(const Vector3f & in) {
		const float scale = in.norm() / _param.f_max;
		const Vector3f thrust = (scale > 1) ? static_cast<Vector3f>(in / scale) : in;
		return thrust;
	}

	/**
	 * Using the bisection method to calculate the minimum angle between the
	 * boundary of attainable space at the reference attitude and desired force
	 * vector.
	 * @param R_r Reference attitude
	 * @param f_r Desired force vector
	 * @param out_theta Minimum angle between the boundary and the desired force vector
	 * @param out_R The closet attitude to the reference attitude that can
	 * 		satisfy the desired force vector
	 * @return true if the reference attitude is not feasible and a new attitude
	 * 		is calculated.
	 */
	bool calc_boundary_theta_bisection(const Dcmf & R_r, const Vector3f & f_r,
		float & out_theta, Dcmf & out_R)
	{
		const int maximum_iteration = 10;
		const Vector3f b1r = R_r.col(0);
		// const Vector3f b2r = R_r.col(1);
		const Vector3f b3r = R_r.col(2);

		const float theta_max = acosf(f_r.dot(b3r) / f_r.norm());
		float theta_delta = theta_max / 2.0f;
		out_theta = theta_max / 2.0f;
		// printf("theta_max: %f, starting from %f, with delta %f\n", static_cast<double>(theta_max), static_cast<double>(out_theta), static_cast<double>(theta_delta));

		if (!is_attainable(R_r, f_r)) {
			// b3r and f_r may be almost in the opposite direction
			const bool is_b3r_and_f_r_almost_parallel = (abs(b3r.dot(f_r)) / f_r.norm() > 0.99f);
			const Vector3f k = is_b3r_and_f_r_almost_parallel ?
				Vector3f(1.0f, 0.0f, 0.0f) :	// rotate along x-axis
				b3r.cross(f_r).normalized(); 	// rotate along the plane of b3r and f_r

			_planner_meta.k = k;

			for (int i = 0; i < maximum_iteration; i++) {
				out_R.col(2) = rotate_vector_by_theta(b3r, k, out_theta);
				out_R.col(1) = -b1r.cross(out_R.col(2)).normalized();        	// b3 x b1r / ||b3 x b1r||_2
				out_R.col(0) = Vector3f(out_R.col(1)).cross(out_R.col(2));	// b2r x b3r

				// printf("iteration %d, theta: %f\n", i, static_cast<double>(out_theta));
				// printf("b3r:\n");
				// b3r.print();
				// printf("k:\n");
				// k.print();
				// printf("R:\n");
				// out_R.print();

				theta_delta /= 2.0f;
				if (is_attainable(out_R, f_r))
					out_theta = out_theta - theta_delta;
				else
					out_theta = out_theta + theta_delta;
			}
			return true;
		} else {
			out_theta = 0.0f;
			out_R = R_r;
			return false;
		}
	}

	/**
	 * Check if the force is attainable at the reference attitude using elliptic cone
	 * approximation.
	 * @return true if the force is attainable
	 */
	bool is_attainable(const Dcmf & R, const Vector3f & f_r)
	{
		const Vector3f b1 = R.col(0);
		const Vector3f b2 = R.col(1);
		const Vector3f b3 = R.col(2);

		// TODO: use tunable parameters instead of hard-coded values
		const float sigma_a = _param.sigma_a * 0.5f;
		const float sigma_b = _param.sigma_b * 0.5f;

		const float z = f_r.dot(b3);
		const float c_x = func_g2(_param.n_x, sigma_b, z, _param.f_max) + func_g2(_param.n_y, sigma_a, z, _param.f_max);
		const float c_y = func_g2(_param.n_x, sigma_a, z, _param.f_max) + func_g2(_param.n_y, sigma_b, z, _param.f_max);

		const float d = (f_r.dot(b1) / c_x) * (f_r.dot(b1) / c_x) + (f_r.dot(b2) / c_y) * (f_r.dot(b2) / c_y);
		// printf("z: %f, c_x: %f, c_y: %f, d: %f\n", static_cast<double>(z), static_cast<double>(c_x), static_cast<double>(c_y), static_cast<double>(d));
		return (z > 0) && (d <= 1);
	}

	inline float func_g2(const float n_k, const float sigma_k_bar, const float z, const float f_max) const
	{
		if (sigma_k_bar >= M_PI_2_F)
			return n_k * f_max;
		else
			return (int)(n_k > 0) * z * tanf(sigma_k_bar);
	}

private:

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PFA_MAX_THR>)  _param_vehicle_max_thrust,
		(ParamFloat<px4::params::CA_SV_TL0_MAXA>)  _param_servo_x_angle_max,
		(ParamFloat<px4::params::CA_SV_TL1_MAXA>)  _param_servo_y_angle_max,
		(ParamInt<px4::params::CA_MD_COUNT>)  _param_module_count
	)

	void updateParams() override
	{
		ModuleParams::updateParams();

		// retrieving vehicle parameters
		// here we assume that all the modules have the same parameters
		_param.f_max = _param_vehicle_max_thrust.get();
		_param.sigma_a = _param_servo_x_angle_max.get() * M_DEG_TO_RAD_F;
		_param.sigma_b = _param_servo_y_angle_max.get() * M_DEG_TO_RAD_F;

		_param.n = _param_module_count.get();
		_param.n_x = 0;
		_param.n_y = 0;

		char buffer[17];
		for (int i = 0; i < _param.n; ++i) {
			snprintf(buffer, sizeof(buffer), "CA_MD%u_AZ", i);
			int32_t ax_psi;
			param_get(param_find(buffer), &ax_psi);
			if (ax_psi == 0)
				_param.n_x += 1;
			else
				_param.n_y += 1;
		}
		if (_param.n_x + _param.n_y != _param.n)
			PX4_ERR("Module count does not match the number of azimuths");

		PX4_INFO("PFA Attitude Planner parameter loaded.");
		PX4_INFO("f_max: %f, sigma_a: %f, sigma_b: %f, n: %d, n_x: %d, n_y: %d",
			static_cast<double>(_param.f_max), static_cast<double>(_param.sigma_a), static_cast<double>(_param.sigma_b),
			_param.n, _param.n_x, _param.n_y);
	}

	inline Vector3f rotate_vector_by_theta(const Vector3f vec, const Vector3f rot_axis, const float theta) const
	{
		// Rodrigues' rotation formula
		return (vec * cosf(theta) + rot_axis.cross(vec) * sinf(theta) + rot_axis * rot_axis.dot(vec) * (1 - cosf(theta)));
	}
};
