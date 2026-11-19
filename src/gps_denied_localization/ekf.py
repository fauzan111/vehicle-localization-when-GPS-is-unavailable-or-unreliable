"""Planar EKF using wheel speed, gyro, relative camera motion, and GNSS."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def wrap_angle(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


@dataclass
class PlanarLocalizationEKF:
    """EKF state [x, y, yaw, gyro bias, wheel-speed bias]."""

    camera_translation_std_m: float
    camera_heading_std_rad: float
    gnss_position_std_m: float
    wheel_speed_std_mps: float
    gyro_rate_std_rps: float
    gyro_bias_walk_std_rps_sqrt_s: float = 0.0002
    speed_bias_walk_std_mps_sqrt_s: float = 0.002

    def __post_init__(self) -> None:
        self.state = np.zeros(5, dtype=float)
        self.covariance = np.diag([1.0, 1.0, 0.1**2, 0.01**2, 0.1**2])
        self._joint_state = np.zeros(10, dtype=float)
        self._joint_covariance = np.zeros((10, 10), dtype=float)

    def predict(self, wheel_speed_mps: float, gyro_z_rps: float, dt: float) -> None:
        """Propagate pose and retain previous/current cross-covariance."""
        if dt <= 0:
            raise ValueError("dt must be positive")
        previous = self.state.copy()
        previous_covariance = self.covariance.copy()
        x, y, yaw, gyro_bias, speed_bias = previous
        speed = wheel_speed_mps - speed_bias
        yaw_rate = gyro_z_rps - gyro_bias
        c, s = np.cos(yaw), np.sin(yaw)

        current = np.array(
            [
                x + speed * c * dt,
                y + speed * s * dt,
                wrap_angle(yaw + yaw_rate * dt),
                gyro_bias,
                speed_bias,
            ]
        )
        transition = np.eye(5)
        transition[0, 2] = -speed * s * dt
        transition[0, 4] = -c * dt
        transition[1, 2] = speed * c * dt
        transition[1, 4] = -s * dt
        transition[2, 3] = -dt
        control_jacobian = np.zeros((5, 2), dtype=float)
        control_jacobian[0, 0] = c * dt
        control_jacobian[1, 0] = s * dt
        control_jacobian[2, 1] = dt
        control_noise = np.diag(
            [self.wheel_speed_std_mps**2, self.gyro_rate_std_rps**2]
        )
        process_noise = np.diag(
            [
                0.0,
                0.0,
                0.0,
                self.gyro_bias_walk_std_rps_sqrt_s**2 * dt,
                self.speed_bias_walk_std_mps_sqrt_s**2 * dt,
            ]
        )
        process_noise += control_jacobian @ control_noise @ control_jacobian.T

        joint_transition = np.vstack((transition, np.eye(5)))
        joint_covariance = joint_transition @ previous_covariance @ joint_transition.T
        joint_covariance[:5, :5] += process_noise
        self._joint_state = np.concatenate((current, previous))
        self._joint_covariance = joint_covariance
        self.state = current
        self.covariance = joint_covariance[:5, :5].copy()

    def update_camera(self, dx_body_m: float, dy_body_m: float, dheading_rad: float) -> None:
        """Update from relative pose, retaining its correlation to the previous state."""
        x, y, yaw, _, _, x_previous, y_previous, yaw_previous, _, _ = self._joint_state
        dx_world = x - x_previous
        dy_world = y - y_previous
        c, s = np.cos(yaw_previous), np.sin(yaw_previous)
        predicted = np.array(
            [
                c * dx_world + s * dy_world,
                -s * dx_world + c * dy_world,
                wrap_angle(yaw - yaw_previous),
            ]
        )
        innovation = np.array([dx_body_m, dy_body_m, dheading_rad]) - predicted
        innovation[2] = wrap_angle(innovation[2])

        observation = np.zeros((3, 10), dtype=float)
        observation[0, [0, 1, 5, 6, 7]] = [c, s, -c, -s, -predicted[1]]
        observation[1, [0, 1, 5, 6, 7]] = [-s, c, s, -c, -predicted[0]]
        observation[2, [2, 7]] = [1.0, -1.0]
        noise = np.diag(
            [
                self.camera_translation_std_m**2,
                self.camera_translation_std_m**2,
                self.camera_heading_std_rad**2,
            ]
        )
        self._joint_update(innovation, observation, noise)
        self._collapse_joint_state()

    def update_gnss(self, x_m: float, y_m: float) -> None:
        """Correct current global position with an available GNSS fix."""
        observation = np.zeros((2, 10), dtype=float)
        observation[0, 0] = 1.0
        observation[1, 1] = 1.0
        innovation = np.array([x_m, y_m]) - observation @ self._joint_state
        noise = np.eye(2) * self.gnss_position_std_m**2
        self._joint_update(innovation, observation, noise)
        self._collapse_joint_state()

    def _joint_update(
        self, innovation: np.ndarray, observation: np.ndarray, noise: np.ndarray
    ) -> None:
        innovation_covariance = (
            observation @ self._joint_covariance @ observation.T.copy() + noise
        )
        gain = np.linalg.solve(
            innovation_covariance, observation @ self._joint_covariance
        ).T
        self._joint_state += gain @ innovation
        self._joint_state[2] = wrap_angle(self._joint_state[2])
        self._joint_state[7] = wrap_angle(self._joint_state[7])
        residual_map = np.eye(10) - gain @ observation
        self._joint_covariance = (
            residual_map @ self._joint_covariance @ residual_map.T.copy()
            + gain @ noise @ gain.T.copy()
        )

    def _collapse_joint_state(self) -> None:
        self.state = self._joint_state[:5].copy()
        self.covariance = self._joint_covariance[:5, :5].copy()
