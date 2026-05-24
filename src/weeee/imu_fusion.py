import math
import numpy as np
from scipy.spatial.transform import Rotation as R

from typing import Optional

ALPHA = 0.95
ACC_CORRECTION_THRESHOLD = 0.05
MAX_DT = 0.1
ACCEL_LP_ALPHA = 0.5
GYRO_STILL_THRESHOLD = 0.2
GYRO_DEADBAND_DPS = 0.5


def accel_to_rotation(ax: np.float64, ay: np.float64, az: np.float64) -> R:
    """
    Creates a rotation that aligns the measured acceleration vector (body frame)
    with the world gravity vector [0, 0, 1].
    This avoids Euler singularities during initialization.
    """
    mag = math.sqrt(ax**2 + ay**2 + az**2)
    if mag < 1e-6:
        return R.identity()

    # Normalized body acceleration (where the device thinks gravity is)
    g_body = np.array([ax, ay, az]) / mag
    # Target gravity in world space
    g_world = np.array([0.0, 0.0, 1.0])

    # Find rotation from g_body to g_world
    # Axis = g_body x g_world takes g_body to g_world
    axis = np.cross(g_body, g_world)
    axis_len = np.linalg.norm(axis)

    if axis_len < 1e-6:
        # Already aligned or anti-aligned
        if g_body[2] > 0:
            return R.identity()
        else:
            # 180 degree flip around X (or any horizontal axis)
            return R.from_euler("x", math.pi)

    # The dot product gives cos(theta)
    angle = math.acos(np.clip(np.dot(g_body, g_world), -1.0, 1.0))
    # Return rotation that takes body to world
    return R.from_rotvec(axis / axis_len * angle)


def decode_gyro(val: int, bias: float, slow: bool = False) -> float:
    v = val - bias
    scale = 0.05 if slow else 0.227
    return math.radians(v * scale)


def is_plausible_gyro(v: int) -> bool:
    return 0 < v < 0x3FFF


class ImuFusion:
    def __init__(self) -> None:
        self.orient: R = R.identity()
        self._orient_prev = R.identity()
        self.gyro_bias = {"yaw": 8192.0, "roll": 8192.0, "pitch": 8192.0}
        self.gyro_signs = {"yaw": 1.0, "roll": 1.0, "pitch": 1.0}
        self._accel_lp = np.array([0.0, 0.0, 0.0])
        self._first_frame = True

    def calibrate_gyro(self, samples: list[dict[str, int]]) -> dict[str, float]:
        bias = {
            k: sum(s[k] for s in samples) / len(samples)
            for k in ("yaw", "roll", "pitch")
        }
        self.gyro_bias = bias
        return bias

    def update(
        self,
        ax: float,
        ay: float,
        az: float,
        gyro: Optional[dict[str, int]] = None,
        dt: float = 0.02,
        gyro_slow: Optional[dict[str, bool]] = None,
    ) -> tuple[float, float, float]:
        """
        Updates orientation using accelerometer and optional gyroscope data.
        Wiimote axes: X-right, Y-forward, Z-up.
        Fusion axes: X-forward (Wiimote Y), Y-left (Wiimote -X), Z-up (Wiimote Z).
        """
        if gyro_slow is None:
            gyro_slow = {"roll": False, "pitch": False, "yaw": False}

        dt = min(dt, MAX_DT)

        # Map Wiimote axes to Fusion axes
        # Fusion X (Forward) = Wiimote Y
        # Fusion Y (Left) = -Wiimote X
        # Fusion Z (Up) = Wiimote Z
        accel_raw = np.array([ay, -ax, az])

        if self._first_frame:
            # Initialize orientation to align gravity perfectly
            self._accel_lp = accel_raw
            self.orient = accel_to_rotation(accel_raw[0], accel_raw[1], accel_raw[2])
            self._first_frame = False

        self._accel_lp = (
            ACCEL_LP_ALPHA * accel_raw + (1.0 - ACCEL_LP_ALPHA) * self._accel_lp
        )
        lax, lay, laz = self._accel_lp

        gr = gp = gy = 0.0
        acc_mag = 0.0

        if gyro is not None:
            plausible = all(
                is_plausible_gyro(gyro[k]) for k in ("roll", "pitch", "yaw")
            )
            acc_mag = math.sqrt(lax**2 + lay**2 + laz**2)

            if plausible:
                # Map Wiimote gyros to Fusion axes
                # Fusion GX (Roll) = Wiimote Roll (rotation around Y)
                # Fusion GY (Pitch) = -Wiimote Pitch (rotation around X)
                # Fusion GZ (Yaw) = Wiimote Yaw (rotation around Z)
                gr = (
                    decode_gyro(
                        gyro["roll"],
                        self.gyro_bias["roll"],
                        gyro_slow.get("roll", False),
                    )
                    * self.gyro_signs["roll"]
                )
                gp = (
                    -decode_gyro(
                        gyro["pitch"],
                        self.gyro_bias["pitch"],
                        gyro_slow.get("pitch", False),
                    )
                    * self.gyro_signs["pitch"]
                )
                gy = (
                    decode_gyro(
                        gyro["yaw"], self.gyro_bias["yaw"], gyro_slow.get("yaw", False)
                    )
                    * self.gyro_signs["yaw"]
                )

                if abs(math.degrees(gr)) < GYRO_DEADBAND_DPS:
                    gr = 0.0
                if abs(math.degrees(gp)) < GYRO_DEADBAND_DPS:
                    gp = 0.0
                if abs(math.degrees(gy)) < GYRO_DEADBAND_DPS:
                    gy = 0.0

                omega_body = np.array([gr, gp, gy])
                rot_delta = R.from_rotvec(omega_body * dt)
                self.orient = self.orient * rot_delta

                gyro_rate_mag = np.linalg.norm(omega_body)
                if (
                    gyro_rate_mag < GYRO_STILL_THRESHOLD
                    and abs(acc_mag - 1.0) < ACC_CORRECTION_THRESHOLD
                ):
                    q = self.orient.as_quat()
                    yaw = math.atan2(
                        2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)
                    )
                    self.orient = (
                        R.from_rotvec(np.array([0.0, 0.0, -yaw * 6e-5])) * self.orient
                    )
                    for k in ("roll", "pitch", "yaw"):
                        self.gyro_bias[k] += (gyro[k] - self.gyro_bias[k]) * 0.005

            if abs(acc_mag - 1.0) < ACC_CORRECTION_THRESHOLD and acc_mag > 0.01:
                g_meas = np.array([lax, lay, laz]) / acc_mag
                g_est_body = self.orient.inv().apply([0.0, 0.0, 1.0])
                error = np.cross(g_meas, g_est_body)
                error_norm = np.linalg.norm(error)
                if error_norm > 1e-6:
                    correction = R.from_rotvec(error * (1.0 - ALPHA))
                    self.orient = self.orient * correction
        else:
            # If no gyro, snap to accelerometer orientation but keep current yaw
            current_yaw = self.yaw
            new_orient = accel_to_rotation(lax, lay, laz)

            self.orient = R.from_euler("z", current_yaw) * new_orient
            acc_mag = math.sqrt(lax**2 + lay**2 + laz**2)

        if np.any(np.isnan(self.orient.as_quat())):
            self.orient = self._orient_prev
        else:
            self._orient_prev = self.orient

        return self.yaw, self.pitch, self.roll

    def reset_yaw(self) -> None:
        yaw = self.yaw
        yaw_reset = R.from_euler("z", -yaw)
        self.orient = yaw_reset * self.orient

    @property
    def yaw(self) -> np.float64:
        x: np.float64 = self.orient.as_euler("zyx")[0]
        return x

    @property
    def pitch(self) -> np.float64:
        x: np.float64 = self.orient.as_euler("zyx")[1]
        return x

    @property
    def roll(self) -> np.float64:
        x: np.float64 = self.orient.as_euler("zyx")[2]
        return x

    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.yaw)

    @property
    def pitch_deg(self) -> float:
        return math.degrees(self.pitch)

    @property
    def roll_deg(self) -> float:
        return math.degrees(self.roll)
