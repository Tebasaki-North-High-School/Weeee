"""
Unit tests for the IMU fusion logic.
"""

import pytest
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from weeee.imu_fusion import ImuFusion, accel_to_rotation, decode_gyro


def test_accel_to_rotation_identity() -> None:
    """Tests that a vertical acceleration vector results in identity rotation."""
    rot = accel_to_rotation(np.float64(0), np.float64(0), np.float64(1))
    assert np.allclose(rot.apply([0, 0, 1]), [0, 0, 1])
    assert np.allclose(rot.as_quat(), [0, 0, 0, 1])


def test_accel_to_rotation_tilted() -> None:
    """Tests that a horizontal acceleration vector results in expected rotation."""
    rot = accel_to_rotation(np.float64(1), np.float64(0), np.float64(0))
    result = rot.apply([1, 0, 0])
    assert np.allclose(result, [0, 0, 1])


def test_decode_gyro() -> None:
    """Tests decoding of raw gyroscope values to radians per second."""
    rad = decode_gyro(8212, 8192, slow=True)
    assert rad == pytest.approx(math.radians(1.0))

    rad = decode_gyro(8292, 8192, slow=False)
    assert rad == pytest.approx(math.radians(22.7))


class TestImuFusion:
    """Test suite for the ImuFusion class."""

    def test_initialization_from_accel(self) -> None:
        """Tests that orientation initializes correctly from the first accelerometer reading."""
        imu = ImuFusion()
        imu.update(0, 0, 1)
        assert float(imu.pitch) == pytest.approx(0.0)
        assert float(imu.roll) == pytest.approx(0.0)

        assert float(imu.yaw) == pytest.approx(0.0)

    def test_initialization_tilted(self) -> None:
        """Tests that orientation initializes correctly from a tilted accelerometer reading."""
        imu = ImuFusion()

        angle = math.radians(45)
        imu.update(0, math.sin(angle), math.cos(angle))

        assert float(imu.pitch) == pytest.approx(-angle)
        assert float(imu.roll) == pytest.approx(0.0)

    def test_gyro_integration_pitch(self) -> None:
        """Tests integration of pitch gyroscope data."""
        imu = ImuFusion()

        imu.update(0, 0, 1)

        val = 8192 + int(45 / 0.227)

        gyro = {"roll": 8192, "pitch": val, "yaw": 8192}

        dt = 0.1
        imu.update(0, 0, 1, gyro=gyro, dt=dt)

        assert float(imu.pitch) == pytest.approx(-0.0745273, abs=1e-6)

    def test_gyro_integration_roll(self) -> None:
        """Tests integration of roll gyroscope data."""
        imu = ImuFusion()
        imu.update(0, 0, 1)

        val = 8192 + int(30 / 0.227)
        gyro = {"roll": val, "pitch": 8192, "yaw": 8192}

        dt = 0.05
        imu.update(0, 0, 1, gyro=gyro, dt=dt)

        assert float(imu.roll) == pytest.approx(0.0248412, abs=1e-6)

    def test_gyro_integration_yaw(self) -> None:
        """Tests integration of yaw gyroscope data."""
        imu = ImuFusion()
        imu.update(0, 0, 1)

        val = 8192 + int(60 / 0.227)
        actual_rate = (val - 8192) * 0.227
        gyro = {"roll": 8192, "pitch": 8192, "yaw": val}

        dt = 0.1
        imu.update(0, 0, 1, gyro=gyro, dt=dt)

        assert float(imu.yaw) == pytest.approx(math.radians(actual_rate * dt))

    def test_reset_yaw(self) -> None:
        """Tests resetting the yaw component of the orientation."""
        imu = ImuFusion()
        imu.update(0, 0, 1)
        # Induce some yaw
        imu.update(0, 0, 1, gyro={"roll": 8192, "pitch": 8192, "yaw": 8500}, dt=1.0)
        assert abs(imu.yaw) > 0.01

        imu.reset_yaw()
        assert float(imu.yaw) == pytest.approx(0.0)

    def test_accel_correction(self) -> None:
        """Test that accelerometer correction slowly pulls orientation back towards gravity."""
        imu = ImuFusion()
        # Start flat
        imu.update(0, 0, 1)

        imu.orient = R.from_euler("y", math.radians(10))  # type: ignore

        initial_pitch = imu.pitch
        for _ in range(10):
            imu.update(
                0, 0, 1, gyro={"roll": 8192, "pitch": 8192, "yaw": 8192}, dt=0.02
            )

        assert abs(imu.pitch) < abs(initial_pitch)

    def test_deadband(self) -> None:
        """Tests that gyroscope data within the deadband is ignored."""
        imu = ImuFusion()
        imu.update(0, 0, 1)

        val = 8192 + int(0.4 / 0.227)
        gyro = {"roll": 8192, "pitch": val, "yaw": 8192}

        imu.update(0, 0, 1, gyro=gyro, dt=1.0)
        assert float(imu.pitch) == pytest.approx(0.0)

    def test_gyro_bias_calibration(self) -> None:
        """Tests the gyroscope bias calibration logic."""
        imu = ImuFusion()
        samples = [
            {"roll": 8200, "pitch": 8190, "yaw": 8195},
            {"roll": 8202, "pitch": 8192, "yaw": 8197},
        ]
        bias = imu.calibrate_gyro(samples)
        assert bias["roll"] == 8201
        assert bias["pitch"] == 8191
        assert bias["yaw"] == 8196
        assert imu.gyro_bias == bias

    def test_set_calibration_sets_bias_and_scale(self) -> None:
        """set_calibration() must derive bias, signs, and scale from factory cal."""
        imu = ImuFusion()
        fast = {
            "yaw_zero": 0x1F40,
            "roll_zero": 0x1F40,
            "pitch_zero": 0x1F40,
            "yaw_scale": 0x0CE4,
            "roll_scale": 0x0CE4,
            "pitch_scale": 0x0CE4,
            "degrees_div_6": 200,
        }
        slow = {
            "yaw_zero": 0x2000,
            "roll_zero": 0x2000,
            "pitch_zero": 0x2000,
            "yaw_scale": 0x0CE4,
            "roll_scale": 0x0CE4,
            "pitch_scale": 0x0CE4,
            "degrees_div_6": 45,
        }
        imu.set_calibration(fast, slow)

        # Bias = 14-bit zero (zero_raw >> 2)
        for ax in ("yaw", "roll", "pitch"):
            assert imu.gyro_bias[ax] == 0x1F40 >> 2  # = 2000.0
            # scale > zero → sign should be -1.0
            assert imu.gyro_signs[ax] == -1.0

        # gyro_scale should be populated
        assert imu.gyro_scale is not None
        for ax in ("yaw", "roll", "pitch"):
            # scale in rad/raw-unit
            assert imu.gyro_scale[ax] < 0.0  # negative because sign is -1

    def test_set_calibration_uses_cal_scale_when_available(self) -> None:
        """With calibration set, update() must use gyro_scale instead of decode_gyro."""
        imu = ImuFusion()
        fast = {
            "yaw_zero": 0x1F40,
            "roll_zero": 0x1F40,
            "pitch_zero": 0x1F40,
            "yaw_scale": 0x0CE4,
            "roll_scale": 0x0CE4,
            "pitch_scale": 0x0CE4,
            "degrees_div_6": 200,
        }
        imu.set_calibration(fast)
        imu.update(0, 0, 1)

        # Apply known gyro rate
        # With bias at 0x1F40 and scale negative, a raw value > bias gives negative rate
        gyro = {"yaw": 0x2000, "roll": 0x1F40, "pitch": 0x1F40}
        imu.update(0, 0, 1, gyro=gyro, dt=0.1)

        # Non-zero yaw should have been integrated
        assert imu.yaw != 0.0

    def test_slow_calibration_scale_is_smaller(self) -> None:
        """slow-mode gyro_scale must have smaller magnitude than fast."""
        imu = ImuFusion()
        fast = {
            "yaw_zero": 0x1F40, "roll_zero": 0x1F40, "pitch_zero": 0x1F40,
            "yaw_scale": 0x0CE4, "roll_scale": 0x0CE4, "pitch_scale": 0x0CE4,
            "degrees_div_6": 200,
        }
        slow = {
            "yaw_zero": 0x2000, "roll_zero": 0x2000, "pitch_zero": 0x2000,
            "yaw_scale": 0x0CE4, "roll_scale": 0x0CE4, "pitch_scale": 0x0CE4,
            "degrees_div_6": 45,
        }
        imu.set_calibration(fast, slow)
        assert imu.gyro_scale_slow is not None
        for ax in ("yaw", "roll", "pitch"):
            assert abs(imu.gyro_scale_slow[ax]) < abs(imu.gyro_scale[ax])

    def test_slow_mode_flag_selects_scale(self) -> None:
        """gyro_slow flag must select slow scale per-axis."""
        imu = ImuFusion()
        fast = {
            "yaw_zero": 0x1F40, "roll_zero": 0x1F40, "pitch_zero": 0x1F40,
            "yaw_scale": 0x0CE4, "roll_scale": 0x0CE4, "pitch_scale": 0x0CE4,
            "degrees_div_6": 200,
        }
        slow = {
            "yaw_zero": 0x2000, "roll_zero": 0x2000, "pitch_zero": 0x2000,
            "yaw_scale": 0x0CE4, "roll_scale": 0x0CE4, "pitch_scale": 0x0CE4,
            "degrees_div_6": 45,
        }
        imu.set_calibration(fast, slow)
        imu.update(0, 0, 1)

        gyro = {"yaw": 0x2000, "roll": 0x2000, "pitch": 0x2000}
        # yaw in slow, roll/pitch in fast
        gyro_slow = {"yaw": True, "roll": False, "pitch": False}
        imu.update(0, 0, 1, gyro=gyro, dt=1.0, gyro_slow=gyro_slow)

        assert imu.yaw != 0.0
        assert imu.roll != 0.0
        # yaw uses slow scale → integration rate should be smaller
        # (same raw delta, smaller deg6 → smaller rad/s → smaller angle)
        # Fast scale for roll gives same magnitude as yaw_slow is different
        # Actually with bias 0x1F40 (2000), raw 0x2000 (8192), delta = 6192
        # Fast: (200*6)/(825-2000) = 1200/-1175 = -1.0213 deg/unit
        # Slow: (45*6)/(825-2000) = 270/-1175 = -0.2298 deg/unit
        # So slow path should produce smaller |yaw| vs |roll|
        assert abs(imu.yaw) < abs(imu.roll)
