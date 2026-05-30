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
        assert imu.pitch == pytest.approx(0.0)
        assert imu.roll == pytest.approx(0.0)

        assert imu.yaw == pytest.approx(0.0)

    def test_initialization_tilted(self) -> None:
        """Tests that orientation initializes correctly from a tilted accelerometer reading."""
        imu = ImuFusion()

        angle = math.radians(45)
        imu.update(0, math.sin(angle), math.cos(angle))

        assert imu.pitch == pytest.approx(-angle)
        assert imu.roll == pytest.approx(0.0)

    def test_gyro_integration_pitch(self) -> None:
        """Tests integration of pitch gyroscope data."""
        imu = ImuFusion()

        imu.update(0, 0, 1)

        val = 8192 + int(45 / 0.227)

        gyro = {"roll": 8192, "pitch": val, "yaw": 8192}

        dt = 0.1
        imu.update(0, 0, 1, gyro=gyro, dt=dt)

        assert imu.pitch == pytest.approx(-0.0745273, abs=1e-6)

    def test_gyro_integration_roll(self) -> None:
        """Tests integration of roll gyroscope data."""
        imu = ImuFusion()
        imu.update(0, 0, 1)

        val = 8192 + int(30 / 0.227)
        gyro = {"roll": val, "pitch": 8192, "yaw": 8192}

        dt = 0.05
        imu.update(0, 0, 1, gyro=gyro, dt=dt)

        assert imu.roll == pytest.approx(0.0248412, abs=1e-6)

    def test_gyro_integration_yaw(self) -> None:
        """Tests integration of yaw gyroscope data."""
        imu = ImuFusion()
        imu.update(0, 0, 1)

        val = 8192 + int(60 / 0.227)
        actual_rate = (val - 8192) * 0.227
        gyro = {"roll": 8192, "pitch": 8192, "yaw": val}

        dt = 0.1
        imu.update(0, 0, 1, gyro=gyro, dt=dt)

        assert imu.yaw == pytest.approx(math.radians(actual_rate * dt))

    def test_reset_yaw(self) -> None:
        """Tests resetting the yaw component of the orientation."""
        imu = ImuFusion()
        imu.update(0, 0, 1)
        # Induce some yaw
        imu.update(0, 0, 1, gyro={"roll": 8192, "pitch": 8192, "yaw": 8500}, dt=1.0)
        assert abs(imu.yaw) > 0.01

        imu.reset_yaw()
        assert imu.yaw == pytest.approx(0.0)

    def test_accel_correction(self) -> None:
        """Test that accelerometer correction slowly pulls orientation back towards gravity."""
        imu = ImuFusion()
        # Start flat
        imu.update(0, 0, 1)

        imu.orient = R.from_euler("y", math.radians(10))

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
        assert imu.pitch == pytest.approx(0.0)

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
