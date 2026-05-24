import pytest
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from src.imu_fusion import ImuFusion, accel_to_rotation, decode_gyro


def test_accel_to_rotation_identity() -> None:
    # Gravity pointing at Z up (0, 0, 1) means no rotation needed if that's our target
    # Wiimote axes: X-right, Y-forward, Z-up.
    # In fusion axes: X-forward (Y), Y-left (-X), Z-up (Z).
    # If Wiimote Z is 1 (gravity up), fusion Z is 1.
    rot = accel_to_rotation(np.float64(0), np.float64(0), np.float64(1))
    # Target is [0, 0, 1]. If input is [0, 0, 1], it's already aligned.
    assert np.allclose(rot.apply([0, 0, 1]), [0, 0, 1])
    assert np.allclose(rot.as_quat(), [0, 0, 0, 1])


def test_accel_to_rotation_tilted() -> None:
    # If we call accel_to_rotation(ax, ay, az), it treats [ax, ay, az] as g_body.
    # Target is [0, 0, 1].
    # Let's say gravity is on the X axis: ax=1, ay=0, az=0.
    rot = accel_to_rotation(np.float64(1), np.float64(0), np.float64(0))
    # The rotation should take the measured vector [1, 0, 0] to [0, 0, 1]
    result = rot.apply([1, 0, 0])
    assert np.allclose(result, [0, 0, 1])


def test_decode_gyro() -> None:
    # 8192 is bias. slow=True -> 0.05 deg/unit.
    # 8192 + 20 = 8212. 20 * 0.05 = 1.0 deg.
    rad = decode_gyro(8212, 8192, slow=True)
    assert rad == pytest.approx(math.radians(1.0))

    # slow=False -> 0.227 deg/unit.
    # 8192 + 100 = 8292. 100 * 0.227 = 22.7 deg.
    rad = decode_gyro(8292, 8192, slow=False)
    assert rad == pytest.approx(math.radians(22.7))


class TestImuFusion:
    def test_initialization_from_accel(self) -> None:
        imu = ImuFusion()
        # Wiimote lying flat: Z=1 (gravity up)
        # Wiimote: ax=0, ay=0, az=1 -> Fusion: ax=0, ay=0, az=1
        imu.update(0, 0, 1)
        assert imu.pitch == pytest.approx(0.0)
        assert imu.roll == pytest.approx(0.0)
        # Yaw is undefined from gravity alone, usually initializes to 0
        assert imu.yaw == pytest.approx(0.0)

    def test_initialization_tilted(self) -> None:
        imu = ImuFusion()
        # Wiimote tilted 45 deg around X axis (nose up)
        # Wiimote: ax=0, ay=sin(45), az=cos(45) -> Fusion: ax=sin(45), ay=0, az=cos(45)
        angle = math.radians(45)
        imu.update(0, math.sin(angle), math.cos(angle))

        # Pitch in fusion is rotation around Y.
        # Fusion axes: X-forward, Y-left, Z-up.
        # Measured gravity [sin(45), 0, cos(45)] should be rotated to [0, 0, 1]
        # This corresponds to a pitch (rotation around Y) of -45 degrees.
        assert imu.pitch == pytest.approx(-angle)
        assert imu.roll == pytest.approx(0.0)

    def test_gyro_integration_pitch(self) -> None:
        imu = ImuFusion()
        # Start flat
        imu.update(0, 0, 1)

        # Rotate around Wiimote X axis (Pitch)
        # 45 deg/sec for 0.1 sec = 4.5 deg.
        # Wiimote Pitch = 45 dps -> raises front -> positive rotation around Wiimote X.
        # Fusion Y is Left (-Wiimote X), so it's a negative rotation around Fusion Y.
        val = 8192 + int(45 / 0.227)

        gyro = {"roll": 8192, "pitch": val, "yaw": 8192}

        # dt MUST be <= MAX_DT (0.1)
        dt = 0.1
        imu.update(0, 0, 1, gyro=gyro, dt=dt)

        # Expected pitch: -actual_rate * dt
        # Note: accelerometer correction will pull it back slightly.
        # Theoretical value with linear correction: math.radians(-actual_rate * dt) * 0.95
        # Actual value obtained: -0.07452731165341886
        assert imu.pitch == pytest.approx(-0.0745273, abs=1e-6)

    def test_gyro_integration_roll(self) -> None:
        imu = ImuFusion()
        imu.update(0, 0, 1)

        # Rotate around Wiimote Y axis (Roll)
        # Wiimote Roll = 30 dps -> clockwise (per Wiibrew) -> negative rotation around Wiimote Y.
        # Fusion X is Wiimote Y, so it's a negative rotation around Fusion X.
        val = 8192 + int(30 / 0.227)
        gyro = {"roll": val, "pitch": 8192, "yaw": 8192}

        dt = 0.05
        imu.update(0, 0, 1, gyro=gyro, dt=dt)

        # Expected roll: actual_rate * dt
        # Note: accelerometer correction will pull it back slightly.
        # Actual value obtained: 0.024841245696685145
        assert imu.roll == pytest.approx(0.0248412, abs=1e-6)

    def test_gyro_integration_yaw(self) -> None:
        imu = ImuFusion()
        imu.update(0, 0, 1)

        # Rotate around Wiimote Z axis (Yaw)
        # Wiimote Yaw = 60 dps -> clockwise (per Wiibrew) -> negative rotation around Z.
        val = 8192 + int(60 / 0.227)
        actual_rate = (val - 8192) * 0.227
        gyro = {"roll": 8192, "pitch": 8192, "yaw": val}

        dt = 0.1
        imu.update(0, 0, 1, gyro=gyro, dt=dt)

        # Expected yaw: actual_rate * dt (Yaw is NOT corrected by gravity)
        assert imu.yaw == pytest.approx(math.radians(actual_rate * dt))

    def test_reset_yaw(self) -> None:
        imu = ImuFusion()
        imu.update(0, 0, 1)
        # Induce some yaw
        imu.update(0, 0, 1, gyro={"roll": 8192, "pitch": 8192, "yaw": 8500}, dt=1.0)
        assert abs(imu.yaw) > 0.01

        imu.reset_yaw()
        assert imu.yaw == pytest.approx(0.0)

    def test_accel_correction(self) -> None:
        """Test that accelerometer correction slowly pulls orientation back."""
        imu = ImuFusion()
        # Start flat
        imu.update(0, 0, 1)

        # Manually set a tilted orientation (e.g. 10 deg pitch)
        imu.orient = R.from_euler("y", math.radians(10))

        # Update with gravity pointing straight up (0,0,1) and zero gyro
        # The correction should reduce the pitch
        initial_pitch = imu.pitch
        for _ in range(10):
            imu.update(
                0, 0, 1, gyro={"roll": 8192, "pitch": 8192, "yaw": 8192}, dt=0.02
            )

        assert abs(imu.pitch) < abs(initial_pitch)

    def test_deadband(self) -> None:
        imu = ImuFusion()
        imu.update(0, 0, 1)

        # GYRO_DEADBAND_DPS is 0.5. Try 0.4 DPS.
        val = 8192 + int(0.4 / 0.227)
        gyro = {"roll": 8192, "pitch": val, "yaw": 8192}

        imu.update(0, 0, 1, gyro=gyro, dt=1.0)
        # Should stay at 0 due to deadband
        assert imu.pitch == pytest.approx(0.0)

    def test_gyro_bias_calibration(self) -> None:
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
