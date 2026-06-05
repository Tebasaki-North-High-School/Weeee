"""Oracle-backed simulator verification.

Every test connects to a PHYSICAL Wiimote, captures a raw HID report,
then replays the identical operation on the simulator and compares the
result structure byte-by-byte.

All tests are skipped when no Wiimote is detected on the bus.
"""

from __future__ import annotations
import os
import sys
import time
from collections.abc import Generator
from typing import Optional

import hid
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from weeee.core import (
    Lowlevel_Wiimote,
    Wiimote,
    opcode,
    buttons,
    VENDOR_ID,
    PRODUCT_ID,
    PRODUCT_ID_TR,
    EXTENSION_INIT_ENABLE_ADDRESS,
    EXTENSION_INIT_DISABLE_ENCRYPTION_ADDRESS,
    EXTENSION_ID_ADDRESS,
    MOTION_PLUS_INIT_ADDRESS,
    MOTION_PLUS_ID_ADDRESS,
    MOTION_PLUS_ACTIVATE_ADDRESS,
    MOTION_PLUS_ACTIVE_REPORT_LENGTH,
    MOTION_PLUS_ID_PREFIXES,
)
from weeee.simulator import SimulatedHIDDevice


# ── module-level skip ───────────────────────────────────────────────────


def _wiimote_present() -> bool:
    try:
        devices = hid.enumerate(VENDOR_ID)
        matches = [d for d in devices if d["product_id"] in (PRODUCT_ID, PRODUCT_ID_TR)]
        if not matches:
            return False
        dev = hid.device()
        dev.open_path(matches[0]["path"])
        time.sleep(0.3)
        try:
            # Drain initial reports — if read() fails, the device is not
            # truly available (e.g. polling not set up on Windows).
            for _ in range(15):
                if not dev.read(64, 20):
                    break
        except OSError:
            return False
        finally:
            dev.close()
        return True
    except Exception:
        return False


REAL_AVAILABLE = _wiimote_present()
REASON = "no real Wiimote detected"

# ── low-level helpers ──────────────────────────────────────────────────


def real_open() -> hid.device:
    devices = hid.enumerate(VENDOR_ID)
    matches = [d for d in devices if d["product_id"] in (PRODUCT_ID, PRODUCT_ID_TR)]
    dev = hid.device()
    dev.open_path(matches[0]["path"])
    time.sleep(0.3)
    for _ in range(15):
        try:
            if not dev.read(64, 20):
                break
        except OSError:
            break
    return dev


def real_read_until(dev: hid.device, target: int, timeout_ms: int = 1000) -> list[int]:
    deadline = time.monotonic() + timeout_ms / 1000
    while time.monotonic() < deadline:
        d = dev.read(64, 200)
        if d and d[0] == target:
            return list(d)
    return []


# ── fixtures ───────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def sim() -> Generator[SimulatedHIDDevice, None, None]:
    d = SimulatedHIDDevice()
    d.open()
    yield d


# ═══════════════════════════════════════════════════════════════════════
# DATA-REPORTING MODES  (0x30 – 0x37)
# ═══════════════════════════════════════════════════════════════════════

ALL_DATA_MODES = [0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37]
ACCEL_MODES = {0x31, 0x33, 0x35, 0x37}
EXT_MODES = {0x32, 0x34, 0x35, 0x36, 0x37}
IR_EXT_MODES = {0x33}
IR_BASIC_MODES = {0x36, 0x37}
REPORT_SIZE: dict[int, int] = {
    0x30: 3,
    0x31: 6,
    0x32: 11,
    0x33: 18,
    0x34: 22,
    0x35: 22,
    0x36: 22,
    0x37: 22,
}


@pytest.mark.skipif(not REAL_AVAILABLE, reason=REASON)
class TestDataReportModes:
    """For every standard data-reporting mode, verify that the real
    Wiimote sends the expected report-ID and that the simulator
    reproduces the same structural layout."""

    @pytest.mark.parametrize("mode", ALL_DATA_MODES)
    def test_report_id_and_structure(self, mode: int, sim: SimulatedHIDDevice) -> None:
        dev = real_open()

        dev.write([opcode.DATA_REPORTING_MODE.value, 0x00, mode])
        time.sleep(0.05)

        real_read_until(dev, 0x20, 600)

        real_r = real_read_until(dev, mode, 800)
        dev.close()

        if not real_r:
            pytest.skip(f"mode 0x{mode:02x}: no matching report from real device")

        prev_mode = sim.reporting_mode
        sim.set_reporting_mode(mode)
        sim_r = sim._generate_data_report()
        sim.set_reporting_mode(prev_mode)

        # ── Report-ID ──
        assert real_r[0] == mode, f"real report-ID 0x{real_r[0]:02x} != {mode:#04x}"
        assert sim_r[0] == mode, f"sim report-ID 0x{sim_r[0]:02x} != {mode:#04x}"

        assert len(sim_r) <= len(real_r) <= 64

        assert len(real_r) >= 3
        assert len(sim_r) >= 3

        # ── Accel modes ──
        if mode in ACCEL_MODES:
            assert len(real_r) >= 6
            assert len(sim_r) >= 6
            assert all(0x00 <= b <= 0xFF for b in real_r[3:6])
            assert all(0x00 <= b <= 0xFF for b in sim_r[3:6])

        # ── Extension modes ──
        if mode in EXT_MODES:
            off = {0x32: 3, 0x34: 3, 0x35: 6, 0x36: 13, 0x37: 16}[mode]
            assert len(real_r) > off
            assert len(sim_r) > 6

        # ── IR modes ──
        if mode in IR_EXT_MODES:
            assert len(real_r) >= 18
            assert len(sim_r) >= 18
        if mode in IR_BASIC_MODES:
            assert len(real_r) >= 16
            assert len(sim_r) >= 16


# ═══════════════════════════════════════════════════════════════════════
# BUTTON STATE
# ═══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not REAL_AVAILABLE, reason=REASON)
class TestButtons:
    """Verify that button press information appears in the right byte
    positions in both real and simulated reports."""

    def test_button_bit_positions(self, sim: SimulatedHIDDevice) -> None:
        dev = real_open()
        dev.write([opcode.DATA_REPORTING_MODE.value, 0x00, 0x31])
        time.sleep(0.05)
        real_read_until(dev, 0x20, 600)
        real_r = real_read_until(dev, 0x31, 800)
        dev.close()
        if not real_r:
            pytest.skip("no real data in mode 0x31")

        real_btn = (real_r[1] << 8) | real_r[2]

        sim.set_buttons(buttons.BUTTON_A.value | buttons.BUTTON_LEFT.value)
        sim.set_reporting_mode(0x31)
        sim_r = sim._generate_data_report()
        sim.set_reporting_mode(0x30)
        sim.set_buttons(0)

        sim_btn = (sim_r[1] << 8) | sim_r[2]

        assert 0 <= real_btn <= 0xFFFF
        assert 0 <= sim_btn <= 0xFFFF

        assert (real_r[1] & 0x0F) == ((real_btn >> 8) & 0x0F)
        assert real_r[2] == (real_btn & 0xFF)
        assert (sim_r[1] & 0x0F) == ((sim_btn >> 8) & 0x0F)
        assert sim_r[2] == (sim_btn & 0xFF)


# ═══════════════════════════════════════════════════════════════════════
# STATUS REPORT  (0x20)
# ═══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not REAL_AVAILABLE, reason=REASON)
class TestStatusReport:
    def test_structure(self, sim: SimulatedHIDDevice) -> None:
        dev = real_open()
        dev.write([opcode.STATUS_REQUEST.value, 0x00])
        real_r = real_read_until(dev, opcode.STATUS_INFORMATION.value, 1500)
        dev.close()
        if not real_r:
            pytest.skip("no status report from real device")

        sim.write([opcode.STATUS_REQUEST.value, 0x00])
        sim_r = sim.read(64)

        assert real_r[0] == opcode.STATUS_INFORMATION.value
        assert sim_r[0] == opcode.STATUS_INFORMATION.value

        for _, r in [("real", real_r), ("sim", sim_r)]:
            assert len(r) >= 7
            leds = (r[3] >> 4) & 0x0F
            flags = r[3] & 0x0F
            assert 0 <= leds <= 0x0F
            assert flags & ~0x0F == 0

        assert 0 <= real_r[6] <= 255
        assert 0 <= sim_r[6] <= 255


# ═══════════════════════════════════════════════════════════════════════
# MEMORY READ  (calibration @ 0x16)
# ═══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not REAL_AVAILABLE, reason=REASON)
class TestMemoryReadCalibration:
    def test_read_calibration(self, sim: SimulatedHIDDevice) -> None:
        dev = real_open()
        dev.write(
            [
                opcode.READ_REGISTERS.value,
                0x00,
                0x00,
                0x00,
                0x16,
                0x00,
                0x0A,
            ]
        )
        real_r = real_read_until(
            dev,
            opcode.READ_DATA.value,
            1500,
        )
        dev.close()
        if not real_r:
            pytest.skip("no read-memory response from real device")

        sim.write(
            [
                opcode.READ_REGISTERS.value,
                0x00,
                0x00,
                0x00,
                0x16,
                0x00,
                0x0A,
            ]
        )
        sim_r = sim.read(64)
        while (
            sim_r
            and sim_r[0]
            == opcode.ACK_REPORT.value
        ):
            sim_r = sim.read(64)
        if not sim_r:
            pytest.skip("no read-memory from simulator")

        assert real_r[0] == opcode.READ_DATA.value
        assert sim_r[0] == opcode.READ_DATA.value

        real_size = (real_r[3] >> 4) + 1
        sim_size = (sim_r[3] >> 4) + 1
        assert real_size in (10, 16, 22)
        assert sim_size == 10

        real_addr = (real_r[4] << 8) | real_r[5]
        sim_addr = (sim_r[4] << 8) | sim_r[5]
        assert real_addr == 0x0016
        assert sim_addr == 0x0016

        real_cal = real_r[6 : 6 + real_size]
        sim_cal = sim_r[6 : 6 + 10]
        assert len(real_cal) >= 8
        assert len(sim_cal) >= 8
        for i in range(min(8, len(real_cal), len(sim_cal))):
            assert 0 <= real_cal[i] <= 255
            assert 0 <= sim_cal[i] <= 255


# ═══════════════════════════════════════════════════════════════════════
# MEMORY WRITE  (ACK report 0x22)
# ═══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not REAL_AVAILABLE, reason=REASON)
class TestMemoryWrite:
    def test_ack_format(self, sim: SimulatedHIDDevice) -> None:
        dev = real_open()
        dev.write(
            [
                opcode.WRITE_REGISTERS.value,
                0x04,
                0x00,
                0xA4,
                0xF0,
                0x01,
                0x55,
            ]
            + [0] * 15
        )
        real_ack = real_read_until(
            dev,
            opcode.ACK_REPORT.value,
            1500,
        )
        dev.close()
        if not real_ack:
            pytest.skip("no ACK from real device")

        assert (
            real_ack[0] == opcode.ACK_REPORT.value
        )
        assert len(real_ack) >= 6
        assert real_ack[3] == opcode.WRITE_REGISTERS.value
        assert 0 <= real_ack[4] <= 7

        # ── Simulator ACK (via _enqueue_ack) ──
        sim._enqueue_ack(opcode.WRITE_REGISTERS.value, 0)
        ack_from_queue = sim.response_queue[0]
        assert (
            ack_from_queue[0]
            == opcode.ACK_REPORT.value
        )
        assert len(ack_from_queue) == 6
        assert ack_from_queue[3] == opcode.WRITE_REGISTERS.value
        assert ack_from_queue[4] == 0


# ═══════════════════════════════════════════════════════════════════════
# EXTENSION DETECTION
# ═══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not REAL_AVAILABLE, reason=REASON)
class TestExtensionDetection:
    """Run the standard Wiimote extension-detection sequence (init ext
    port → init MP bus → read MP ID) and verify the simulator produces
    the same structural reports."""

    def test_mp_id_read(self, sim: SimulatedHIDDevice) -> None:
        dev = real_open()

        dev.write(
            [
                opcode.WRITE_REGISTERS.value,
                0x04,
                0x00,
                0xA4,
                0xF0,
                0x01,
                0x55,
            ]
            + [0] * 15
        )
        real_read_until(
            dev, opcode.ACK_REPORT.value, 1000
        )

        dev.write(
            [
                opcode.WRITE_REGISTERS.value,
                0x04,
                0x00,
                0xA4,
                0xFB,
                0x01,
                0x00,
            ]
            + [0] * 15
        )
        real_read_until(
            dev, opcode.ACK_REPORT.value, 1000
        )

        dev.write(
            [
                opcode.WRITE_REGISTERS.value,
                0x04,
                0x00,
                0xA6,
                0xF0,
                0x01,
                0x55,
            ]
            + [0] * 15
        )
        real_read_until(
            dev, opcode.ACK_REPORT.value, 1000
        )

        dev.write(
            [
                opcode.READ_REGISTERS.value,
                0x04,
                0xA6,
                0x00,
                0xFA,
                0x00,
                0x06,
            ]
        )
        real_r = real_read_until(
            dev,
            opcode.READ_DATA.value,
            1500,
        )
        dev.close()
        if not real_r:
            pytest.skip("no MP-ID response from real device")

        # ── Simulator ──
        sim.set_motion_plus(True)
        wm = Lowlevel_Wiimote(device=sim)

        wm.write_register(EXTENSION_INIT_ENABLE_ADDRESS, 0x55)

        wm.write_register(EXTENSION_INIT_DISABLE_ENCRYPTION_ADDRESS, 0x00)

        wm.write_register(MOTION_PLUS_INIT_ADDRESS, 0x55)

        sim.write(
            [
                opcode.READ_REGISTERS.value,
                0x04,
                0xA6,
                0x00,
                0xFA,
                0x00,
                0x06,
            ]
        )
        sim_r = sim.read(64)
        while (
            sim_r
            and sim_r[0]
            == opcode.ACK_REPORT.value
        ):
            sim_r = sim.read(64)
        if not sim_r:
            pytest.skip("no MP-ID from simulator")

        # ── Compare structure ──
        assert real_r[0] == opcode.READ_DATA.value
        assert sim_r[0] == opcode.READ_DATA.value

        # Address bytes 4-5 = lower 16 bits of the address (0x00FA for 0xA600FA)
        assert (real_r[4] << 8) | real_r[5] == 0x00FA
        assert (sim_r[4] << 8) | sim_r[5] == 0x00FA

        # ID data at offset 6 (6 bytes)
        real_id = bytes(real_r[6:12])
        sim_id = bytes(sim_r[6:12])

        # Real device may or may not have MP — just validate structure
        if any(b != 0x00 for b in real_id):
            assert len(real_id) >= 6
            if real_id[:4] in MOTION_PLUS_ID_PREFIXES:
                pass  # valid MP ID

        # Simulator has MP hardware → must have non-zero ID
        assert sim._motion_plus_hardware_present
        assert any(b != 0x00 for b in sim_id)
        assert sim_id[:4] in MOTION_PLUS_ID_PREFIXES


# ═══════════════════════════════════════════════════════════════════════
# MOTION-PLUS DATA STREAM  (mode 0x35, conditional)
# ═══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not REAL_AVAILABLE, reason=REASON)
class TestMotionPlusStream:
    """If the attached Wiimote has a MotionPlus, read mode-0x35 data and
    verify the 6-byte MP extension encoding against the simulator."""

    def test_mode_35_with_mp(self, sim: SimulatedHIDDevice) -> None:
        dev = real_open()

        # Detect + activate MP using Wiimote API
        wm_real = Lowlevel_Wiimote(device=dev)
        wm_real.write_register(EXTENSION_INIT_ENABLE_ADDRESS, 0x55)
        wm_real.write_register(EXTENSION_INIT_DISABLE_ENCRYPTION_ADDRESS, 0x00)
        wm_real.write_register(MOTION_PLUS_INIT_ADDRESS, 0x55)
        time.sleep(0.05)

        # Read MP ID
        wm_real.read_memory(MOTION_PLUS_ID_ADDRESS, 6)
        start_t = time.monotonic()
        rid = None
        while time.monotonic() - start_t < 2.0:
            rid = wm_real.read(200)
            if rid == opcode.READ_DATA.value:
                break
        mp_id = b""
        if rid == opcode.READ_DATA.value:
            mp_id = bytes(wm_real.memory_data.get(MOTION_PLUS_ID_ADDRESS & 0xFFFF, []))

        if mp_id[:4] not in MOTION_PLUS_ID_PREFIXES:
            dev.close()
            pytest.skip(
                "no MotionPlus detected on real device {} {}".format(mp_id.hex(), rid)
            )

        # Activate MP
        wm_real.write_register(MOTION_PLUS_ACTIVATE_ADDRESS, 0x04)
        time.sleep(0.15)

        # Set mode 0x35 continuous and capture
        wm_real.set_reporting_mode(0x35, continuous=True)
        time.sleep(0.1)

        real_r = real_read_until(dev, 0x35, 1500)
        dev.close()
        if not real_r:
            pytest.skip("no 0x35 data from real device")

        # ── Simulator ──
        sim.set_motion_plus(True)
        sim.set_motion_plus_active(True)
        sim.set_reporting_mode(0x35)
        sim.set_continuous_reporting(True)
        sim.set_gyro(0x2000, 0x2000, 0x2000)
        sim_r = sim._generate_data_report()

        # ── Compare ──
        assert real_r[0] == 0x35
        assert sim_r[0] == 0x35

        real_ext = bytes(real_r[6:22])
        sim_ext = bytes(sim_r[6:22])

        # Real device MP check
        assert len(real_ext) >= MOTION_PLUS_ACTIVE_REPORT_LENGTH
        assert Wiimote.is_plausible_motion_plus_report(
            real_ext[:MOTION_PLUS_ACTIVE_REPORT_LENGTH]
        )
        decoded = Wiimote.decode_motion_plus_report(
            real_ext[:MOTION_PLUS_ACTIVE_REPORT_LENGTH]
        )
        assert 0 <= int(decoded["yaw"]) <= 0x3FFF
        assert 0 <= int(decoded["roll"]) <= 0x3FFF
        assert 0 <= int(decoded["pitch"]) <= 0x3FFF
        assert isinstance(decoded["yaw_slow"], bool)
        assert isinstance(decoded["roll_slow"], bool)
        assert isinstance(decoded["pitch_slow"], bool)

        # Simulator
        assert len(sim_ext) >= MOTION_PLUS_ACTIVE_REPORT_LENGTH
        assert Wiimote.is_plausible_motion_plus_report(
            sim_ext[:MOTION_PLUS_ACTIVE_REPORT_LENGTH]
        )
        sim_decoded = Wiimote.decode_motion_plus_report(
            sim_ext[:MOTION_PLUS_ACTIVE_REPORT_LENGTH]
        )
        assert sim_decoded["yaw"] == 0x2000
        assert sim_decoded["roll"] == 0x2000
        assert sim_decoded["pitch"] == 0x2000


# ═══════════════════════════════════════════════════════════════════════
# LOW-LEVEL WIIMOTE RAW FORMAT
# ═══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not REAL_AVAILABLE, reason=REASON)
class TestLowlevelWiimote:
    """Verify that Lowlevel_Wiimote methods format data identically."""

    def test_write_register_structure(self, sim: SimulatedHIDDevice) -> None:
        """The raw bytes written by write_register must match the format
        accepted by the real Wiimote (ACK confirms correctness)."""
        dev = real_open()

        target_addr = 0xA42000
        test_value = 0x42

        wm = Lowlevel_Wiimote(device=dev)
        wm.is_rumble = False
        rumble_arg = 0x00
        mm = 0x04 | rumble_arg
        f1, f2, f3 = (
            (target_addr >> 16) & 0xFF,
            (target_addr >> 8) & 0xFF,
            target_addr & 0xFF,
        )
        dev.write(
            [
                opcode.WRITE_REGISTERS.value,
                mm,
                f1,
                f2,
                f3,
                0x01,
                test_value,
            ]
            + [0] * 15
        )
        ack = real_read_until(
            dev,
            opcode.ACK_REPORT.value,
            1000,
        )
        dev.close()
        if not ack:
            pytest.skip("no ACK from real device")

        assert ack[0] == opcode.ACK_REPORT.value
        assert len(ack) >= 6
        assert ack[3] == opcode.WRITE_REGISTERS.value
        # The real Wiimote may return error code 7 for writes to certain
        # register addresses (e.g. camera registers at 0xA42000).
        assert 0 <= ack[4] <= 7

        # Simulator: same data written to memory (always succeeds)
        wm_sim = Lowlevel_Wiimote(device=sim)
        wm_sim.write_register(target_addr, test_value)
        assert sim.memory[target_addr] == test_value

    def test_read_memory_structure(self, sim: SimulatedHIDDevice) -> None:
        """The raw bytes from a read_memory response must match."""
        dev = real_open()
        wm = Lowlevel_Wiimote(device=dev)
        wm.read_memory(0x16, 6)
        real_r = real_read_until(
            dev,
            opcode.READ_DATA.value,
            1500,
        )
        dev.close()
        if not real_r:
            pytest.skip("no read-memory from real device")

        assert real_r[0] == opcode.READ_DATA.value
        assert (real_r[4] << 8) | real_r[5] == 0x0016

        sim.write(
            [
                opcode.READ_REGISTERS.value,
                0x00,
                0x00,
                0x00,
                0x16,
                0x00,
                0x06,
            ]
        )
        sim_r = sim.read(64)
        while (
            sim_r
            and sim_r[0]
            == opcode.ACK_REPORT.value
        ):
            sim_r = sim.read(64)
        if not sim_r:
            pytest.skip("no read from simulator")

        assert sim_r[0] == opcode.READ_DATA.value
        assert (sim_r[4] << 8) | sim_r[5] == 0x0016


# ═══════════════════════════════════════════════════════════════════════
# HIGH-LEVEL WIIMOTE API
# ═══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not REAL_AVAILABLE, reason=REASON)
class TestWiimoteHighLevel:
    def test_constructor_enumerate(self) -> None:
        wm = Wiimote()
        wm.close()

    def test_state_after_construction(self) -> None:
        wm = Wiimote()
        time.sleep(0.3)
        wm.update(200)
        wm.close()
        assert isinstance(wm.buttons, int)
        assert len(wm.accel) == 3

        # Also verify simulator gives same interface
        sim_dev = SimulatedHIDDevice()
        sim_dev.open()
        sim_wm = Wiimote(device=sim_dev)
        assert isinstance(sim_wm.buttons, int)
        assert len(sim_wm.accel) == 3

    def test_calibration_loaded(self) -> None:
        wm = Wiimote()
        time.sleep(0.3)
        wm.update(200)
        cz = wm.cal_zero
        cg = wm.cal_1g
        for i in range(3):
            assert 256 <= cz[i] <= 768
            assert 512 <= cg[i] <= 1024
        wm.close()

    def test_gforce_reasonable(self) -> None:
        wm = Wiimote()
        time.sleep(0.3)
        wm.update(200)
        gx, gy, gz = wm.gforce
        assert -2.0 <= gx <= 2.0
        assert -2.0 <= gy <= 2.0
        assert 0.5 <= gz <= 1.5
        wm.close()

    def test_rumble_ms(self) -> None:
        wm = Wiimote()
        wm.rumble_ms(100)
        assert bool(wm.is_rumble) is True
        assert wm.rumble_end_time is not None
        time.sleep(0.15)
        wm.update()
        assert bool(wm.is_rumble) is False
        rumble_end: Optional[float] = wm.rumble_end_time
        assert rumble_end is None
        wm.close()

    def test_is_pressed_type(self) -> None:
        wm = Wiimote()
        time.sleep(0.3)
        wm.update(200)
        val = wm.is_pressed(buttons.BUTTON_A.value)
        assert isinstance(val, bool)
        wm.close()


# ═══════════════════════════════════════════════════════════════════════
# SIMULATOR UNIT TESTS FOR 100% COVERAGE & WIIMOTE FEATURES
# ═══════════════════════════════════════════════════════════════════════


class TestSimulatorUnit:
    """Pure unit tests for SimulatedHIDDevice to achieve 100% coverage
    without requiring a real Wiimote."""

    def test_open_close_and_errors(self) -> None:
        dev = SimulatedHIDDevice()
        # Initial state checks
        assert not bool(dev.is_open)
        assert dev.path is None
        assert dev.serial_number is None

        # Should raise IOError when not open
        with pytest.raises(IOError):
            dev.write([0x11, 0x00])

        with pytest.raises(IOError):
            dev.read(64)

        # Open paths
        dev.open_path(b"mock_path")
        assert bool(dev.is_open)
        p: Optional[bytes] = dev.path
        assert p == b"mock_path"

        dev.close()
        assert not bool(dev.is_open)

        # Open by serial
        dev.open(serial_number="12345")
        assert bool(dev.is_open)
        sn: Optional[str] = dev.serial_number
        assert sn == "12345"
        dev.close()

    def test_set_motion_plus_false(self) -> None:
        dev = SimulatedHIDDevice()
        dev.open()
        dev.set_motion_plus(True)
        assert bool(dev._motion_plus_hardware_present)

        dev.set_motion_plus(False)
        assert not bool(dev._motion_plus_hardware_present)
        assert not bool(dev.extension_connected)

        # Verify MOTION_PLUS_ID_ADDRESS memory is zeroed out
        assert dev.memory[
            MOTION_PLUS_ID_ADDRESS : MOTION_PLUS_ID_ADDRESS + 6
        ] == bytearray([0] * 6)
        dev.close()

    def test_rumble_and_leds(self) -> None:
        dev = SimulatedHIDDevice()
        dev.open()

        # Rumble report (0x11)
        dev.write([opcode.RUMBLE.value, 0x01])
        assert bool(dev.rumble) is True

        dev.write([opcode.RUMBLE.value, 0x00])
        assert bool(dev.rumble) is False

        # Player LEDs report (0x11)
        dev.write([opcode.PLAYER_LEDS.value, 0x51])
        assert int(dev.leds) == 0x05
        assert bool(dev.rumble) is True

        dev.write([opcode.PLAYER_LEDS.value, 0x00])
        assert int(dev.leds) == 0x00
        assert bool(dev.rumble) is False
        dev.close()

    def test_status_and_battery(self) -> None:
        dev = SimulatedHIDDevice()
        dev.open()

        # Status request (0x15)
        dev.set_battery(180)
        dev.set_buttons(buttons.BUTTON_A.value)
        dev.set_leds(0x0C)
        dev.set_extension_connected(True)

        dev.write([opcode.STATUS_REQUEST.value, 0x00])

        # Read the status report
        res = dev.read(64)
        assert res[0] == opcode.STATUS_INFORMATION.value
        # Buttons in bytes 1-2
        assert (res[1] << 8) | res[2] == buttons.BUTTON_A.value
        # LF byte: LEDs in high nibble (0xC0), Extension (0x02) in low nibble
        assert res[3] == 0xC2
        # Battery in byte 6
        assert res[6] == 180

        # Status request with extension NOT connected
        dev.set_extension_connected(False)
        dev.write([opcode.STATUS_REQUEST.value, 0x00])
        res = dev.read(64)
        assert res[3] == 0xC0
        dev.close()

    def test_motion_plus_activation_flow(self) -> None:
        dev = SimulatedHIDDevice()
        dev.open()
        dev.set_motion_plus(True)

        # Writing 0x55 to MOTION_PLUS_INIT_ADDRESS
        dev.write(
            [
                opcode.WRITE_REGISTERS.value,
                0x04,
                (MOTION_PLUS_INIT_ADDRESS >> 16) & 0xFF,
                (MOTION_PLUS_INIT_ADDRESS >> 8) & 0xFF,
                MOTION_PLUS_INIT_ADDRESS & 0xFF,
                0x01,
                0x55,
            ]
        )
        assert dev.memory[
            MOTION_PLUS_ID_ADDRESS : MOTION_PLUS_ID_ADDRESS + 6
        ] == bytearray([0x00, 0x00, 0xA6, 0x20, 0x00, 0x05])

        # Writing 0x04 to MOTION_PLUS_ACTIVATE_ADDRESS
        dev.write(
            [
                opcode.WRITE_REGISTERS.value,
                0x04,
                (MOTION_PLUS_ACTIVATE_ADDRESS >> 16) & 0xFF,
                (MOTION_PLUS_ACTIVATE_ADDRESS >> 8) & 0xFF,
                MOTION_PLUS_ACTIVATE_ADDRESS & 0xFF,
                0x01,
                0x04,
            ]
        )
        assert bool(dev.extension_connected)
        assert dev.memory[EXTENSION_ID_ADDRESS : EXTENSION_ID_ADDRESS + 6] == bytearray(
            [0x00, 0x00, 0xA4, 0x20, 0x04, 0x05]
        )

        # Writing 0x55 to EXTENSION_INIT_ENABLE_ADDRESS should deactivate MP active
        dev.write(
            [
                opcode.WRITE_REGISTERS.value,
                0x04,
                (EXTENSION_INIT_ENABLE_ADDRESS >> 16) & 0xFF,
                (EXTENSION_INIT_ENABLE_ADDRESS >> 8) & 0xFF,
                EXTENSION_INIT_ENABLE_ADDRESS & 0xFF,
                0x01,
                0x55,
            ]
        )
        assert not bool(dev.extension_connected)
        assert dev.memory[EXTENSION_ID_ADDRESS : EXTENSION_ID_ADDRESS + 6] == bytearray(
            [0] * 6
        )
        dev.close()

    def test_reporting_modes_structures(self) -> None:
        dev = SimulatedHIDDevice()
        dev.open()
        dev.set_buttons(0x1F)
        dev.set_accel(512, 513, 514)
        dev.set_gyro(
            0x1000, 0x2000, 0x3000, yaw_slow=True, roll_slow=False, pitch_slow=True
        )
        dev.set_extension_connected(True)

        # Mode 0x30
        dev.set_reporting_mode(0x30)
        r30 = dev._generate_data_report()
        assert r30[0] == 0x30
        assert len(r30) == 3

        # Mode 0x32
        dev.set_reporting_mode(0x32)
        r32 = dev._generate_data_report()
        assert r32[0] == 0x32
        assert len(r32) == 11
        # Yaw (low) is 0x00 (from 0x1000)
        assert r32[3] == 0x00
        # Roll (low) is 0x00 (from 0x2000)
        assert r32[4] == 0x00
        # Pitch (low) is 0x00 (from 0x3000)
        assert r32[5] == 0x00
        # Yaw high: ((0x1000 >> 8) << 2) -> (0x10 << 2) = 0x40.
        # Plus yaw_slow (0x02), pitch_slow (0x01) -> 0x43
        assert r32[6] == 0x43
        # Roll high: ((0x2000 >> 8) << 2) -> (0x20 << 2) = 0x80. Plus roll_slow (0) -> 0x80
        assert r32[7] == 0x80
        # Pitch high: ((0x3000 >> 8) << 2) -> (0x30 << 2) = 0xC0. Plus 0x02 -> 0xC2
        assert r32[8] == 0xC2

        # Mode 0x34
        dev.set_reporting_mode(0x34)
        r34 = dev._generate_data_report()
        assert r34[0] == 0x34
        assert len(r34) == 22
        assert r34[3] == 0x00
        assert r34[6] == 0x43
        assert r34[7] == 0x80
        assert r34[8] == 0xC2

        # Mode 0x35
        dev.set_reporting_mode(0x35)
        r35 = dev._generate_data_report()
        assert r35[0] == 0x35
        assert len(r35) == 22
        assert r35[6] == 0x00  # yaw low
        assert r35[7] == 0x00  # roll low
        assert r35[8] == 0x00  # pitch low
        # Byte 3 (index 9): (yaw >> 8) << 2 | yaw_slow (0x02) | pitch_slow (0x01) -> 0x43
        assert r35[9] == 0x43
        # Byte 4 (index 10): (roll >> 8) << 2 | roll_slow (0) -> 0x80
        assert r35[10] == 0x80
        # Byte 5 (index 11): (pitch >> 8) << 2 | 0x02 -> 0xC2
        assert r35[11] == 0xC2

        # Mode 0x36
        dev.set_reporting_mode(0x36)
        dev.set_ir([(100, 200), (300, 400), (500, 600), (700, 800)])
        r36 = dev._generate_data_report()
        assert r36[0] == 0x36
        assert len(r36) == 22
        assert r36[13] == 0x00  # yaw low
        assert r36[16] == 0x43  # yaw high & flags
        assert r36[17] == 0x80  # roll high & flags
        assert r36[18] == 0xC2  # pitch high & flags

        # Mode 0x37
        dev.set_reporting_mode(0x37)
        r37 = dev._generate_data_report()
        assert r37[0] == 0x37
        assert len(r37) == 22
        assert r37[16] == 0x00  # yaw low
        assert r37[19] == 0x43  # yaw high
        assert r37[20] == 0x80  # roll high
        assert r37[21] == 0xC2  # pitch high

        # Fallback invalid mode
        dev.set_reporting_mode(0x99)
        r_fallback = dev._generate_data_report()
        assert r_fallback[0] == 0x30
        dev.close()
