import pytest
from collections.abc import Generator
from weeee.core import Wiimote, Lowlevel_Wiimote, buttons, opcode
from weeee.simulator import SimulatedHIDDevice


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def sim_wiimote() -> Generator[tuple[SimulatedHIDDevice, Wiimote], None, None]:
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    wiimote = Wiimote(device=sim_device)
    yield sim_device, wiimote


# ---------------------------------------------------------------------------
# Existing basic tests (unchanged behaviour)
# ---------------------------------------------------------------------------


def test_simulator_basic_buttons(
    sim_wiimote: tuple[SimulatedHIDDevice, Wiimote],
) -> None:
    sim_device, wiimote = sim_wiimote
    sim_device.set_buttons(buttons.BUTTON_A.value)
    wiimote.update()
    assert wiimote.is_pressed(buttons.BUTTON_A.value)
    assert not wiimote.is_pressed(buttons.BUTTON_B.value)


def test_simulator_accel(sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]) -> None:
    sim_device, wiimote = sim_wiimote
    sim_device.set_accel(512, 512, 612)
    wiimote.update()
    gx, gy, gz = wiimote.gforce
    assert gx == pytest.approx(0.0)
    assert gy == pytest.approx(0.0)
    assert gz == pytest.approx(1.0)


def test_simulator_rumble(sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]) -> None:
    sim_device, wiimote = sim_wiimote
    wiimote.rumble(True)
    assert sim_device.rumble is True
    wiimote.rumble(False)
    assert sim_device.rumble is False


def test_simulator_leds(sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]) -> None:
    sim_device, wiimote = sim_wiimote
    wiimote.player_led(p1=True, p3=True)
    assert sim_device.leds == 0x05


def test_simulator_motion_plus_detection() -> None:
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.set_motion_plus(True)
    wiimote = Wiimote(device=sim_device)
    mp_id = wiimote.detect_motion_plus()
    assert wiimote.is_motion_plus_id(mp_id)
    assert wiimote.motion_plus_id == mp_id


def test_simulator_motion_plus_activation() -> None:
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.set_motion_plus(True)
    wiimote = Wiimote(device=sim_device, require_motion_plus=True)
    assert wiimote.motion_plus_activated is True
    assert sim_device.reporting_mode == 0x35
    assert sim_device.extension_connected is True


# ---------------------------------------------------------------------------
# Critical fix: no false-positive MP activation without set_motion_plus()
# ---------------------------------------------------------------------------


def test_mode_35_extension_bytes_all_zero_when_not_connected() -> None:
    """Mode 0x35 must NOT produce plausible MP data when extension is
    not connected — regression test for the false-positive bug."""
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    wiimote = Wiimote(device=sim_device)  # no MotionPlus
    wiimote.set_reporting_mode(opcode.DATA_35.value, continuous=True)

    sim_device.set_gyro(0x1234, 0x5678, 0x9ABC)
    report = sim_device._generate_data_report()

    assert report[0] == 0x35
    # All 16 extension bytes must be zero when no extension is connected
    assert report[6:22] == [0] * 16


def test_mode_35_extension_non_zero_when_connected() -> None:
    """Mode 0x35 must produce plausible MP data when extension IS connected."""
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.set_motion_plus(True)
    sim_device.set_motion_plus_active(True)
    sim_device.reporting_mode = 0x35
    sim_device.set_gyro(
        0x1234, 0x2D78, 0x1ABC, yaw_slow=False, roll_slow=True, pitch_slow=False
    )

    report = sim_device._generate_data_report()
    assert report[0] == 0x35
    # Byte 5 must have bit 1 set (plausible check in is_plausible_motion_plus_report)
    assert report[11] & 0x02

    decoded = Wiimote.decode_motion_plus_report(bytes(report[6:12]))
    assert decoded["yaw"] == 0x1234
    assert decoded["roll"] == 0x2D78
    assert decoded["pitch"] == 0x1ABC
    assert decoded["yaw_slow"] is False
    assert decoded["roll_slow"] is True
    assert decoded["pitch_slow"] is False


def test_no_motion_plus_raises_connection_error() -> None:
    """require_motion_plus=True without set_motion_plus() must raise."""
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    with pytest.raises(ConnectionError):
        Wiimote(device=sim_device, require_motion_plus=True)


def test_activation_without_set_motion_plus_fails_at_stream_check() -> None:
    """Even if 0x04 is forcibly written to 0xA600FE, the probe-stream check
    must fail because extension data bytes are all zero."""
    sim_device = SimulatedHIDDevice()
    sim_device.open()

    # Set MP ID manually without calling set_motion_plus()
    from weeee.core import MOTION_PLUS_ID_ADDRESS

    sim_device.memory[MOTION_PLUS_ID_ADDRESS : MOTION_PLUS_ID_ADDRESS + 6] = [
        0x00,
        0x00,
        0xA6,
        0x20,
        0x00,
        0x05,
    ]

    wiimote = Wiimote(device=sim_device)
    with pytest.raises(ConnectionError):
        wiimote.require_motion_plus()


# ---------------------------------------------------------------------------
# Init-flow: 0xA600F0 write populates MP ID
# ---------------------------------------------------------------------------


def test_motion_plus_init_populates_id() -> None:
    """Writing 0x55 to 0xA600F0 (MOTION_PLUS_INIT_ADDRESS) must populate
    the ID at 0xA600FA when hardware is present."""
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.set_motion_plus(True)

    # Clear the ID bytes
    from weeee.core import MOTION_PLUS_ID_ADDRESS

    sim_device.memory[MOTION_PLUS_ID_ADDRESS : MOTION_PLUS_ID_ADDRESS + 6] = [0] * 6

    # Simulate the init write
    from weeee.core import MOTION_PLUS_INIT_ADDRESS

    wiim_low = Lowlevel_Wiimote(device=sim_device)
    wiim_low.write_register(MOTION_PLUS_INIT_ADDRESS, 0x55)

    id_bytes = bytes(
        sim_device.memory[MOTION_PLUS_ID_ADDRESS : MOTION_PLUS_ID_ADDRESS + 6]
    )
    assert Wiimote.is_motion_plus_id(id_bytes)


# ---------------------------------------------------------------------------
# Reporting-mode output format round-trips
# ---------------------------------------------------------------------------


def test_report_mode_30_format() -> None:
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.reporting_mode = 0x30
    sim_device.set_buttons(buttons.BUTTON_A.value | buttons.BUTTON_LEFT.value)

    report = sim_device._generate_data_report()
    assert report[0] == 0x30
    assert len(report) == 3
    assert (report[1] << 8) | report[2] == (
        buttons.BUTTON_A.value | buttons.BUTTON_LEFT.value
    )


def test_report_mode_31_format_roundtrip() -> None:
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.reporting_mode = 0x31
    sim_device.set_accel(515, 514, 514)  # 10-bit values
    sim_device.set_buttons(0x6060)  # known LSB bits

    report = sim_device._generate_data_report()
    assert report[0] == 0x31
    assert len(report) == 6

    # Parse via Lowlevel_Wiimote to verify roundtrip
    wm = Lowlevel_Wiimote(device=sim_device)
    wm.buttons = (report[1] << 8) | report[2]
    wm._parse_accel(report[3:6])
    assert wm.accel == (515, 514, 514)


def test_report_mode_33_ir_roundtrip() -> None:
    """Set IR dots → generate 0x33 report → parse with _parse_ir_extended."""
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.reporting_mode = 0x33
    # IR dots at known positions (x, y) – each 10-bit value < 1024
    sim_device.ir = [(100, 200), (300, 400), (500, 600), (700, 800)]

    report = sim_device._generate_data_report()
    assert report[0] == 0x33
    assert len(report) == 18

    wm = Lowlevel_Wiimote(device=sim_device)
    wm._parse_ir_extended(report[6:18])
    assert wm.ir == [(100, 200), (300, 400), (500, 600), (700, 800)]


def test_report_mode_33_ir_out_of_range() -> None:
    """IR dots at (1023,1023) should be treated as 'not seen'."""
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.reporting_mode = 0x33
    sim_device.ir = [(1023, 1023), (500, 600), (1023, 1023), (1023, 1023)]

    report = sim_device._generate_data_report()
    wm = Lowlevel_Wiimote(device=sim_device)
    wm._parse_ir_extended(report[6:18])
    # Dot 1 at (500,600) should be seen
    assert wm.ir[1] == (500, 600)
    # All others beyond 1023 should be None
    # _parse_ir_extended uses "if x < 1023 or y < 1023"
    # Since (1023, 1023): neither < 1023 → None
    assert wm.ir[0] is None
    assert wm.ir[1] == (500, 600)
    assert wm.ir[2] is None
    assert wm.ir[3] is None


# ---------------------------------------------------------------------------
# Read / timeout behaviour
# ---------------------------------------------------------------------------


def test_read_with_timeout_still_returns_data(
    sim_wiimote: tuple[SimulatedHIDDevice, Wiimote],
) -> None:
    """With a positive timeout and no queued / continuous data, read()
    still returns a data report (the real Wiimote always sends the first
    report after a mode change, even in non-continuous mode)."""
    sim_device, wiimote = sim_wiimote
    sim_device.continuous_reporting = False
    sim_device.response_queue.clear()
    sim_device.reporting_mode = 0x30
    sim_device.set_buttons(buttons.BUTTON_A.value)
    result = sim_device.read(64, timeout_ms=50)
    assert result != []
    assert result[0] == 0x30
    assert (result[1] << 8) | result[2] == buttons.BUTTON_A.value


def test_read_non_blocking_returns_data_immediately(
    sim_wiimote: tuple[SimulatedHIDDevice, Wiimote],
) -> None:
    """With timeout=0 (non-blocking), read() returns a data report even if
    the queue is empty."""
    sim_device, wiimote = sim_wiimote
    sim_device.response_queue.clear()
    sim_device.reporting_mode = 0x30
    sim_device.set_buttons(buttons.BUTTON_HOME.value)
    result = sim_device.read(64, timeout_ms=0)
    assert result[0] == 0x30
    assert (result[1] << 8) | result[2] == buttons.BUTTON_HOME.value


def test_continuous_reporting_ignores_timeout(
    sim_wiimote: tuple[SimulatedHIDDevice, Wiimote],
) -> None:
    """In continuous mode, read() always returns a data report even with
    a positive timeout."""
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.reporting_mode = 0x31
    sim_device.continuous_reporting = True
    sim_device.response_queue.clear()
    result = sim_device.read(64, timeout_ms=100)
    assert result[0] == 0x31


# ---------------------------------------------------------------------------
# ACK draining
# ---------------------------------------------------------------------------


def test_ack_reports_are_silently_drained() -> None:
    """ACK reports (0x22) in the response queue must be silently removed
    by read() and not returned to the caller."""
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    # Build a queue: [ACK, ACK, STATUS]
    sim_device._enqueue_ack(opcode.WRITE_MEMORY_AND_REGISTERS.value, 0)
    sim_device._enqueue_ack(opcode.WRITE_MEMORY_AND_REGISTERS.value, 0)
    sim_device._enqueue_status_report()

    result = sim_device.read(64)
    assert result[0] == opcode.STATUS_INFORMATION.value


def test_ack_drain_does_not_affect_queued_data_reports() -> None:
    """ACK draining must leave non-ACK reports in the queue untouched."""
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device._enqueue_ack(opcode.WRITE_MEMORY_AND_REGISTERS.value, 0)
    sim_device._enqueue_status_report()
    sim_device._enqueue_ack(opcode.WRITE_MEMORY_AND_REGISTERS.value, 0)

    # First read should drain first ACK and pop status
    r1 = sim_device.read(64)
    assert r1[0] == opcode.STATUS_INFORMATION.value

    # Second read drains remaining ACK, no data report yet → generates one
    sim_device.reporting_mode = 0x30
    sim_device.set_buttons(0)
    r2 = sim_device.read(64)
    assert r2[0] == 0x30


# ---------------------------------------------------------------------------
# Memory read / write
# ---------------------------------------------------------------------------


def test_memory_write_and_read_back() -> None:
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    wm = Lowlevel_Wiimote(device=sim_device)

    wm.write_register(0xA600FE, 0x04)
    readback = bytes(sim_device.memory[0xA600FE:0xA600FF])
    assert readback == b"\x04"


def test_read_memory_block() -> None:
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.set_motion_plus(True)

    wiimote = Wiimote(device=sim_device)
    # After init, the response queue may be dirty; detect_motion_plus works
    # around that, but here we read directly.
    wiimote.read_memory(0x16, 10)
    rid = wiimote.read(100)
    assert rid == opcode.READ_MEMORY_AND_REGISTERS_DATA.value


# ---------------------------------------------------------------------------
# IOError on closed device
# ---------------------------------------------------------------------------


def test_write_on_closed_device_raises() -> None:
    sim_device = SimulatedHIDDevice()
    with pytest.raises(IOError):
        sim_device.write([0x11, 0x10])


def test_read_on_closed_device_raises() -> None:
    sim_device = SimulatedHIDDevice()
    with pytest.raises(IOError):
        sim_device.read(64)


# ---------------------------------------------------------------------------
# Memory address boundary: large reads generate multiple chunks
# ---------------------------------------------------------------------------


def test_large_read_generates_multiple_chunks() -> None:
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.memory[0x1000:0x1020] = range(0x20)

    wm = Lowlevel_Wiimote(device=sim_device)
    wm.read_memory(0x1000, 32)

    queue = sim_device.response_queue
    assert len(queue) == 2  # Two 16-byte chunks
    assert queue[0][0] == opcode.READ_MEMORY_AND_REGISTERS_DATA.value
    assert queue[1][0] == opcode.READ_MEMORY_AND_REGISTERS_DATA.value


# ---------------------------------------------------------------------------
# Gyro -> extension report roundtrip via decode_motion_plus_report
# ---------------------------------------------------------------------------


def test_gyro_roundtrip_via_decode() -> None:
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    sim_device.set_motion_plus(True)
    sim_device.set_motion_plus_active(True)
    sim_device.reporting_mode = 0x35

    test_cases = [
        (0x0000, 0x1111, 0x2222, False, False, False),
        (0x3FFF, 0x3FFF, 0x3FFF, True, True, True),
        (0x0800, 0x1000, 0x2000, False, True, False),
    ]
    for yaw, roll, pitch, yaw_slow, roll_slow, pitch_slow in test_cases:
        sim_device.set_gyro(
            yaw,
            roll,
            pitch,
            yaw_slow=yaw_slow,
            roll_slow=roll_slow,
            pitch_slow=pitch_slow,
        )
        report = sim_device._generate_data_report()
        decoded = Wiimote.decode_motion_plus_report(bytes(report[6:12]))
        assert decoded["yaw"] == yaw
        assert decoded["roll"] == roll
        assert decoded["pitch"] == pitch
        assert decoded["yaw_slow"] == yaw_slow
        assert decoded["roll_slow"] == roll_slow
        assert decoded["pitch_slow"] == pitch_slow
