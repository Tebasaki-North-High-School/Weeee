import pytest
import time
from collections.abc import Generator

from src.weeee.wiimote import (
    Wiimote,
    opcode,
    VENDOR_ID,
    PRODUCT_ID,
    PRODUCT_ID_TR,
    buttons,
    IR_SENSITIVITY_BLOCKS,
    MOTION_PLUS_ACTIVE_ID,
    MOTION_PLUS_ACTIVE_ID_TR,
    MOTION_PLUS_INACTIVE_ID,
    MOTION_PLUS_ID_PREFIX,
    MOTION_PLUS_ACTIVE_REPORT_LENGTH,
    EXTENSION_INIT_ENABLE_ADDRESS,
    EXTENSION_INIT_DISABLE_ENCRYPTION_ADDRESS,
    EXTENSION_ID_ADDRESS,
    MOTION_PLUS_INIT_ADDRESS,
    MOTION_PLUS_ID_ADDRESS,
    MOTION_PLUS_ACTIVATE_ADDRESS,
)
from src.weeee.simulator import SimulatedHIDDevice


@pytest.fixture
def sim_wiimote() -> Generator[tuple[SimulatedHIDDevice, Wiimote], None, None]:
    sim_device = SimulatedHIDDevice()
    sim_device.open()
    wiimote = Wiimote(device=sim_device)
    yield sim_device, wiimote


@pytest.mark.constants
class TestConstants:
    @pytest.mark.parametrize(
        "button, expected_value",
        [
            (buttons.BUTTON_LEFT, 0x0100),
            (buttons.BUTTON_RIGHT, 0x0200),
            (buttons.BUTTON_DOWN, 0x0400),
            (buttons.BUTTON_UP, 0x0800),
            (buttons.BUTTON_PLUS, 0x1000),
            (buttons.BUTTON_TWO, 0x0001),
            (buttons.BUTTON_ONE, 0x0002),
            (buttons.BUTTON_B, 0x0004),
            (buttons.BUTTON_A, 0x0008),
            (buttons.BUTTON_MINUS, 0x0010),
            (buttons.BUTTON_HOME, 0x0080),
        ],
    )
    def test_button_values(self, button: buttons, expected_value: int) -> None:
        assert button.value == expected_value

    def test_buttons_do_not_overlap(self) -> None:
        union = 0
        for b in buttons:
            assert union & b.value == 0, (
                f"Button mask {b.value:#06x} overlaps with existing {union:#06x}"
            )
            union |= b.value

    def test_vendor_id_is_nintendo(self) -> None:
        assert VENDOR_ID == 0x057E

    def test_product_ids(self) -> None:
        assert PRODUCT_ID == 0x0306
        assert PRODUCT_ID_TR == 0x0330


@pytest.mark.constants
class TestIRConstants:
    def test_all_levels_present(self) -> None:
        for level in range(1, 6):
            assert level in IR_SENSITIVITY_BLOCKS

    def test_each_block_is_tuple_of_two_lists(self) -> None:
        for level in range(1, 6):
            block1, block2 = IR_SENSITIVITY_BLOCKS[level]
            assert len(block1) == 9
            assert len(block2) == 2

    def test_block1_matches_wiki_wii_level1(self) -> None:
        b1, b2 = IR_SENSITIVITY_BLOCKS[1]
        assert b1 == [0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0x64, 0x00, 0xFE]
        assert b2 == [0xFD, 0x05]

    def test_block1_matches_wiki_wii_level5(self) -> None:
        b1, b2 = IR_SENSITIVITY_BLOCKS[5]
        assert b1 == [0x07, 0x00, 0x00, 0x71, 0x01, 0x00, 0x72, 0x00, 0x20]
        assert b2 == [0x1F, 0x03]


@pytest.mark.constants
class MotionPlusConstantsTests:
    @pytest.mark.parametrize(
        "target, expected_value",
        [
            (MOTION_PLUS_ACTIVE_ID, bytes([0x00, 0x00, 0xA4, 0x20, 0x04, 0x05])),
            (MOTION_PLUS_ACTIVE_ID_TR, bytes([0x01, 0x00, 0xA4, 0x20, 0x04, 0x05])),
            (MOTION_PLUS_INACTIVE_ID, bytes([0x00, 0x00, 0xA6, 0x20, 0x00, 0x05])),
            (MOTION_PLUS_ID_PREFIX, bytes([0x00, 0x00, 0xA6, 0x20])),
        ],
    )
    def test_motionplus_bytes_values(
        self, target: bytes, expected_value: bytes
    ) -> None:
        assert target == expected_value

    @pytest.mark.parametrize(
        "target, expected_value",
        [
            (MOTION_PLUS_ACTIVE_REPORT_LENGTH, 6),
            (EXTENSION_INIT_ENABLE_ADDRESS, 0xA400F0),
            (EXTENSION_INIT_DISABLE_ENCRYPTION_ADDRESS, 0xA400FB),
            (EXTENSION_ID_ADDRESS, 0xA400FA),
            (MOTION_PLUS_INIT_ADDRESS, 0xA600F0),
            (MOTION_PLUS_ID_ADDRESS, 0xA600FA),
            (MOTION_PLUS_ACTIVATE_ADDRESS, 0xA600FE),
        ],
    )
    def test_motionplus_values(self, target: int, expected_value: int) -> None:
        assert target == expected_value


@pytest.mark.constants
class TestOpcodeEnum:
    @pytest.mark.parametrize(
        "opcode, expected_value",
        [
            # 出力レポート Opcodes
            (opcode.RUMBLE, 0x10),
            (opcode.PLAYER_LEDS, 0x11),
            (opcode.DATA_REPORTING_MODE, 0x12),
            (opcode.IR_CAMERA_ENABLE, 0x13),
            (opcode.SPEAKER_ENABLE, 0x14),
            (opcode.STATUS_INFORMATION_REQUEST, 0x15),
            (opcode.WRITE_MEMORY_AND_REGISTERS, 0x16),
            (opcode.READ_MEMORY_AND_REGISTERS, 0x17),
            (opcode.SPEAKER_DATA, 0x18),
            (opcode.SPEAKER_MUTE, 0x19),
            (opcode.IR_CAMERA_ENABLE_2, 0x1A),
            # 入力レポート Opcodes
            (opcode.STATUS_INFORMATION, 0x20),
            (opcode.READ_MEMORY_AND_REGISTERS_DATA, 0x21),
            (opcode.ACKNOWLEDGE_OUTPUT_REPORT_RETURN_FUNCTION_RESULT, 0x22),
            # データレポート Opcodes
            (opcode.DATA_30, 0x30),
            (opcode.DATA_31, 0x31),
            (opcode.DATA_32, 0x32),
            (opcode.DATA_33, 0x33),
            (opcode.DATA_34, 0x34),
            (opcode.DATA_35, 0x35),
            (opcode.DATA_36, 0x36),
            (opcode.DATA_37, 0x37),
            (opcode.DATA_3d, 0x3D),
            (opcode.DATA_3e, 0x3E),
            (opcode.DATA_3f, 0x3F),
        ],
    )
    def test_opcode_values(self, opcode: opcode, expected_value: int) -> None:
        assert opcode.value == expected_value


@pytest.mark.logic
class TestWiimoteLogic:
    def test_basic_buttons(
        self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]
    ) -> None:
        sim_device, wiimote = sim_wiimote
        sim_device.set_buttons(buttons.BUTTON_A.value | buttons.BUTTON_PLUS.value)
        wiimote.update()
        assert wiimote.is_pressed(buttons.BUTTON_A.value)
        assert wiimote.is_pressed(buttons.BUTTON_PLUS.value)
        assert not wiimote.is_pressed(buttons.BUTTON_B.value)

    def test_accel_and_gforce(
        self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]
    ) -> None:
        sim_device, wiimote = sim_wiimote
        # 512 is 0G, 612 is 1G by default calibration in simulator
        sim_device.set_accel(512, 512, 612)
        wiimote.update()
        gx, gy, gz = wiimote.gforce
        assert gx == pytest.approx(0.0)
        assert gy == pytest.approx(0.0)
        assert gz == pytest.approx(1.0)
        assert wiimote.accel == (512, 512, 612)

    def test_rumble(self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]) -> None:
        sim_device, wiimote = sim_wiimote
        wiimote.rumble(True)
        assert sim_device.rumble is True
        wiimote.rumble(False)
        assert sim_device.rumble is False

    def test_rumble_ms(self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]) -> None:
        sim_device, wiimote = sim_wiimote
        wiimote.rumble_ms(50)
        assert sim_device.rumble is True
        time.sleep(0.1)
        wiimote.update()
        assert sim_device.rumble is False

    def test_player_leds(self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]) -> None:
        sim_device, wiimote = sim_wiimote
        wiimote.player_led(p1=True, p4=True)
        assert sim_device.leds == 0x09  # binary 1001

    def test_status_update(
        self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]
    ) -> None:
        sim_device, wiimote = sim_wiimote
        sim_device.battery = 150
        sim_device.extension_connected = True
        wiimote.request_status()
        wiimote.update()
        assert wiimote.battery == 150
        assert wiimote.extension_connected is True

    def test_reporting_mode_change(
        self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]
    ) -> None:
        sim_device, wiimote = sim_wiimote
        wiimote.set_reporting_mode(opcode.DATA_30.value)
        assert sim_device.reporting_mode == 0x30
        wiimote.set_reporting_mode(opcode.DATA_31.value)
        assert sim_device.reporting_mode == 0x31


@pytest.mark.motionplus
class TestMotionPlus:
    def test_motion_plus_detection(self) -> None:
        sim_device = SimulatedHIDDevice()
        sim_device.open()
        sim_device.set_motion_plus(True)
        wiimote = Wiimote(device=sim_device)
        mp_id = wiimote.detect_motion_plus()
        assert Wiimote.is_motion_plus_id(mp_id)
        assert wiimote.motion_plus_id == mp_id

    def test_motion_plus_inside_detection(self) -> None:
        """Test detection of Wii Remote Plus (Inside) where MP is already active."""
        sim_device = SimulatedHIDDevice()
        sim_device.open()
        # Simulate Inside Wiimote: A6 ID unreadable (zeros), A4 ID is active MP
        sim_device.memory[MOTION_PLUS_ID_ADDRESS : MOTION_PLUS_ID_ADDRESS + 6] = [0] * 6
        sim_device.set_motion_plus_active(True)

        wiimote = Wiimote(device=sim_device)
        mp_id = wiimote.detect_motion_plus()
        assert Wiimote.is_motion_plus_active_id(mp_id)
        assert wiimote.motion_plus_activated is True

    def test_motion_plus_activation(self) -> None:
        sim_device = SimulatedHIDDevice()
        sim_device.open()
        sim_device.set_motion_plus(True)
        wiimote = Wiimote(device=sim_device, require_motion_plus=True)
        assert wiimote.motion_plus_activated is True
        assert sim_device.reporting_mode == 0x35
        assert sim_device.extension_connected is True

    def test_motion_plus_data_roundtrip(self) -> None:
        sim_device = SimulatedHIDDevice()
        sim_device.open()
        sim_device.set_motion_plus(True)
        wiimote = Wiimote(device=sim_device, require_motion_plus=True)

        sim_device.set_gyro(
            0x1234, 0x2345, 0x3456, yaw_slow=False, roll_slow=True, pitch_slow=False
        )
        wiimote.update()

        assert wiimote.gyro_raw["yaw"] == 0x1234
        assert wiimote.gyro_raw["roll"] == 0x2345
        assert wiimote.gyro_raw["pitch"] == 0x3456
        assert wiimote.gyro_slow["yaw"] is False
        assert wiimote.gyro_slow["roll"] is True
        assert wiimote.gyro_slow["pitch"] is False


@pytest.mark.ir
class TestIR:
    def test_ir_extended_parsing(
        self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]
    ) -> None:
        sim_device, wiimote = sim_wiimote
        wiimote.set_reporting_mode(opcode.DATA_33.value)
        sim_device.ir = [(100, 200), (300, 400), (500, 600), (700, 800)]
        wiimote.update()
        assert wiimote.ir == [(100, 200), (300, 400), (500, 600), (700, 800)]

    def test_ir_basic_parsing(
        self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]
    ) -> None:
        sim_device, wiimote = sim_wiimote
        wiimote.set_reporting_mode(opcode.DATA_36.value)
        sim_device.ir = [(100, 200), (300, 400), (500, 600), (700, 800)]
        wiimote.update()
        # Basic mode only reports first 2 dots
        assert wiimote.ir[0] == (100, 200)
        assert wiimote.ir[1] == (300, 400)

    def test_init_ir(self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]) -> None:
        sim_device, wiimote = sim_wiimote
        wiimote.init_ir(mode=3, sensitivity=5)
        # Verify mode was written to 0xB00033
        assert sim_device.memory[0xB00033] == 3
        # Verify block1 for sensitivity 5 was written to 0xB00000
        block1, _ = IR_SENSITIVITY_BLOCKS[5]
        assert list(sim_device.memory[0xB00000 : 0xB00000 + len(block1)]) == block1


@pytest.mark.logic
class TestHardwareInit:
    def test_init_speaker(
        self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]
    ) -> None:
        sim_device, wiimote = sim_wiimote
        wiimote.init_speaker()
        # Check some key registers written during speaker init
        assert sim_device.memory[0xA20009] == 0x01
        assert sim_device.memory[0xA20008] == 0x01


@pytest.mark.memory
class TestMemory:
    def test_read_write_register(
        self, sim_wiimote: tuple[SimulatedHIDDevice, Wiimote]
    ) -> None:
        sim_device, wiimote = sim_wiimote
        wiimote.write_register(0xA20000, [0xDE, 0xAD, 0xBE, 0xEF])
        assert sim_device.memory[0xA20000:0xA20004] == bytearray(
            [0xDE, 0xAD, 0xBE, 0xEF]
        )

        data = wiimote._read_memory_block(0xA20000, 4)
        assert data == b"\xde\xad\xbe\xef"
