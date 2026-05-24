import time
import hid

from typing import Optional
from enum import Enum

# Vendor ID (Nintendo)
VENDOR_ID = 0x057E

# Nintendo RVL-CNT-01
PRODUCT_ID = 0x0306

# Nintendo RVL-CNT-01-TR
PRODUCT_ID_TR = 0x0330


# Buttons bitmask
class buttons(Enum):
    BUTTON_LEFT = 0x0100
    BUTTON_RIGHT = 0x0200
    BUTTON_DOWN = 0x0400
    BUTTON_UP = 0x0800
    BUTTON_PLUS = 0x1000

    BUTTON_TWO = 0x0001
    BUTTON_ONE = 0x0002
    BUTTON_B = 0x0004
    BUTTON_A = 0x0008
    BUTTON_MINUS = 0x0010
    BUTTON_HOME = 0x0080


class opcode(Enum):
    # Output report
    RUMBLE = 0x10
    PLAYER_LEDS = 0x11
    DATA_REPORTING_MODE = 0x12
    IR_CAMERA_ENABLE = 0x13
    SPEAKER_ENABLE = 0x14
    STATUS_INFORMATION_REQUEST = 0x15
    WRITE_MEMORY_AND_REGISTERS = 0x16
    READ_MEMORY_AND_REGISTERS = 0x17
    SPEAKER_DATA = 0x18
    SPEAKER_MUTE = 0x19
    IR_CAMERA_ENABLE_2 = 0x1A

    # Input report
    STATUS_INFORMATION = 0x20
    READ_MEMORY_AND_REGISTERS_DATA = 0x21
    ACKNOWLEDGE_OUTPUT_REPORT_RETURN_FUNCTION_RESULT = 0x22

    # Data reports
    DATA_30 = 0x30
    DATA_31 = 0x31
    DATA_32 = 0x32
    DATA_33 = 0x33
    DATA_34 = 0x34
    DATA_35 = 0x35
    DATA_36 = 0x36
    DATA_37 = 0x37
    DATA_3d = 0x3D
    DATA_3e = 0x3E
    DATA_3f = 0x3F


IR_SENSITIVITY_BLOCKS = {
    1: ([0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0x64, 0x00, 0xFE], [0xFD, 0x05]),
    2: ([0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0x96, 0x00, 0xB4], [0xB3, 0x04]),
    3: ([0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0xAA, 0x00, 0x64], [0x63, 0x03]),
    4: ([0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0xC8, 0x00, 0x36], [0x35, 0x03]),
    5: ([0x07, 0x00, 0x00, 0x71, 0x01, 0x00, 0x72, 0x00, 0x20], [0x1F, 0x03]),
}

EXTENSION_INIT_ENABLE_ADDRESS = 0xA400F0
EXTENSION_INIT_DISABLE_ENCRYPTION_ADDRESS = 0xA400FB
EXTENSION_ID_ADDRESS = 0xA400FA
MOTION_PLUS_INIT_ADDRESS = 0xA600F0
MOTION_PLUS_ID_ADDRESS = 0xA600FA
MOTION_PLUS_ACTIVATE_ADDRESS = 0xA600FE
MOTION_PLUS_ID_PREFIX = bytes([0x00, 0x00, 0xA6, 0x20])
MOTION_PLUS_ID_PREFIX_TR = bytes([0x01, 0x00, 0xA6, 0x20])
MOTION_PLUS_ACTIVE_ID = bytes([0x00, 0x00, 0xA4, 0x20, 0x04, 0x05])
MOTION_PLUS_ACTIVE_ID_TR = bytes([0x01, 0x00, 0xA4, 0x20, 0x04, 0x05])
MOTION_PLUS_INACTIVE_ID = bytes([0x00, 0x00, 0xA6, 0x20, 0x00, 0x05])
MOTION_PLUS_ACTIVE_REPORT_LENGTH = 6


class Lowlevel_Wiimote:
    def __init__(
        self, target: Optional[bytes | str] = None, device: Optional[hid.device] = None
    ) -> None:
        if device is not None:
            self.device = device
        else:
            self.device = hid.device()

        if target is not None:
            if isinstance(target, bytes):
                self.device.open_path(target)
            elif isinstance(target, str):
                self.device.open(serial_number=target)

        self.is_rumble = False
        self.buttons = 0
        self.accel = (0, 0, 0)
        self.ir: list[Optional[tuple[int, int]]] = [None] * 4
        self.battery = 0
        self.leds = 0
        self.extension = b""
        self.extension_connected = False
        self.memory_data: dict[int, list[int]] = {}

        # For interleaved mode (0x3e/0x3f)
        self._interleaved_buffer: dict[int, Optional[list[int]]] = {
            0x3E: None,
            0x3F: None,
        }

        # Set to True when read() updates self.extension from a fresh report
        self._extension_fresh = False

    def close(self) -> None:
        self.device.close()

    def _send_report(self, report_id: int, *payload: int) -> None:
        data = [report_id] + list(payload)
        self.device.write(data)

    def rumble(self, is_rumble: bool) -> None:
        self.is_rumble = is_rumble
        rumble_arg = 0x01 if self.is_rumble else 0x00
        self._send_report(opcode.RUMBLE.value, rumble_arg)

    def player_led(
        self, p1: bool = False, p2: bool = False, p3: bool = False, p4: bool = False
    ) -> None:
        led_arg = 0x00
        if p1:
            led_arg |= 0x10
        if p2:
            led_arg |= 0x20
        if p3:
            led_arg |= 0x40
        if p4:
            led_arg |= 0x80
        led_arg |= 0x01 if self.is_rumble else 0x00
        self._send_report(opcode.PLAYER_LEDS.value, led_arg)
        self.leds = led_arg >> 4

    def set_reporting_mode(self, mode: int, continuous: bool = False) -> None:
        tt = 0x04 if continuous else 0x00
        tt |= 0x01 if self.is_rumble else 0x00
        self._send_report(opcode.DATA_REPORTING_MODE.value, tt, mode)

    def request_status(self) -> None:
        self._send_report(
            opcode.STATUS_INFORMATION_REQUEST.value, (0x01 if self.is_rumble else 0x00)
        )

    def write_register(self, address: int, data: int | bytes | list[int]) -> None:
        if isinstance(data, int):
            data = [data]
        rumble_arg = 0x01 if self.is_rumble else 0x00
        mm = 0x04 | rumble_arg
        for i in range(0, len(data), 16):
            chunk = data[i : i + 16]
            size = len(chunk)
            payload = list(chunk) + [0] * (16 - size)
            addr = address + i
            f1, f2, f3 = (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF
            self.device.write(
                [opcode.WRITE_MEMORY_AND_REGISTERS.value, mm, f1, f2, f3, size]
                + payload
            )
            time.sleep(0.05)

    def read_memory(self, address: int, size: int) -> None:
        rumble_arg = 0x01 if self.is_rumble else 0x00
        mm = rumble_arg
        if address >= 0xA20000:
            mm |= 0x04
        f1, f2, f3 = (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF
        s1, s2 = (size >> 8) & 0xFF, size & 0xFF
        self._send_report(
            opcode.READ_MEMORY_AND_REGISTERS.value, mm, f1, f2, f3, s1, s2
        )

    def init_ir(self, mode: int = 3, sensitivity: int = 3) -> None:
        rumble_arg = 0x01 if self.is_rumble else 0x00
        self._send_report(opcode.IR_CAMERA_ENABLE.value, 0x04 | rumble_arg)
        self._send_report(opcode.IR_CAMERA_ENABLE_2.value, 0x04 | rumble_arg)
        self.write_register(0xB00030, 0x08)
        block1, block2 = IR_SENSITIVITY_BLOCKS[sensitivity]
        self.write_register(0xB00000, block1)
        self.write_register(0xB0001A, block2)
        self.write_register(0xB00033, mode)
        self.write_register(0xB00030, 0x08)

    def init_speaker(self) -> None:
        rumble_arg = 0x01 if self.is_rumble else 0x00
        self._send_report(opcode.SPEAKER_ENABLE.value, 0x04 | rumble_arg)
        self._send_report(opcode.SPEAKER_MUTE.value, 0x04 | rumble_arg)
        self.write_register(0xA20009, 0x01)
        self.write_register(0xA20001, 0x08)
        self.write_register(0xA20001, [0x00, 0x00, 0xD0, 0x07, 0x40, 0x00, 0x00])
        self.write_register(0xA20008, 0x01)
        self._send_report(opcode.SPEAKER_MUTE.value, 0x00 | rumble_arg)

    def read(self, timeout: int = 0) -> Optional[int]:
        data = self.device.read(64, timeout)
        if not data:
            return None
        self._extension_fresh = False
        report_id = data[0]

        # All reports except 0x3d contain button data in bytes 1-2
        if report_id != opcode.DATA_3d.value:
            self.buttons = (data[1] << 8) | data[2]

        if report_id == opcode.STATUS_INFORMATION.value:
            self.leds = (data[3] >> 4) & 0x0F
            self.extension_connected = bool(data[3] & 0x02)
            self.battery = data[6]
        elif report_id == opcode.READ_MEMORY_AND_REGISTERS_DATA.value:
            size = (data[3] >> 4) + 1
            addr = (data[4] << 8) | data[5]
            self.memory_data[addr] = list(data[6 : 6 + size])
        elif report_id == opcode.DATA_30.value:
            pass
        elif report_id == opcode.DATA_31.value:
            self._parse_accel(data[3:6])
        elif report_id == opcode.DATA_32.value:
            self.extension = bytes(data[3:11])
            self._extension_fresh = True
        elif report_id == opcode.DATA_33.value:
            self._parse_accel(data[3:6])
            self._parse_ir_extended(data[6:18])
        elif report_id == opcode.DATA_34.value:
            self.extension = bytes(data[3:22])
            self._extension_fresh = True
        elif report_id == opcode.DATA_35.value:
            self._parse_accel(data[3:6])
            self.extension = bytes(data[6:22])
            self._extension_fresh = True
        elif report_id == opcode.DATA_36.value:
            self._parse_ir_basic(data[3:13])
            self.extension = bytes(data[13:22])
            self._extension_fresh = True
        elif report_id == opcode.DATA_37.value:
            self._parse_accel(data[3:6])
            self._parse_ir_basic(data[6:16])
            self.extension = bytes(data[16:22])
            self._extension_fresh = True
        elif report_id == opcode.DATA_3d.value:
            self.extension = bytes(data[1:22])
            self._extension_fresh = True
        elif report_id in (opcode.DATA_3e.value, opcode.DATA_3f.value):
            self._handle_interleaved(report_id, data)

        return report_id

    def _parse_accel(self, data: list[int]) -> None:
        xx, yy, zz = data
        # report[1] bits 6-5 are X<1:0>
        x_lsb = (self.buttons >> 13) & 0x03
        # report[2] bit 6 is Z<1>, bit 5 is Y<1>
        y_lsb = (self.buttons >> 4) & 0x02  # Bit 5 of report[2] shifted to bit 1
        z_lsb = (self.buttons >> 5) & 0x02  # Bit 6 of report[2] shifted to bit 1
        self.accel = ((xx << 2) | x_lsb, (yy << 2) | y_lsb, (zz << 2) | z_lsb)

    def _parse_ir_basic(self, data: list[int]) -> None:
        for i in range(2):
            base = i * 5
            b = data[base : base + 5]
            x1 = ((b[2] >> 0) & 0x03) << 8 | b[0]
            y1 = ((b[2] >> 2) & 0x03) << 8 | b[1]
            x2 = ((b[2] >> 4) & 0x03) << 8 | b[3]
            y2 = ((b[2] >> 6) & 0x03) << 8 | b[4]
            self.ir[i * 2] = (x1, y1) if x1 < 1023 or y1 < 1023 else None
            self.ir[i * 2 + 1] = (x2, y2) if x2 < 1023 or y2 < 1023 else None

    def _parse_ir_extended(self, data: list[int]) -> None:
        for i in range(4):
            base = i * 3
            b = data[base : base + 3]
            x = ((b[2] >> 4) & 0x03) << 8 | b[0]
            y = ((b[2] >> 6) & 0x03) << 8 | b[1]
            self.ir[i] = (x, y) if x < 1023 or y < 1023 else None

    def _handle_interleaved(self, report_id: int, data: list[int]) -> None:
        self._interleaved_buffer[report_id] = list(data)
        if self._interleaved_buffer[0x3E] and self._interleaved_buffer[0x3F]:
            d3e = self._interleaved_buffer[0x3E]
            d3f = self._interleaved_buffer[0x3F]

            # Accelerometer in Interleaved mode
            xx, yy = d3e[3], d3f[3]
            z_bits = (
                ((d3e[1] >> 5) & 0x03) << 4
                | ((d3e[2] >> 5) & 0x03) << 6
                | ((d3f[1] >> 5) & 0x03) << 0
                | ((d3f[2] >> 5) & 0x03) << 2
            )
            self.accel = (xx << 2, yy << 2, z_bits << 2)

            # IR Full mode (36 bytes total, 9 per object)
            full_ir_data = d3e[4:22] + d3f[4:22]
            for i in range(4):
                base = i * 9
                b = full_ir_data[base : base + 3]
                x = ((b[2] >> 4) & 0x03) << 8 | b[0]
                y = ((b[2] >> 6) & 0x03) << 8 | b[1]
                self.ir[i] = (x, y) if x < 1023 or y < 1023 else None

            self._interleaved_buffer = {0x3E: None, 0x3F: None}


class Wiimote(Lowlevel_Wiimote):
    def __init__(
        self,
        target: Optional[str | bytes] = None,
        require_motion_plus: bool = False,
        device: Optional[hid.device] = None,
    ) -> None:
        detected_product_id: Optional[int] = None
        if device is None and target is None:
            # Find first Wiimote
            devices = hid.enumerate(VENDOR_ID)
            matches = [
                d for d in devices if d["product_id"] in (PRODUCT_ID, PRODUCT_ID_TR)
            ]
            if not matches:
                raise ConnectionError("No Wiimote found")
            target = matches[0]["path"]
            detected_product_id = matches[0]["product_id"]
        elif device is not None:
            detected_product_id = PRODUCT_ID

        super().__init__(target, device=device)
        self.product_id = detected_product_id
        self.cal_zero = (512, 512, 512)
        self.cal_1g = (612, 612, 612)
        self.rumble_end_time: Optional[float] = None
        self.motion_plus_id = b""
        self.motion_plus_activated = False
        self.gyro_raw = {"yaw": 0, "roll": 0, "pitch": 0}
        self.gyro_slow = {"yaw": True, "roll": True, "pitch": True}
        self.gyro_signs = {"yaw": 1.0, "roll": 1.0, "pitch": 1.0}
        self.motion_plus_extension_connected = False

        # Initial status and mode
        self.request_status()
        self.set_reporting_mode(opcode.DATA_31.value)  # Buttons + Accel

        # Load calibration
        self._load_calibration()
        if require_motion_plus:
            self.require_motion_plus()

    @staticmethod
    def is_motion_plus_id(extension_id: bytes) -> bool:
        return len(extension_id) == 6 and extension_id[:4] == MOTION_PLUS_ID_PREFIX

    @staticmethod
    def is_motion_plus_active_id(extension_id: bytes) -> bool:
        return (
            extension_id == MOTION_PLUS_ACTIVE_ID
            or extension_id == MOTION_PLUS_ACTIVE_ID_TR
        )

    @staticmethod
    def is_zero_extension_id(extension_id: bytes) -> bool:
        return bool(extension_id) and all(value == 0x00 for value in extension_id)

    @staticmethod
    def decode_motion_plus_report(data: bytes) -> dict[str, int | bool]:
        if len(data) < MOTION_PLUS_ACTIVE_REPORT_LENGTH:
            raise ValueError("MotionPlus report must contain at least 6 bytes")

        yaw = data[0] | ((data[3] & 0xFC) << 6)
        roll = data[1] | ((data[4] & 0xFC) << 6)
        pitch = data[2] | ((data[5] & 0xFC) << 6)
        return {
            "yaw": yaw,
            "roll": roll,
            "pitch": pitch,
            "yaw_slow": bool(data[3] & 0x02),
            "pitch_slow": bool(data[3] & 0x01),
            "roll_slow": bool(data[4] & 0x02),
            "extension_connected": bool(data[4] & 0x01),
        }

    @staticmethod
    def is_plausible_motion_plus_report(data: bytes) -> bool:
        if len(data) < MOTION_PLUS_ACTIVE_REPORT_LENGTH:
            return False
        if not (data[5] & 0x02):
            return False

        decoded = Wiimote.decode_motion_plus_report(
            data[:MOTION_PLUS_ACTIVE_REPORT_LENGTH]
        )
        raw_values = [int(decoded["yaw"]), int(decoded["roll"]), int(decoded["pitch"])]
        if all(value == 0 for value in raw_values):
            return False
        if all(value == 0x3FFF for value in raw_values):
            return False
        return True

    def _wait_for_status(self, timeout_s: float = 1.0) -> bool:
        start_time = time.time()
        while time.time() - start_time < timeout_s:
            rid = self.read(50)
            if rid == opcode.STATUS_INFORMATION.value:
                return True
            time.sleep(0.01)
        return False

    def _read_memory_block(
        self, address: int, size: int, timeout_s: float = 1.0
    ) -> bytes:
        self.read_memory(address, size)
        low_addr = address & 0xFFFF
        start_time = time.time()
        while time.time() - start_time < timeout_s:
            rid = self.read(50)
            if (
                rid == opcode.READ_MEMORY_AND_REGISTERS_DATA.value
                and low_addr in self.memory_data
            ):
                data = self.memory_data.pop(low_addr)
                return bytes(data[:size])
            time.sleep(0.01)
        return b""

    def _initialize_extension_port(self) -> None:
        self.write_register(EXTENSION_INIT_ENABLE_ADDRESS, 0x55)
        self.write_register(EXTENSION_INIT_DISABLE_ENCRYPTION_ADDRESS, 0x00)

    def _initialize_motion_plus(self) -> None:
        self.write_register(MOTION_PLUS_INIT_ADDRESS, 0x55)

    def _probe_motion_plus_stream(self, timeout_s: float = 1.0) -> bool:
        self.set_reporting_mode(opcode.DATA_35.value, continuous=True)
        start_time = time.time()
        while time.time() - start_time < timeout_s:
            rid = self.read(50)
            if rid == opcode.DATA_35.value and self.is_plausible_motion_plus_report(
                self.extension
            ):
                return True
            time.sleep(0.005)
        return False

    def activate_motion_plus(self) -> bytes:
        self.write_register(MOTION_PLUS_ACTIVATE_ADDRESS, 0x04)
        time.sleep(0.1)  # Wait for activation to process
        active_id = self._read_memory_block(EXTENSION_ID_ADDRESS, 6)
        # ID check
        self.motion_plus_activated = self.is_motion_plus_active_id(active_id)

        # Stream check
        if self._probe_motion_plus_stream(timeout_s=1.5):
            self.motion_plus_activated = True

        return active_id

    def detect_motion_plus(self) -> bytes:
        self.request_status()
        self._wait_for_status()
        self._initialize_extension_port()
        self._initialize_motion_plus()

        motion_plus_id = self._read_memory_block(MOTION_PLUS_ID_ADDRESS, 6)
        if self.is_motion_plus_id(motion_plus_id):
            self.motion_plus_id = motion_plus_id
            return motion_plus_id

        # Wii Remote Plus / "Wii MotionPlus INSIDE" devices can present the
        # gyroscope as already active on the standard extension bus, so the
        # inactive A600 ID may be unreadable or all-zero.
        active_id = self._read_memory_block(EXTENSION_ID_ADDRESS, 6)
        if self.is_motion_plus_active_id(active_id):
            self.motion_plus_id = active_id
            self.motion_plus_activated = True
            return active_id

        # Fall back to the normal extension ID space so callers can distinguish
        # "non-MotionPlus extension connected" from "nothing readable".
        self.motion_plus_id = active_id
        return self.motion_plus_id

    def require_motion_plus(self) -> bytes:
        extension_id = self.detect_motion_plus()

        # MotionPlus ID か、Wii Remote Plus (Inside) で 0 が返ってきた場合にアクティベーションを試行
        is_mp_candidate = (
            self.is_motion_plus_id(extension_id)
            or self.is_motion_plus_active_id(extension_id)
            or self.is_zero_extension_id(extension_id)
        )

        if not is_mp_candidate:
            self.close()
            if extension_id:
                raise ConnectionError(
                    f"Wii MotionPlus is required, but extension id was {extension_id.hex()}"
                )
            raise ConnectionError(
                "Wii MotionPlus is required, but no supported extension was found"
            )

        if self.motion_plus_activated:
            self.set_reporting_mode(opcode.DATA_35.value, continuous=True)
            return extension_id

        # アクティベーション実行
        active_id = self.activate_motion_plus()

        if not self.motion_plus_activated:
            self.close()
            raise ConnectionError(
                f"Wii MotionPlus activation failed; data stream not detected. (ID: {active_id.hex()})"
            )

        self.set_reporting_mode(opcode.DATA_35.value, continuous=True)
        return extension_id

    def _load_calibration(self) -> None:
        # Read calibration from EEPROM (0x0016)
        # 0x16: X0, 0x17: Y0, 0x18: Z0
        # 0x19: LSBs for 0G (--XXYYZZ)
        # 0x1A: XG, 0x1B: YG, 0x1C: ZG
        # 0x1D: LSBs for 1G (--XXYYZZ)
        self.read_memory(0x16, 10)

        start_time = time.time()
        while time.time() - start_time < 2.0:
            rid = self.read(10)
            if rid == opcode.READ_MEMORY_AND_REGISTERS_DATA.value:
                if 0x16 in self.memory_data:
                    data = self.memory_data[0x16]
                    if len(data) >= 8:
                        self.cal_zero = (
                            (data[0] << 2) | ((data[3] >> 4) & 0x03),
                            (data[1] << 2) | ((data[3] >> 2) & 0x03),
                            (data[2] << 2) | ((data[3] >> 0) & 0x03),
                        )
                        self.cal_1g = (
                            (data[4] << 2) | ((data[7] >> 4) & 0x03),
                            (data[5] << 2) | ((data[7] >> 2) & 0x03),
                            (data[6] << 2) | ((data[7] >> 0) & 0x03),
                        )
                        break
            time.sleep(0.01)

    def rumble_ms(self, duration_ms: int) -> None:
        if duration_ms <= 0:
            self.rumble(False)
            self.rumble_end_time = None
        else:
            self.rumble(True)
            self.rumble_end_time = time.time() + (duration_ms / 1000.0)

    def update(self, timeout: int = 0) -> Optional[int]:
        rid = self.read(timeout)
        if self.motion_plus_activated and self._extension_fresh:
            if len(self.extension) >= MOTION_PLUS_ACTIVE_REPORT_LENGTH:
                decoded = self.decode_motion_plus_report(
                    self.extension[:MOTION_PLUS_ACTIVE_REPORT_LENGTH]
                )
                self.gyro_raw["yaw"] = int(decoded["yaw"])
                self.gyro_raw["roll"] = int(decoded["roll"])
                self.gyro_raw["pitch"] = int(decoded["pitch"])
                self.gyro_slow["yaw"] = bool(decoded["yaw_slow"])
                self.gyro_slow["roll"] = bool(decoded["roll_slow"])
                self.gyro_slow["pitch"] = bool(decoded["pitch_slow"])
                self.motion_plus_extension_connected = bool(
                    decoded["extension_connected"]
                )
                self._extension_fresh = False
        if self.rumble_end_time is not None:
            if time.time() >= self.rumble_end_time:
                self.rumble(False)
                self.rumble_end_time = None
        return rid

    @property
    def gforce(self) -> tuple[float, float, float]:
        x, y, z = self.accel

        def to_g(val: int, zero: int, one_g: int) -> float:
            denom = one_g - zero
            if denom == 0:
                return 0.0
            return (val - zero) / denom

        return (
            to_g(x, self.cal_zero[0], self.cal_1g[0]),
            to_g(y, self.cal_zero[1], self.cal_1g[1]),
            to_g(z, self.cal_zero[2], self.cal_1g[2]),
        )

    def is_pressed(self, mask: buttons | int) -> bool:
        if isinstance(mask, int):
            return bool(self.buttons & mask)
        return bool(self.buttons & mask.value)
