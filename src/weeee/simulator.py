import hid

from typing import Optional, List, TypedDict

from .wiimote import (
    opcode,
    VENDOR_ID,
    PRODUCT_ID,
    EXTENSION_ID_ADDRESS,
    EXTENSION_INIT_ENABLE_ADDRESS,
    MOTION_PLUS_ACTIVATE_ADDRESS,
    MOTION_PLUS_ID_ADDRESS,
    MOTION_PLUS_INIT_ADDRESS,
)

class _gyro(TypedDict):
    yaw: int
    pitch: int
    roll: int
    yaw_slow: bool
    pitch_slow: bool
    roll_slow: bool


class SimulatedHIDDevice(hid.device):
    """
    A class that mimics the hid.device interface for simulating a Wiimote.
    """

    def __init__(self) -> None:
        self.is_open = False
        self.path: Optional[bytes] = None
        self.serial_number: Optional[str] = None

        # Wiimote State
        self.rumble = False
        self.leds = 0
        self.reporting_mode = 0x30
        self.continuous_reporting = False
        self.battery = 200  # ~80%
        self.extension_connected = False

        # Memory
        self.memory = bytearray(0x1000000)  # 16MB address space
        # Initialize calibration data at 0x0016
        self.memory[0x16] = 128  # 512 >> 2
        self.memory[0x17] = 128
        self.memory[0x18] = 128
        self.memory[0x19] = 0x00
        self.memory[0x1A] = 153  # 612 >> 2
        self.memory[0x1B] = 153
        self.memory[0x1C] = 153
        self.memory[0x1D] = 0x00

        # Input data
        self.buttons = 0
        self.accel = (512, 512, 512)
        self.ir = [(1023, 1023)] * 4
        self.gyro: _gyro = {
            "yaw": 0x2000,
            "roll": 0x2000,
            "pitch": 0x2000,
            "yaw_slow": True,
            "roll_slow": True,
            "pitch_slow": True,
        }

        # Response queue for pending reports (e.g. status, memory read)
        self.response_queue: List[List[int]] = []

        # Whether a MotionPlus hardware is attached (distinct from activated state)
        self._motion_plus_hardware_present = False

    def set_motion_plus(self, connected: bool = True) -> None:
        self._motion_plus_hardware_present = connected
        if connected:
            # MotionPlus ID at 0xA600FA
            self.memory[MOTION_PLUS_ID_ADDRESS : MOTION_PLUS_ID_ADDRESS + 6] = [
                0x00,
                0x00,
                0xA6,
                0x20,
                0x00,
                0x05,
            ]
        else:
            self.memory[MOTION_PLUS_ID_ADDRESS : MOTION_PLUS_ID_ADDRESS + 6] = [
                0x00
            ] * 6
            self.set_motion_plus_active(False)

    def set_motion_plus_active(self, active: bool = True) -> None:
        if active:
            # Active ID at 0xA400FA
            self.memory[EXTENSION_ID_ADDRESS : EXTENSION_ID_ADDRESS + 6] = [
                0x00,
                0x00,
                0xA4,
                0x20,
                0x04,
                0x05,
            ]
            self.extension_connected = True
        else:
            self.memory[EXTENSION_ID_ADDRESS : EXTENSION_ID_ADDRESS + 6] = [0x00] * 6
            self.extension_connected = False

    def open(
        self,
        vendor_id: int = VENDOR_ID,
        product_id: int = PRODUCT_ID,
        serial_number: Optional[str] = None,
    ) -> None:
        self.is_open = True
        self.serial_number = serial_number

    def open_path(self, device_path: bytes) -> None:
        self.is_open = True
        self.path = device_path

    def close(self) -> None:
        self.is_open = False

    def write(self, data: bytes | List[int]) -> int:
        if not self.is_open:
            raise IOError("Device not open")

        report_id = data[0]
        payload = data[1:]

        if report_id == opcode.RUMBLE.value:
            self.rumble = bool(payload[0] & 0x01)
        elif report_id == opcode.PLAYER_LEDS.value:
            self.leds = (payload[0] >> 4) & 0x0F
            self.rumble = bool(payload[0] & 0x01)
        elif report_id == opcode.DATA_REPORTING_MODE.value:
            self.continuous_reporting = bool(payload[0] & 0x04)
            self.reporting_mode = payload[1]
            self.rumble = bool(payload[0] & 0x01)
        elif report_id == opcode.STATUS_INFORMATION_REQUEST.value:
            self.rumble = bool(payload[0] & 0x01)
            self._enqueue_status_report()
        elif report_id == opcode.WRITE_MEMORY_AND_REGISTERS.value:
            self.rumble = bool(payload[0] & 0x01)
            addr = (payload[1] << 16) | (payload[2] << 8) | payload[3]
            size = payload[4]
            data_to_write = payload[5 : 5 + size]
            for i, b in enumerate(data_to_write):
                if addr + i < len(self.memory):
                    self.memory[addr + i] = b

            if (
                addr == MOTION_PLUS_ACTIVATE_ADDRESS
                and data_to_write
                and data_to_write[0] == 0x04
                and self._motion_plus_hardware_present
            ):
                self.set_motion_plus_active(True)
            elif (
                addr == MOTION_PLUS_INIT_ADDRESS
                and data_to_write
                and data_to_write[0] == 0x55
                and self._motion_plus_hardware_present
            ):
                # Writing 0x55 to 0xA600F0 initialises the I2C bus and makes
                # the MotionPlus ID readable at 0xA600FA.
                self.memory[MOTION_PLUS_ID_ADDRESS : MOTION_PLUS_ID_ADDRESS + 6] = [
                    0x00,
                    0x00,
                    0xA6,
                    0x20,
                    0x00,
                    0x05,
                ]
            elif (
                addr == EXTENSION_INIT_ENABLE_ADDRESS
                and data_to_write
                and data_to_write[0] == 0x55
                and self._motion_plus_hardware_present
            ):
                # Writing 0x55 to 0xA400F0 deactivates external MotionPlus
                self.set_motion_plus_active(False)

        elif report_id == opcode.READ_MEMORY_AND_REGISTERS.value:
            self.rumble = bool(payload[0] & 0x01)
            addr = (payload[1] << 16) | (payload[2] << 8) | payload[3]
            size = (payload[4] << 8) | payload[5]
            self._enqueue_read_report(addr, size)

        return len(data)

    def read(self, max_length: int, timeout_ms: int = 0) -> List[int]:
        if not self.is_open:
            raise IOError("Device not open")

        # Silently drain ACK reports — Lowlevel_Wiimote.read() has no handler
        # for 0x22, so they would otherwise waste reads in every wait loop.
        while (
            self.response_queue
            and self.response_queue[0][0]
            == opcode.ACKNOWLEDGE_OUTPUT_REPORT_RETURN_FUNCTION_RESULT.value
        ):
            self.response_queue.pop(0)

        if self.response_queue:
            return self.response_queue.pop(0)

        # When no data is queued, generate a live data report.
        # The real Wiimote always sends the first report after a mode change,
        # and in continuous mode it keeps streaming.  We do the same here.
        return self._generate_data_report()

    def _enqueue_status_report(self) -> None:
        report = [0] * 7
        report[0] = opcode.STATUS_INFORMATION.value
        report[1] = (self.buttons >> 8) & 0xFF
        report[2] = self.buttons & 0xFF
        # LF byte: L (LEDs) in high nibble, F (Flags) in low nibble
        # bit 1: Extension connected, bit 0: Battery nearly empty
        report[3] = (self.leds << 4) | (0x02 if self.extension_connected else 0x00)
        # Note: self.battery is 0-255. VV is the current battery level.
        report[6] = self.battery
        self.response_queue.append(report)

    def _enqueue_ack(self, report_id: int, error_code: int) -> None:
        report = [0] * 6
        report[0] = opcode.ACKNOWLEDGE_OUTPUT_REPORT_RETURN_FUNCTION_RESULT.value
        report[1] = (self.buttons >> 8) & 0xFF
        report[2] = self.buttons & 0xFF
        report[3] = report_id
        report[4] = error_code
        self.response_queue.append(report)

    def _enqueue_read_report(self, addr: int, size: int) -> None:
        remaining = size
        current_addr = addr
        while remaining > 0:
            chunk_size = min(remaining, 16)
            report = [0] * 22
            report[0] = opcode.READ_MEMORY_AND_REGISTERS_DATA.value
            report[1] = (self.buttons >> 8) & 0xFF
            report[2] = self.buttons & 0xFF
            report[3] = (chunk_size - 1) << 4
            report[4] = (current_addr >> 8) & 0xFF
            report[5] = current_addr & 0xFF

            data = self.memory[current_addr : current_addr + chunk_size]
            for i, b in enumerate(data):
                report[6 + i] = b

            self.response_queue.append(report)
            current_addr += chunk_size
            remaining -= chunk_size

    def _generate_data_report(self) -> List[int]:
        report_1 = (self.buttons >> 8) & 0xFF
        report_2 = self.buttons & 0xFF

        # Add Accelerometer LSBs if applicable (except for interleaved mode)
        if self.reporting_mode in (0x31, 0x33, 0x35, 0x37):
            # X<1:0> to bits 6-5 of report[1]
            report_1 |= (self.accel[0] & 0x03) << 5
            # Z<1> to bit 6 of report[2], Y<1> to bit 5 of report[2]
            report_2 |= (self.accel[2] & 0x02) << 5
            report_2 |= (self.accel[1] & 0x02) << 4

        if self.reporting_mode == 0x30:  # Buttons only
            report = [0] * 3
            report[0] = 0x30
            report[1] = report_1
            report[2] = report_2
            return report
        elif self.reporting_mode == 0x31:  # Buttons + Accel
            report = [0] * 6
            report[0] = 0x31
            report[1] = report_1
            report[2] = report_2
            report[3] = (self.accel[0] >> 2) & 0xFF
            report[4] = (self.accel[1] >> 2) & 0xFF
            report[5] = (self.accel[2] >> 2) & 0xFF
            return report
        elif self.reporting_mode == 0x32:  # Buttons + Extension (8 bytes basic)
            report = [0] * 11
            report[0] = 0x32
            report[1] = (self.buttons >> 8) & 0xFF
            report[2] = self.buttons & 0xFF
            if self.extension_connected:
                yaw, roll, pitch = (
                    self.gyro["yaw"],
                    self.gyro["roll"],
                    self.gyro["pitch"],
                )
                report[3] = yaw & 0xFF
                report[4] = roll & 0xFF
                report[5] = pitch & 0xFF
                report[6] = (
                    ((yaw >> 8) << 2)
                    | (0x02 if self.gyro["yaw_slow"] else 0)
                    | (0x01 if self.gyro["pitch_slow"] else 0)
                )
                report[7] = ((roll >> 8) << 2) | (0x02 if self.gyro["roll_slow"] else 0)
                report[8] = ((pitch >> 8) << 2) | 0x02
            return report
        elif self.reporting_mode == 0x33:  # Buttons + Accel + IR Extended
            report = [0] * 18
            report[0] = 0x33
            report[1] = report_1
            report[2] = report_2
            report[3] = (self.accel[0] >> 2) & 0xFF
            report[4] = (self.accel[1] >> 2) & 0xFF
            report[5] = (self.accel[2] >> 2) & 0xFF
            # IR data (12 bytes)
            for i in range(4):
                x, y = self.ir[i]
                base = 6 + i * 3
                report[base] = x & 0xFF
                report[base + 1] = y & 0xFF
                report[base + 2] = ((x >> 8) << 4) | ((y >> 8) << 6)
            return report
        elif self.reporting_mode == 0x34:  # Buttons + Extension (16 bytes full)
            report = [0] * 22
            report[0] = 0x34
            report[1] = (self.buttons >> 8) & 0xFF
            report[2] = self.buttons & 0xFF
            if self.extension_connected:
                yaw, roll, pitch = (
                    self.gyro["yaw"],
                    self.gyro["roll"],
                    self.gyro["pitch"],
                )
                report[3] = yaw & 0xFF
                report[4] = roll & 0xFF
                report[5] = pitch & 0xFF
                report[6] = (
                    ((yaw >> 8) << 2)
                    | (0x02 if self.gyro["yaw_slow"] else 0)
                    | (0x01 if self.gyro["pitch_slow"] else 0)
                )
                report[7] = ((roll >> 8) << 2) | (0x02 if self.gyro["roll_slow"] else 0)
                report[8] = ((pitch >> 8) << 2) | 0x02
            return report
        elif self.reporting_mode == 0x35:  # Buttons + Accel + Extension
            report = [0] * 22
            report[0] = 0x35
            report[1] = report_1
            report[2] = report_2
            report[3] = (self.accel[0] >> 2) & 0xFF
            report[4] = (self.accel[1] >> 2) & 0xFF
            report[5] = (self.accel[2] >> 2) & 0xFF
            # Extension data (16 bytes)
            if self.extension_connected:
                # MotionPlus uses 6 bytes
                yaw, roll, pitch = (
                    self.gyro["yaw"],
                    self.gyro["roll"],
                    self.gyro["pitch"],
                )
                # Byte 0: Yaw 7:0
                report[6] = yaw & 0xFF
                # Byte 1: Roll 7:0
                report[7] = roll & 0xFF
                # Byte 2: Pitch 7:0
                report[8] = pitch & 0xFF
                # Byte 3: Yaw 13:8 (bits 7-2), Yaw slow (bit 1), Pitch slow (bit 0)
                report[9] = (yaw >> 8) << 2
                if self.gyro["yaw_slow"]:
                    report[9] |= 0x02
                if self.gyro["pitch_slow"]:
                    report[9] |= 0x01
                # Byte 4: Roll 13:8 (bits 7-2), Roll slow (bit 1), Ext connected (bit 0)
                report[10] = (roll >> 8) << 2
                if self.gyro["roll_slow"]:
                    report[10] |= 0x02
                # Byte 5: Pitch 13:8 (bits 7-2), 1 (bit 1), 0 (bit 0)
                report[11] = ((pitch >> 8) << 2) | 0x02
            return report
        elif self.reporting_mode == 0x36:  # Buttons + IR (basic, 10) + Extension (9)
            report = [0] * 22
            report[0] = 0x36
            report[1] = (self.buttons >> 8) & 0xFF
            report[2] = self.buttons & 0xFF
            # IR basic: 5 bytes per 2 dots, 10 bytes total
            for i in range(2):
                x1, y1 = self.ir[i * 2]
                x2, y2 = self.ir[i * 2 + 1]
                base = 3 + i * 5
                report[base] = x1 & 0xFF
                report[base + 1] = y1 & 0xFF
                report[base + 2] = (
                    ((x1 >> 8) & 0x03)
                    | (((y1 >> 8) & 0x03) << 2)
                    | (((x2 >> 8) & 0x03) << 4)
                    | (((y2 >> 8) & 0x03) << 6)
                )
                report[base + 3] = x2 & 0xFF
                report[base + 4] = y2 & 0xFF
            # Extension (9 bytes)
            if self.extension_connected:
                yaw, roll, pitch = (
                    self.gyro["yaw"],
                    self.gyro["roll"],
                    self.gyro["pitch"],
                )
                report[13] = yaw & 0xFF
                report[14] = roll & 0xFF
                report[15] = pitch & 0xFF
                # MP extension uses 6 bytes in the 9-byte slot
                report[16] = (
                    ((yaw >> 8) << 2)
                    | (0x02 if self.gyro["yaw_slow"] else 0)
                    | (0x01 if self.gyro["pitch_slow"] else 0)
                )
                report[17] = ((roll >> 8) << 2) | (
                    0x02 if self.gyro["roll_slow"] else 0
                )
                report[18] = ((pitch >> 8) << 2) | 0x02
            return report
        elif (
            self.reporting_mode == 0x37
        ):  # Buttons + Accel + IR (basic, 10) + Extension (6)
            report = [0] * 22
            report[0] = 0x37
            report[1] = report_1
            report[2] = report_2
            report[3] = (self.accel[0] >> 2) & 0xFF
            report[4] = (self.accel[1] >> 2) & 0xFF
            report[5] = (self.accel[2] >> 2) & 0xFF
            # IR basic: 5 bytes per 2 dots, 10 bytes total
            for i in range(2):
                x1, y1 = self.ir[i * 2]
                x2, y2 = self.ir[i * 2 + 1]
                base = 6 + i * 5
                report[base] = x1 & 0xFF
                report[base + 1] = y1 & 0xFF
                report[base + 2] = (
                    ((x1 >> 8) & 0x03)
                    | (((y1 >> 8) & 0x03) << 2)
                    | (((x2 >> 8) & 0x03) << 4)
                    | (((y2 >> 8) & 0x03) << 6)
                )
                report[base + 3] = x2 & 0xFF
                report[base + 4] = y2 & 0xFF
            # Extension (6 bytes)
            if self.extension_connected:
                yaw, roll, pitch = (
                    self.gyro["yaw"],
                    self.gyro["roll"],
                    self.gyro["pitch"],
                )
                report[16] = yaw & 0xFF
                report[17] = roll & 0xFF
                report[18] = pitch & 0xFF
                report[19] = (
                    ((yaw >> 8) << 2)
                    | (0x02 if self.gyro["yaw_slow"] else 0)
                    | (0x01 if self.gyro["pitch_slow"] else 0)
                )
                report[20] = ((roll >> 8) << 2) | (
                    0x02 if self.gyro["roll_slow"] else 0
                )
                report[21] = ((pitch >> 8) << 2) | 0x02
            return report

        # Default fallback (should not be reached)
        report = [0] * 3
        report[0] = 0x30
        report[1] = report_1
        report[2] = report_2
        return report

    def set_buttons(self, button_mask: int) -> None:
        self.buttons = button_mask

    def set_accel(self, x: int, y: int, z: int) -> None:
        self.accel = (x, y, z)

    def set_gyro(
        self,
        yaw: int,
        roll: int,
        pitch: int,
        yaw_slow: bool = True,
        roll_slow: bool = True,
        pitch_slow: bool = True,
    ) -> None:
        self.gyro = {
            "yaw": yaw,
            "roll": roll,
            "pitch": pitch,
            "yaw_slow": yaw_slow,
            "roll_slow": roll_slow,
            "pitch_slow": pitch_slow,
        }
