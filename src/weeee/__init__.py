"""
Weeee: A Python library for interacting with Nintendo Wiimotes.
Provides low-level HID communication, high-level Wiimote logic, MotionPlus support, and IMU fusion.
"""

from .core import Wiimote, buttons, Lowlevel_Wiimote
from .imu_fusion import ImuFusion
from .simulator import SimulatedHIDDevice

__all__ = [
    "Wiimote",
    "Lowlevel_Wiimote",
    "buttons",
    "ImuFusion",
    "SimulatedHIDDevice",
]
