"""
Weeee: A Python library for interacting with Nintendo Wiimotes.
Provides low-level HID communication, high-level Wiimote logic, MotionPlus support, and IMU fusion.
"""

from .core import Wiimote, buttons
from .imu_fusion import ImuFusion

__all__ = ["Wiimote", "buttons", "ImuFusion"]
