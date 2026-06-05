# Weeee

A Python library for high-performance Wiimote communication with built-in MotionPlus IMU fusion.

## Features

- **Easy connection** — auto-detect and connect to any Wiimote / Wiimote Plus (TR) via HID
- **Full button input** — all buttons with `is_pressed()`, bitmask-ready
- **LED & Rumble** — per-LED control, timed vibration
- **IR Camera** — multi-mode tracking with sensitivity adjustment
- **MotionPlus** — automatic detection, activation, and factory-calibrated gyro data
- **IMU Fusion** — built-in sensor fusion (accelerometer + gyroscope) exposes `yaw_deg`, `pitch_deg`, `roll_deg` directly on the Wiimote object
- **Simulator** — bundled virtual Wiimote for testing without hardware
- **Type-safe** — fully annotated, strict mypy clean
- **Cross-platform** — Windows, macOS, Linux (via `hidapi`)

## Installation

```bash
git clone https://github.com/Tebasaki-North-High-School/Weeee.git
cd Weeee
uv sync
```

Or install directly:

```bash
pip install .
```

## Quick Start

```python
from weeee import Wiimote
import time

wiimote = Wiimote(require_motion_plus=True, enable_fusion=True)
print(f"Connected to {wiimote.serial}: {wiimote.type}")

while True:
    wiimote.update()
    if wiimote.is_pressed("HOME"):
        break
    if wiimote.yaw_deg is not None:
        print(f"yaw={wiimote.yaw_deg:.1f}  pitch={wiimote.pitch_deg:.1f}  roll={wiimote.roll_deg:.1f}")
    time.sleep(0.01)

wiimote.close()
```

## Examples

Stream orientation from the command line:

```bash
python -m weeee.ex.motionplus_console -s <serial>
```

## API Overview

| Class / Module | Description |
|---|---|
| `Wiimote(target, ...)` | High-level Wiimote with buttons, LEDs, rumble, IR, MP, fusion |
| `Lowlevel_Wiimote(target, ...)` | Low-level HID read/write — registers, memory, raw reports |
| `ImuFusion()` | Standalone orientation filter using gyroscope + accelerometer |
| `SimulatedHIDDevice()` | Virtual Wiimote for unit tests |
| `buttons` | Constants: `BUTTON_A`, `BUTTON_B`, `BUTTON_HOME`, … |

### Accessible properties on `Wiimote`

- `buttons`, `accel`, `gyro_raw`
- `yaw`, `pitch`, `roll` (radians, or `None` if MP/fusion inactive)
- `yaw_deg`, `pitch_deg`, `roll_deg` (degrees)
- `is_pressed(name)`, `set_rumble(ms)`, `set_leds(...)`
- `reset_yaw()`, `close()`

## Development

```bash
# Install dev dependencies
uv sync --group dev

# Run tests
uv run pytest

# Type-check
uv run mypy src/weeee/

# Lint
uv run ruff check src/weeee/ tests/
```

## License

MIT — see [LICENSE](LICENSE).
