# Weeee 🎮

**Weeee** は、Nintendo Wiimote と通信するための軽量な Python ライブラリです。ボタン、LED、振動、IRカメラ、および MotionPlus (ジャイロスコープ) データを扱うためのハイレベルな抽象化と低レベルな HID アクセスの両方を提供します。

**Weeee** is a lightweight Python library for communicating with Nintendo Wiimotes. It provides both high-level abstractions and low-level HID access to handle buttons, LEDs, rumble, IR cameras, and MotionPlus (Gyroscope) data.

## ✨ Features

- 🔌 **Plug & Play**: 標準的な Wiimote および Wiimote Plus (TR) コントローラーへの簡単な接続。
- 🔘 **Full Button Support**: すべてのボタン状態をリアルタイムで監視。
- 💡 **Peripheral Control**: LED と振動（Rumble）を簡単なコマンドで制御。
- 📡 **IR Camera**: 感度調整が可能なマルチモード IR トラッキング。
- 🔄 **MotionPlus & IMU Fusion**: 加速度計とジャイロスコープを組み合わせた、センサーフュージョンによる安定した 6 軸モーション追跡。
- 🧪 **Hardware Simulator**: 物理的なハードウェアなしで開発とテストが可能な仮想 Wiimote を内蔵。

## 🚀 Installation

このプロジェクトは [uv](https://github.com/astral-sh/uv) を使用して依存関係を管理しています。

```bash
# Clone the repository
git clone https://github.com/Tebasaki-North-High-School/Weeee.git
cd Weeee

# Install dependencies
uv sync
```



```bash
pip install .
```

## 📖 Quick Start

```python
from weeee.wiimote import Wiimote, buttons
import time

# Connect to the first available Wiimote
wiimote = Wiimote()

print("Connected! Press 'A' to rumble, 'Home' to exit.")

try:
    while True:
        wiimote.update()
        
        if wiimote.is_pressed(buttons.BUTTON_A):
            wiimote.set_rumble(True)
        else:
            wiimote.set_rumble(False)
            
        if wiimote.is_pressed(buttons.BUTTON_HOME):
            break
            
        time.sleep(0.01)
finally:
    wiimote.close()
```

## 📜 License

このプロジェクトは MIT ライセンスの下で提供されています。詳細は [LICENSE](LICENSE) ファイルを参照してください。
