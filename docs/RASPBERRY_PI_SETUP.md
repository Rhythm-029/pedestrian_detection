# Raspberry Pi Setup Guide
*Pedestrian Detection Vehicle — Pi 4 + Camera V2 + YOLOv8n*

## Hardware Requirements

- Raspberry Pi 4 (2GB+ RAM recommended)
- Raspberry Pi Camera Module V2 (IMX219)
- microSD card with Raspberry Pi OS (Bullseye or Bookworm)
- 5V ≥ 3A regulated power supply for Pi

## 1. Camera Setup

Connect Camera V2 to the Pi CSI ribbon port:
1. Power off Pi.
2. Lift the CSI connector latch.
3. Insert ribbon cable (contacts facing away from USB ports).
4. Close latch. Power on.
5. Verify: `libcamera-hello --timeout 5000` or `rpicam-hello`

## 2. Python Environment

```bash
cd ~/pedestrian_detection
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
pip install picamera2        # Pi camera support
pip install opencv-python    # Optional: webcam fallback
```

For YOLO model:
```bash
pip install ultralytics      # for .pt model format
# OR: onnxruntime            # for .onnx (faster on Pi CPU)
pip install onnxruntime
```

## 3. YOLO Model Weights

The model weights are **not committed to the repository** (too large).

### Option A — ONNX Runtime (recommended for Pi 4 CPU)

```bash
# Download YOLOv8n ONNX export
pip install ultralytics
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx', imgsz=320)"
# This creates yolov8n.onnx in the current directory
mv yolov8n.onnx data/models/yolov8n.onnx
```

### Option B — PyTorch (.pt format)

```bash
pip install ultralytics
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
# Model auto-downloads to ~/.cache/ultralytics/
```

## 4. Run the Pi Controller

```bash
# Console-only mode (no UART — for testing)
python3 src/raspberry_pi/pi_controller.py --camera picamera2 --model data/models/yolov8n.onnx

# With UART to ESP32
python3 src/raspberry_pi/pi_controller.py \
    --camera picamera2 \
    --model data/models/yolov8n.onnx \
    --port /dev/serial0 \
    --baud 115200 \
    --fps 10

# Mock camera (for software-only testing)
python3 src/raspberry_pi/pi_controller.py --camera mock --fps 5 --iterations 20
```

## 5. UART Setup on Pi

Enable UART in `/boot/config.txt`:
```
enable_uart=1
```

Disable Bluetooth UART conflict (Pi 4):
```
dtoverlay=disable-bt
```

Reboot. Then verify: `ls /dev/serial0` should exist.

## 6. Run Tests

```bash
python3 -m pytest tests/ -v
```

Expected: **60 passed**

## 7. Camera Distance Calibration

Before claiming distance accuracy, re-calibrate k for your mounted camera:
```bash
python3 scripts/calibrate_camera.py --save
```
Take measurements at known distances (1m, 1.5m, 2m, 2.5m, 3m) with a person in frame.
