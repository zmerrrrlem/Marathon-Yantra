#!/usr/bin/env python3
"""Interactive Tester for Altair Vision (OpenVINO FL3 model on LifeCam)."""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Ensure altair_vision_deeplearning can be imported
sys.path.insert(0, str(Path(__file__).resolve().parent / "src" / "altair_vision_deeplearning"))

try:
    from altair_vision_deeplearning.openvino_detector import PureOpenVINOSeg
except ImportError as e:
    print(f"Error importing PureOpenVINOSeg: {e}")
    sys.exit(1)

COLOR_BALL = (0, 255, 0)
COLOR_FRIEND = (255, 120, 0)
COLOR_GOAL = (0, 165, 255)

CLASS_NAMES = {0: "ball", 1: "friend", 2: "goal"}


def main():
    parser = argparse.ArgumentParser(description="Test Altair OpenVINO Vision")
    parser.add_argument("--device", type=str, default="/dev/video2", help="Camera device (default: /dev/video2)")
    parser.add_argument("--mode", type=str, choices=["united_soccer", "penalty_kick"], default="penalty_kick")
    parser.add_argument("--model", type=str, default="models/FL3_openvino_int8", help="Model path")
    parser.add_argument("--conf", type=float, default=0.35, help="Confidence threshold")
    args = parser.parse_args()

    dev = int(args.device) if args.device.isdigit() else args.device
    print(f"Opening camera: {dev} [Mode: {args.mode}]...")
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(dev)
        if not cap.isOpened():
            print(f"Error opening camera {args.device}")
            sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    print(f"Loading OpenVINO model from: {args.model} ...")
    model = PureOpenVINOSeg(args.model, imgsz=640, device="CPU")
    print("Model loaded successfully!")
    print("\nControls:\n  'q'/ESC: Exit | '+'/'-': Adjust confidence | 'm': Switch mode | 's': Screenshot\n")

    mode = args.mode
    conf = args.conf
    frame_count = 0
    fps_start = time.perf_counter()
    fps = 0.0

    cv2.namedWindow("Altair Vision Tester", cv2.WINDOW_NORMAL)

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            frame_count += 1
            t0 = time.perf_counter()
            result = model(frame, conf=conf)
            t_infer = (time.perf_counter() - t0) * 1000

            if frame_count % 10 == 0:
                elapsed = time.perf_counter() - fps_start
                fps = 10.0 / elapsed if elapsed > 0 else 0.0
                fps_start = time.perf_counter()

            annotated = frame.copy()
            h, w = frame.shape[:2]

            # Goal mask
            if result.masks and hasattr(result.masks, 'data'):
                for idx, mask_t in result.masks.data.items():
                    mask_np = (mask_t.detach().cpu().numpy() > 0.5).astype(np.uint8)
                    if np.any(mask_np):
                        overlay = np.zeros_like(annotated)
                        overlay[:, :] = (255, 80, 60)
                        annotated[mask_np > 0] = cv2.addWeighted(annotated, 0.65, overlay, 0.35, 0)[mask_np > 0]

            if result.boxes is not None:
                boxes = result.boxes.xyxy.detach().cpu().numpy()
                confs = result.boxes.conf.detach().cpu().numpy()
                classes = result.boxes.cls.detach().cpu().numpy()

                for box, score, cls_id in zip(boxes, confs, classes):
                    cls_id = int(cls_id)
                    if mode == "penalty_kick" and cls_id == 1:
                        continue  # Skip friend robot in penalty kick

                    x1, y1, x2, y2 = map(int, box)
                    label = CLASS_NAMES.get(cls_id, str(cls_id))
                    color = COLOR_BALL if cls_id == 0 else (COLOR_FRIEND if cls_id == 1 else COLOR_GOAL)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(annotated, f"{label} {score:.2f}", (x1, max(15, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            hud = f"MODE: {mode.upper()} | FPS: {fps:.1f} | Latency: {t_infer:.1f}ms | Conf: {conf:.2f}"
            cv2.putText(annotated, hud, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow("Altair Vision Tester", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                break
            elif key in [ord("+"), ord("=")]:
                conf = min(0.95, conf + 0.05)
            elif key == ord("-"):
                conf = max(0.10, conf - 0.05)
            elif key == ord("m"):
                mode = "united_soccer" if mode == "penalty_kick" else "penalty_kick"
            elif key == ord("s"):
                filename = f"altair_snap_{int(time.time())}.jpg"
                cv2.imwrite(filename, annotated)
                print(f"Saved snapshot to {filename}")

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
