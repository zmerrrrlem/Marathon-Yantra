#!/usr/bin/env python3
"""OpenVINO Unified Detector for Yantra Robot (United Soccer & Penalty Kick)."""

from pathlib import Path
from types import SimpleNamespace
from time import perf_counter
import math
import os

import cv2
import numpy as np
import torch
import torch.nn.functional as functional
from torchvision.ops import batched_nms

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class PureOpenVINOSeg:
    """Decode raw xywh/class/mask outputs from FL3 OpenVINO model."""

    def __init__(self, model_path, imgsz=640, device='CPU', num_threads=0):
        import openvino as ov

        xml_files = list(Path(model_path).glob('*.xml'))
        if len(xml_files) != 1:
            raise ValueError(f'model_path ({model_path}) must contain exactly one OpenVINO XML model')
        core = ov.Core()
        model = core.read_model(str(xml_files[0]))
        shape = list(model.input().shape)
        if shape != [1, 3, imgsz, imgsz]:
            raise ValueError(f'Model input is {shape}, requested [1, 3, {imgsz}, {imgsz}].')

        config = {'PERFORMANCE_HINT': 'LATENCY'}
        if num_threads:
            config['INFERENCE_NUM_THREADS'] = num_threads
        self.compiled = core.compile_model(model, device, config)
        self.output0, self.output1 = self.compiled.outputs
        self.imgsz = imgsz
        self.names = {0: 'ball', 1: 'friend', 2: 'goal'}

    def __call__(self, img, conf=0.25):
        h, w = img.shape[:2]
        size = self.imgsz
        ratio = min(size / h, size / w)
        resized = (round(w * ratio), round(h * ratio))
        dw, dh = (size - resized[0]) / 2, (size - resized[1]) / 2
        left, right = round(dw - 0.1), round(dw + 0.1)
        top, bottom = round(dh - 0.1), round(dh + 0.1)

        im = cv2.resize(img, resized, interpolation=cv2.INTER_LINEAR)
        im = cv2.copyMakeBorder(
            im, top, bottom, left, right, cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        tensor = np.ascontiguousarray(im[:, :, ::-1].transpose(2, 0, 1)[None],
                                      dtype=np.float32) / 255.0

        outputs = self.compiled([tensor])
        predictions = torch.from_numpy(outputs[self.output0])[0].T
        scores, labels = predictions[:, 4:7].max(1)
        selected = scores >= conf
        predictions, scores, labels = (
            predictions[selected], scores[selected], labels[selected]
        )
        xywh = predictions[:, :4]
        boxes = torch.cat((xywh[:, :2] - xywh[:, 2:] / 2,
                           xywh[:, :2] + xywh[:, 2:] / 2), dim=1)
        keep = batched_nms(boxes, scores, labels, 0.45)[:300]
        boxes, scores, labels = boxes[keep], scores[keep], labels[keep]
        coefficients = predictions[keep, 7:]

        masks = {}
        goals = torch.where(labels == 2)[0]
        if len(goals):
            index = int(goals[scores[goals].argmax()])
            prototypes = torch.from_numpy(outputs[self.output1])[0]
            nm, mh, mw = prototypes.shape
            logits = (coefficients[index] @ prototypes.reshape(nm, -1))
            mask = functional.interpolate(
                logits.reshape(1, 1, mh, mw), size=(size, size),
                mode='bilinear', align_corners=False,
            )[0, 0]
            mask = mask[top:size - bottom, left:size - right]
            mask = functional.interpolate(
                mask[None, None], size=(h, w), mode='bilinear',
                align_corners=False,
            )[0, 0].sigmoid()
            masks[index] = mask

        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - left) / ratio
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - top) / ratio
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clamp(0, w)
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clamp(0, h)

        for index, mask in masks.items():
            x1, y1, x2, y2 = boxes[index]
            xs = torch.arange(w)[None, :]
            ys = torch.arange(h)[:, None]
            mask *= (xs >= x1) & (xs < x2) & (ys >= y1) & (ys < y2)

        return SimpleNamespace(
            boxes=SimpleNamespace(xyxy=boxes, conf=scores, cls=labels) if len(scores) else None,
            masks=SimpleNamespace(data=masks),
        )


class YantraOpenVINODetector(Node):
    def __init__(self):
        super().__init__('yantra_openvino_detector')

        # Parameters
        self.declare_parameter('model_path', '/home/dika/altair_ws_fix/models/FL3_openvino_int8')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('mode', 'united_soccer')  # 'united_soccer' or 'penalty_kick'
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('ball_conf', 0.50)
        self.declare_parameter('goal_conf', 0.50)
        self.declare_parameter('friend_conf', 0.35)
        self.declare_parameter('cal_const', 10000.0)  # Pixel diameter * distance calibration constant
        self.declare_parameter('publish_annotated', True)

        self.model_path = os.path.expanduser(self.get_parameter('model_path').value)
        self.camera_topic = self.get_parameter('camera_topic').value
        self.mode = self.get_parameter('mode').value.lower()
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.ball_conf = self.get_parameter('ball_conf').value
        self.goal_conf = self.get_parameter('goal_conf').value
        self.friend_conf = self.get_parameter('friend_conf').value
        self.cal_const = self.get_parameter('cal_const').value
        self.publish_annotated = self.get_parameter('publish_annotated').value

        self.get_logger().info(f'Starting Yantra Vision Node [Mode: {self.mode.upper()}]')
        self.get_logger().info(f'Loading model from: {self.model_path}')

        if not os.path.exists(self.model_path):
            fallback = '/home/dika/altair_ws_fix/models/FL3_openvino_int8'
            self.get_logger().warn(f'Model not found at {self.model_path}, trying {fallback}')
            self.model_path = fallback

        self.model = PureOpenVINOSeg(self.model_path, imgsz=640, device='CPU')
        self.get_logger().info('OpenVINO model loaded successfully!')

        self.bridge = CvBridge()

        # QoS
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            sensor_qos,
        )

        # Publishers
        self.annotated_pub = self.create_publisher(Image, '/vision/annotated_frame', sensor_qos)
        self.ball_pos_pub = self.create_publisher(Point, '/vision/ball_position', 10)
        self.goal_pos_pub = self.create_publisher(Point, '/vision/goal_position', 10)
        self.friend_pos_pub = self.create_publisher(Point, '/vision/friend_position', 10)

        # Optional BallInfo publisher if ball_detect msg exists
        self.ball_info_pub = None
        try:
            from ball_detect.msg import BallInfo
            self.ball_info_msg_cls = BallInfo
            self.ball_info_pub = self.create_publisher(BallInfo, 'ball_info', 10)
            self.get_logger().info('Registered ball_info (BallInfo) publisher.')
        except ImportError:
            self.ball_info_pub = None

        self.frame_count = 0
        self.fps_timer = perf_counter()
        self.fps = 0.0

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        self.frame_count += 1
        t0 = perf_counter()

        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0

        # Run inference
        result = self.model(frame, conf=self.conf_threshold)
        t_infer = (perf_counter() - t0) * 1000

        if self.frame_count % 15 == 0:
            elapsed = perf_counter() - self.fps_timer
            self.fps = 15.0 / elapsed if elapsed > 0 else 0.0
            self.fps_timer = perf_counter()

        annotated = frame.copy() if self.publish_annotated else None

        # Goal segmentation overlay
        if annotated is not None and result.masks and hasattr(result.masks, 'data'):
            for idx, mask_t in result.masks.data.items():
                mask_np = (mask_t.detach().cpu().numpy() > 0.5).astype(np.uint8)
                if np.any(mask_np):
                    overlay = np.zeros_like(annotated)
                    overlay[:, :] = (255, 80, 60)
                    annotated[mask_np > 0] = cv2.addWeighted(
                        annotated, 0.65, overlay, 0.35, 0
                    )[mask_np > 0]

        best_ball = None
        best_goal = None
        best_friend = None

        if result.boxes is not None:
            boxes = result.boxes.xyxy.detach().cpu().numpy()
            confs = result.boxes.conf.detach().cpu().numpy()
            classes = result.boxes.cls.detach().cpu().numpy()

            for box, conf, cls_id in zip(boxes, confs, classes):
                cls_id = int(cls_id)
                x1, y1, x2, y2 = map(int, box)

                # Ball
                if cls_id == 0 and conf >= self.ball_conf:
                    if best_ball is None or conf > best_ball['conf']:
                        best_ball = {'box': (x1, y1, x2, y2), 'conf': conf}

                # Friend Robot (United Soccer)
                elif cls_id == 1 and conf >= self.friend_conf:
                    if self.mode == 'united_soccer':
                        if best_friend is None or conf > best_friend['conf']:
                            best_friend = {'box': (x1, y1, x2, y2), 'conf': conf}

                # Goal
                elif cls_id == 2 and conf >= self.goal_conf:
                    if best_goal is None or conf > best_goal['conf']:
                        best_goal = {'box': (x1, y1, x2, y2), 'conf': conf}

                # Draw annotations
                if annotated is not None:
                    color = (0, 255, 0) if cls_id == 0 else ((255, 120, 0) if cls_id == 1 else (0, 165, 255))
                    label = self.model.names.get(cls_id, str(cls_id))
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(
                        annotated, f'{label} {conf:.2f}', (x1, max(15, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA
                    )

        # Publish Ball Position
        if best_ball is not None:
            bx1, by1, bx2, by2 = best_ball['box']
            bcx = (bx1 + bx2) / 2.0
            bcy = (by1 + by2) / 2.0
            diameter = max(bx2 - bx1, by2 - by1)
            dist_est = (self.cal_const / diameter) / 100.0 if diameter > 0 else 0.0  # approximate meters

            # Normalized coordinate: center is (0,0), range roughly [-1, 1]
            ball_pt = Point()
            ball_pt.x = float((bcx - cx) / cx)
            ball_pt.y = float((cy - bcy) / cy)
            ball_pt.z = float(dist_est)
            self.ball_pos_pub.publish(ball_pt)

            if self.ball_info_pub is not None:
                info_msg = self.ball_info_msg_cls()
                info_msg.header = msg.header
                info_msg.x_pixel = int(bcx)
                info_msg.y_pixel = int(bcy)
                info_msg.radius_pixel = float(diameter / 2.0)
                info_msg.distance = float(dist_est)
                info_msg.confidence = float(best_ball['conf'])
                self.ball_info_pub.publish(info_msg)

            if annotated is not None:
                cv2.circle(annotated, (int(bcx), int(bcy)), 5, (0, 0, 255), -1)
                cv2.line(annotated, (int(cx), int(cy)), (int(bcx), int(bcy)), (0, 255, 0), 1)

        # Publish Goal Position
        if best_goal is not None:
            gx1, gy1, gx2, gy2 = best_goal['box']
            gcx = (gx1 + gx2) / 2.0
            gcy = (gy1 + gy2) / 2.0
            gw = gx2 - gx1

            goal_pt = Point()
            goal_pt.x = float((gcx - cx) / cx)
            goal_pt.y = float((cy - gcy) / cy)
            goal_pt.z = float(gw / w)  # relative width
            self.goal_pos_pub.publish(goal_pt)

            if annotated is not None:
                cv2.circle(annotated, (int(gcx), int(gcy)), 5, (255, 0, 255), -1)

        # Publish Friend Position (United Soccer)
        if best_friend is not None and self.mode == 'united_soccer':
            fx1, fy1, fx2, fy2 = best_friend['box']
            fcx = (fx1 + fx2) / 2.0
            fcy = (fy1 + fy2) / 2.0
            friend_pt = Point()
            friend_pt.x = float((fcx - cx) / cx)
            friend_pt.y = float((cy - fcy) / cy)
            friend_pt.z = 1.0
            self.friend_pos_pub.publish(friend_pt)

        # Publish Annotated Frame
        if annotated is not None:
            mode_str = f"MODE: {self.mode.upper()}"
            hud = f"{mode_str} | FPS: {self.fps:.1f} | Latency: {t_infer:.1f}ms"
            cv2.putText(
                annotated, hud, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA
            )
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self.annotated_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YantraOpenVINODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
