#!/usr/bin/env python3
"""
RDK X5 BPU Inference Node
地平线RDK X5 BPU推理节点 - YOLOv8-Pose目标检测
基于实际BPU推理代码集成

注意: 此节点必须在RDK X5上运行，需要hobot_dnn库
"""

import sys
import time
from collections import deque

import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: OpenCV missing. Run: pip install opencv-python")
    sys.exit(1)

# 强制要求hobot_dnn
try:
    from hobot_dnn import pyeasy_dnn as dnn
except ImportError:
    print("ERROR: hobot_dnn is REQUIRED!")
    print("This node must run on RDK X5 with hobot_dnn installed.")
    print("Install: apt install hobot-dnn")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge


# ==================== 配置 ====================
MODEL_PATH = "yolov8n_pose_rdkx5.bin"
IMG_SIZE = 640
CONF_THRESH = 0.15
NMS_IOU_THRESH = 0.45
REG_MAX = 16
NK = 5          # 关键点数量
NC = 2          # 类别数: 0=完好球, 1=破损球
STRIDES = [8, 16, 32]

# 几何判断参数
SYMMETRY_RATIO_MIN = 0.85
ANGLE_MIN = 30.0
ANGLE_MAX = 50.0
VISIBILITY_MIN = 0.45

# 投票参数
VOTE_SIZE = 15
VOTE_DMG_THRESH = 8


# ==================== 预处理 ====================
def preprocess(bgr):
    """图像预处理: resize + HWC->CHW + 添加batch维度"""
    img = cv2.resize(bgr, (IMG_SIZE, IMG_SIZE))
    return np.expand_dims(img.transpose(2, 0, 1), axis=0).astype(np.uint8)


# ==================== 后处理函数 ====================
def _sigmoid(x):
    """Sigmoid激活函数"""
    return 1.0 / (1.0 + np.exp(-np.clip(x, -20.0, 20.0)))


def _dfl(x):
    """DFL解码: 分布式焦点损失解码"""
    n = x.shape[0]
    v = x.reshape(n, 4, REG_MAX)
    v -= v.max(axis=-1, keepdims=True)
    e = np.exp(v) / np.exp(v).sum(axis=-1, keepdims=True)
    return (e * np.arange(REG_MAX, dtype=np.float32).reshape(1, 1, -1)).sum(axis=-1)


def _anchors(h, w, s):
    """生成锚点网格"""
    gy, gx = np.mgrid[0:h, 0:w]
    return np.stack([(gx.ravel() + 0.5) * s, (gy.ravel() + 0.5) * s], axis=-1).astype(np.float32)


def _dist2bbox(d, a):
    """距离转边界框: (left, top, right, bottom) -> (x1, y1, x2, y2)"""
    return np.stack([a[:, 0] - d[:, 0], a[:, 1] - d[:, 1],
                     a[:, 0] + d[:, 2], a[:, 1] + d[:, 3]], axis=-1)


def _decode_kpts(p, a, s):
    """解码关键点坐标"""
    k = p.reshape(-1, NK, 3).astype(np.float32).copy()
    k[:, :, 0] = (_sigmoid(k[:, :, 0]) * 2.0 - 0.5 + a[:, 0:1]) * s
    k[:, :, 1] = (_sigmoid(k[:, :, 1]) * 2.0 - 0.5 + a[:, 1:2]) * s
    k[:, :, 2] = _sigmoid(k[:, :, 2])
    return k


def _nms(boxes, scores, th=NMS_IOU_THRESH):
    """非极大值抑制"""
    if len(boxes) == 0:
        return np.array([], dtype=np.int32)
    
    order = np.argsort(scores)[::-1]
    keep = []
    
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        
        iw = np.maximum(0.0, xx2 - xx1)
        ih = np.maximum(0.0, yy2 - yy1)
        inter = iw * ih
        
        a_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        a_r = (boxes[rest, 2] - boxes[rest, 0]) * (boxes[rest, 3] - boxes[rest, 1])
        iou = inter / (a_i + a_r - inter + 1e-6)
        
        order = rest[np.where(iou <= th)[0]]
    
    return np.array(keep, dtype=np.int32)


def process(raw):
    """处理BPU输出: 解码边界框、类别、关键点
    
    Args:
        raw: BPU原始输出, 9个tensor (3个检测尺度 × 3个分支)
    
    Returns:
        boxes: 边界框 [N, 4]
        scores: 置信度 [N]
        cls_ids: 类别ID [N]
        kpts: 关键点 [N, NK, 3]
    """
    out_b, out_s, out_c, out_k = [], [], [], []
    
    for i in range(3):
        # 获取3个分支的输出
        box_r, cls_r, pose_r = [raw[i * 3 + j][0] for j in range(3)]
        _, h, w = box_r.shape
        s = STRIDES[i]
        
        # 重塑为 [N, C] 格式
        bf = box_r.reshape(-1, h * w).T
        cf = cls_r.reshape(NC, h * w).T  # [N, NC]
        pf = pose_r.reshape(-1, h * w).T
        
        # DFL解码边界框
        dist = _dfl(bf)
        anchors = _anchors(h, w, s)
        boxes = _dist2bbox(dist, anchors)
        
        # 类别概率
        probs = _sigmoid(cf)
        cls_ids = np.argmax(probs, axis=1)
        scores = probs[np.arange(len(cls_ids)), cls_ids]
        
        # 置信度过滤
        mask = scores > CONF_THRESH
        if not mask.any():
            continue
        
        bf = boxes[mask]
        sf = scores[mask]
        cf_m = cls_ids[mask]
        af = anchors[mask]
        pf = pf[mask]
        
        out_b.append(bf)
        out_s.append(sf)
        out_c.append(cf_m)
        out_k.append(_decode_kpts(pf, af, s))
    
    if not out_b:
        return (np.empty((0, 4)), np.empty((0,)), 
                np.empty((0,), dtype=int), np.empty((0, NK, 3)))
    
    all_b = np.concatenate(out_b)
    all_s = np.concatenate(out_s)
    all_c = np.concatenate(out_c)
    all_k = np.concatenate(out_k)
    
    keep = _nms(all_b, all_s)
    return all_b[keep], all_s[keep], all_c[keep], all_k[keep]


# ==================== 几何信息 ====================
def geo_info(kpts):
    """返回几何详情用于显示"""
    P = [kpts[i, :2] for i in range(5)]
    v3, v4 = kpts[3, 2], kpts[4, 2]
    
    if v3 < VISIBILITY_MIN or v4 < VISIBILITY_MIN:
        return f"v3={v3:.2f} v4={v4:.2f}"
    
    Ll = np.linalg.norm(P[3] - P[1])
    Lr = np.linalg.norm(P[4] - P[2])
    
    if Ll < 1e-6 or Lr < 1e-6:
        return "zero_seg"
    
    ratio = min(Ll, Lr) / max(Ll, Lr)
    
    vL = P[3] - P[0]
    vR = P[4] - P[0]
    nL, nR = np.linalg.norm(vL), np.linalg.norm(vR)
    
    if nL < 1e-6 or nR < 1e-6:
        return f"r={ratio:.2f}"
    
    t = np.arccos(np.clip(np.dot(vL, vR) / (nL * nR), -1, 1)) * 180.0 / np.pi
    return f"r={ratio:.2f} a={t:.0f}"


# ==================== 可视化 ====================
SKEL = [(0, 1, (255, 255, 0)), (0, 2, (255, 255, 0)),
        (1, 3, (0, 255, 255)), (2, 4, (0, 255, 255))]


def draw(frame, boxes, scores, kpts_all, verdicts, oh, ow):
    """绘制检测结果"""
    sx, sy = ow / IMG_SIZE, oh / IMG_SIZE
    
    for box, sc, kpts, (ok, detail) in zip(boxes, scores, kpts_all, verdicts):
        x1, y1, x2, y2 = [int(v * s) for v, s in zip(box, [sx, sy, sx, sy])]
        color = (0, 255, 0) if ok else (0, 0, 255)
        label = "PERFECT" if ok else "DAMAGED"
        
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"{label} {sc:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(frame, detail, (x1, y2 + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # 绘制关键点
        kp = kpts.copy()
        kp[:, 0] *= sx
        kp[:, 1] *= sy
        
        for j, (kx, ky, kv) in enumerate(kp):
            if kv > 0.3:
                cv2.circle(frame, (int(kx), int(ky)), 4, (255, 0, 0), -1)
                cv2.putText(frame, str(j), (int(kx) + 5, int(ky) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        
        # 绘制骨架
        for a, b, clr in SKEL:
            if kp[a, 2] > 0.3 and kp[b, 2] > 0.3:
                cv2.line(frame, (int(kp[a, 0]), int(kp[a, 1])),
                         (int(kp[b, 0]), int(kp[b, 1])), clr, 2)


def draw_vote(frame, vw):
    """绘制投票窗口"""
    bx, by = frame.shape[1] - 160, 10
    bw = max(1, 150 // VOTE_SIZE)
    
    for i in range(VOTE_SIZE):
        x = bx + i * (bw + 2)
        if i >= len(vw):
            c = (80, 80, 80)
        elif list(vw)[i]:
            c = (0, 255, 0)  # 完好
        else:
            c = (0, 0, 255)  # 破损
        cv2.rectangle(frame, (x, by), (x + bw, by + 20), c, -1)
    
    dmg = sum(1 for v in vw if not v)
    cv2.putText(frame, f"Vote: {dmg}/{len(vw)} (need {VOTE_DMG_THRESH})", 
                (bx, by + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)


# ==================== ROS2节点 ====================
class BPUInference(Node):
    """RDK X5 BPU推理节点"""
    
    # 类别名称
    CLASS_NAMES = {0: 'PERFECT', 1: 'DAMAGED'}
    
    def __init__(self):
        super().__init__('rdkx5_inference')
        
        # 参数
        self.declare_parameter('model_path', MODEL_PATH)
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('enable_visualization', True)
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.enable_vis = self.get_parameter('enable_visualization').get_parameter_value().bool_value
        
        # 加载模型 (必须成功)
        self.model = self.load_model(model_path)
        
        # 打开摄像头
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error('Camera failed to open!')
            raise RuntimeError('Camera init failed')
        
        # 发布者
        self.detection_pub = self.create_publisher(String, '/detection/raw', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/detection/pixel', 10)
        self.class_pub = self.create_publisher(Int32, '/detection/class', 10)
        
        # OpenCV桥
        self.bridge = CvBridge()
        
        # 投票窗口
        self.vote_window = deque(maxlen=VOTE_SIZE)
        
        # 性能统计
        self.fps_history = deque(maxlen=30)
        self.frame_count = 0
        
        # 定时器 (30fps)
        self.timer = self.create_timer(1.0 / 30.0, self.inference_loop)
        
        self.get_logger().info(f'BPU Inference started: {model_path}')
        self.get_logger().info(f'Vote: size={VOTE_SIZE}, threshold={VOTE_DMG_THRESH}')
    
    def load_model(self, model_path):
        """加载BPU模型 - 必须成功"""
        try:
            model = dnn.load(model_path)[0]
            self.get_logger().info(f'Model loaded: {model_path}')
            for o in model.outputs:
                self.get_logger().info(f'  Output: {o.name} {o.shape}')
            return model
        except Exception as e:
            self.get_logger().error(f'Model load FAILED: {e}')
            self.get_logger().error('Make sure model file exists and is valid.')
            raise RuntimeError(f'Model load failed: {e}')
    
    def inference_loop(self):
        """推理主循环"""
        t0 = time.time()
        
        # 读取摄像头
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Failed to read camera')
            return
        
        oh, ow = frame.shape[:2]
        
        # 预处理
        data = preprocess(frame)
        t1 = time.time()
        
        # BPU推理 (必须成功)
        raw = self.model.forward(data)
        t2 = time.time()
        
        # 后处理
        boxes, scores, cls_ids, kpts = process(raw)
        
        # 判断结果
        verdicts = []
        for cid, k in zip(cls_ids, kpts):
            is_perfect = (cid == 0)
            detail = geo_info(k)
            verdicts.append((is_perfect, f"{self.CLASS_NAMES[cid]} {detail}"))
        
        # 投票机制
        is_ok = all(v[0] for v in verdicts) if verdicts else True
        self.vote_window.append(is_ok)
        dmg_count = sum(1 for v in self.vote_window if not v)
        override = dmg_count >= VOTE_DMG_THRESH
        
        # 最终判定
        if override:
            final = [(False, f"DAMAGED vote:{dmg_count}/{len(self.vote_window)}") 
                     for ok, d in verdicts]
        else:
            final = verdicts
        
        # 发布检测结果
        self.publish_detections(boxes, scores, cls_ids, kpts, final, frame.header if hasattr(frame, 'header') else None)
        
        # 可视化
        if self.enable_vis:
            if len(boxes):
                draw(frame, boxes, scores, kpts, final, oh, ow)
            
            # FPS显示
            self.fps_history.append(1.0 / (time.time() - t0 + 1e-6))
            cv2.putText(frame, f"FPS:{np.mean(self.fps_history):.1f} BPU:{(t2-t1)*1000:.1f}ms",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            draw_vote(frame, self.vote_window)
            
            cv2.imshow("Shuttlecock Integrity", frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                rclpy.shutdown()
            elif k == ord('s'):
                fn = f"screenshot_{int(time.time())}.jpg"
                cv2.imwrite(fn, frame)
                self.get_logger().info(f'Saved: {fn}')
        
        self.frame_count += 1
    
    def publish_detections(self, boxes, scores, cls_ids, kpts, verdicts, stamp):
        """发布检测结果到ROS话题"""
        for i, (box, sc, cid, kpt, (ok, detail)) in enumerate(
                zip(boxes, scores, cls_ids, kpts, verdicts)):
            
            # 计算中心点
            cx = (box[0] + box[2]) / 2
            cy = (box[1] + box[3]) / 2
            
            # 发布像素坐标
            pose_msg = PoseStamped()
            if stamp is not None:
                pose_msg.header = stamp
            else:
                pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'camera_frame'
            pose_msg.pose.position.x = float(cx)
            pose_msg.pose.position.y = float(cy)
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)
            
            # 发布类别
            class_msg = Int32()
            class_msg.data = int(cid)
            self.class_pub.publish(class_msg)
            
            # 发布原始结果
            raw_msg = String()
            raw_msg.data = f'DET:{cx:.0f},{cy:.0f},0.0,{cid},{sc:.3f},{detail}'
            self.detection_pub.publish(raw_msg)
            
            self.get_logger().info(
                f'Detected: class={self.CLASS_NAMES[cid]}, '
                f'pos=({cx:.0f}, {cy:.0f}), conf={sc:.3f}, {detail}'
            )
    
    def destroy_node(self):
        """销毁节点"""
        self.cap.release()
        if self.enable_vis:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BPUInference()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"FATAL: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
