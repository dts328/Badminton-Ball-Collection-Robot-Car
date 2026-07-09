#!/usr/bin/env python3
"""
Orbbec Depth Camera Node
奥比中光深度相机驱动节点

注意: 此节点需要安装pyorbbecsdk库
"""

import sys
import time

import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: OpenCV missing. Run: pip install opencv-python")
    sys.exit(1)

# 强制要求pyorbbecsdk
try:
    from pyorbbecsdk import Pipeline, FrameSet, VideoStreamType, OBSensorType
    ORBBEC_SDK_AVAILABLE = True
except ImportError:
    print("ERROR: pyorbbecsdk is REQUIRED!")
    print("This node requires Orbbec SDK installed.")
    print("Install: pip install pyorbbecsdk")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header


class OrbbecNode(Node):
    """奥比中光深度相机节点"""
    
    def __init__(self):
        super().__init__('orbbec_camera')
        
        # Parameters
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_pointcloud', True)
        self.declare_parameter('frame_id', 'orbbec_frame')
        
        # Get parameters
        self.color_width = self.get_parameter('color_width').get_parameter_value().integer_value
        self.color_height = self.get_parameter('color_height').get_parameter_value().integer_value
        self.depth_width = self.get_parameter('depth_width').get_parameter_value().integer_value
        self.depth_height = self.get_parameter('depth_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Publishers
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/camera/depth/points', 10)
        
        # Camera intrinsics (default, will be updated from device)
        self.color_fx = 616.0
        self.color_fy = 616.0
        self.color_cx = 320.0
        self.color_cy = 240.0
        
        self.depth_fx = 570.0
        self.depth_fy = 570.0
        self.depth_cx = 320.0
        self.depth_cy = 240.0
        
        # Initialize camera (must succeed)
        self.pipeline = self.init_camera()
        
        # Timer
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frames)
        
        self.get_logger().info(f'Orbbec Camera started: {self.color_width}x{self.color_height}@{self.fps}fps')
    
    def init_camera(self):
        """Initialize Orbbec camera - must succeed"""
        try:
            # Create pipeline
            pipeline = Pipeline()
            
            # Configure streams
            config = pipeline.get_config()
            
            # Enable color stream
            config.enable_stream(OBSensorType.COLOR_SENSOR, 
                                self.color_width, self.color_height, 
                                self.fps)
            
            # Enable depth stream
            config.enable_stream(OBSensorType.DEPTH_SENSOR,
                                self.depth_width, self.depth_height,
                                self.fps)
            
            # Start pipeline
            pipeline.start(config)
            
            # Get camera intrinsics
            self.get_camera_intrinsics(pipeline)
            
            self.get_logger().info('Orbbec camera initialized successfully')
            return pipeline
            
        except Exception as e:
            self.get_logger().error(f'Camera init FAILED: {e}')
            self.get_logger().error('Make sure Orbbec camera is connected.')
            raise RuntimeError(f'Camera init failed: {e}')
    
    def get_camera_intrinsics(self, pipeline):
        """Get camera intrinsics from device"""
        try:
            # Get color stream profile
            profiles = pipeline.get_stream_profiles(OBSensorType.COLOR_SENSOR)
            if profiles:
                profile = profiles[0]
                intrinsics = profile.get_intrinsic()
                self.color_fx = intrinsics.fx
                self.color_fy = intrinsics.fy
                self.color_cx = intrinsics.cx
                self.color_cy = intrinsics.cy
                self.get_logger().info(f'Color intrinsics: fx={self.color_fx}, fy={self.color_fy}')
            
            # Get depth stream profile
            profiles = pipeline.get_stream_profiles(OBSensorType.DEPTH_SENSOR)
            if profiles:
                profile = profiles[0]
                intrinsics = profile.get_intrinsic()
                self.depth_fx = intrinsics.fx
                self.depth_fy = intrinsics.fy
                self.depth_cx = intrinsics.cx
                self.depth_cy = intrinsics.cy
                self.get_logger().info(f'Depth intrinsics: fx={self.depth_fx}, fy={self.depth_fy}')
                
        except Exception as e:
            self.get_logger().warn(f'Failed to get intrinsics, using defaults: {e}')
    
    def publish_frames(self):
        """Publish camera frames"""
        stamp = self.get_clock().now().to_msg()
        
        try:
            frames = self.pipeline.wait_for_frames(100)  # 100ms timeout
            
            if frames is not None:
                # Publish color frame
                color_frame = frames.get_color_frame()
                if color_frame:
                    self.publish_color_image(color_frame, stamp)
                
                # Publish depth frame
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    self.publish_depth_image(depth_frame, stamp)
                    self.publish_pointcloud(depth_frame, stamp)
                    
        except Exception as e:
            self.get_logger().debug(f'Frame capture error: {e}')
    
    def publish_color_image(self, frame, stamp):
        """Publish color image from Orbbec frame"""
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = frame.get_height()
        msg.width = frame.get_width()
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = msg.width * 3
        
        # Get frame data
        data = frame.get_data()
        msg.data = data.tobytes() if isinstance(data, np.ndarray) else bytes(data)
        
        self.color_pub.publish(msg)
        
        # Publish camera info
        info = self.create_camera_info(stamp, 'color')
        self.color_info_pub.publish(info)
    
    def publish_depth_image(self, frame, stamp):
        """Publish depth image from Orbbec frame"""
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = frame.get_height()
        msg.width = frame.get_width()
        msg.encoding = '16UC1'
        msg.is_bigendian = False
        msg.step = msg.width * 2
        
        # Get frame data
        data = frame.get_data()
        msg.data = data.tobytes() if isinstance(data, np.ndarray) else bytes(data)
        
        self.depth_pub.publish(msg)
        
        # Publish camera info
        info = self.create_camera_info(stamp, 'depth')
        self.depth_info_pub.publish(info)
    
    def publish_pointcloud(self, depth_frame, stamp):
        """Publish point cloud from depth frame"""
        # Get depth data
        depth_data = depth_frame.get_data()
        if isinstance(depth_data, bytes):
            depth = np.frombuffer(depth_data, dtype=np.uint16).reshape(
                depth_frame.get_height(), depth_frame.get_width()
            )
        else:
            depth = depth_data
        
        # Generate point cloud
        height, width = depth.shape
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        
        # Convert to meters
        z = depth.astype(np.float32) * 0.001  # mm to m
        x = (u - self.depth_cx) * z / self.depth_fx
        y = (v - self.depth_cy) * z / self.depth_fy
        
        # Filter valid points
        mask = z > 0.1  # Minimum 10cm
        points = np.stack([x[mask], y[mask], z[mask]], axis=-1)
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = points.shape[0]
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        
        self.cloud_pub.publish(msg)
    
    def create_camera_info(self, stamp, camera_type):
        """Create camera info message"""
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        
        if camera_type == 'color':
            info.width = self.color_width
            info.height = self.color_height
            info.k = [self.color_fx, 0, self.color_cx,
                      0, self.color_fy, self.color_cy,
                      0, 0, 1]
            info.p = [self.color_fx, 0, self.color_cx, 0,
                      0, self.color_fy, self.color_cy, 0,
                      0, 0, 1, 0]
        else:
            info.width = self.depth_width
            info.height = self.depth_height
            info.k = [self.depth_fx, 0, self.depth_cx,
                      0, self.depth_fy, self.depth_cy,
                      0, 0, 1]
            info.p = [self.depth_fx, 0, self.depth_cx, 0,
                      0, self.depth_fy, self.depth_cy, 0,
                      0, 0, 1, 0]
        
        return info
    
    def destroy_node(self):
        """Destroy node"""
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OrbbecNode()
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
