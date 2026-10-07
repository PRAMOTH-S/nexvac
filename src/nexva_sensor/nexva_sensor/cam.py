#!/usr/bin/env python3

import threading
import time

import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)


class LowLatencyCameraPublisher(Node):

    def __init__(self):
        super().__init__('low_latency_camera_publisher')

        # ============================================================
        # CAMERA CONFIGURATION
        # ============================================================

        # Use /dev/cam if your udev rule is configured.
        self.camera_device = '/dev/video0'

        # Low-bandwidth / low-latency settings
        self.width = 320
        self.height = 240
        self.fps = 10

        # ============================================================
        # LOW-LATENCY ROS 2 QoS
        # ============================================================

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(
            Image,
            '/vivo_camera/image_raw',
            qos
        )

        self.bridge = CvBridge()

        # ============================================================
        # LATEST FRAME BUFFER
        # ============================================================

        self.latest_frame = None
        self.frame_lock = threading.Lock()

        self.running = True

        # ============================================================
        # OPEN CAMERA
        # ============================================================

        self.cap = cv2.VideoCapture(
            self.camera_device,
            cv2.CAP_V4L2
        )

        if not self.cap.isOpened():

            self.get_logger().error(
                f'Cannot open camera: {self.camera_device}'
            )

            raise RuntimeError(
                f'Camera could not be opened: {self.camera_device}'
            )

        # ============================================================
        # CAMERA FORMAT
        # ============================================================

        # Force MJPEG.
        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*'MJPG')
        )

        # ============================================================
        # RESOLUTION
        # ============================================================

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self.width
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self.height
        )

        # ============================================================
        # FPS
        # ============================================================

        self.cap.set(
            cv2.CAP_PROP_FPS,
            self.fps
        )

        # ============================================================
        # MINIMUM CAMERA BUFFER
        # ============================================================

        self.cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1
        )

        # ============================================================
        # AUTOFOCUS
        # ============================================================

        # Disable autofocus if the camera supports this control.
        self.cap.set(
            cv2.CAP_PROP_AUTOFOCUS,
            0
        )

        # ============================================================
        # VERIFY CAMERA SETTINGS
        # ============================================================

        actual_width = int(
            self.cap.get(
                cv2.CAP_PROP_FRAME_WIDTH
            )
        )

        actual_height = int(
            self.cap.get(
                cv2.CAP_PROP_FRAME_HEIGHT
            )
        )

        actual_fps = self.cap.get(
            cv2.CAP_PROP_FPS
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Low-Latency Camera Started'
        )

        self.get_logger().info(
            f'Device     : {self.camera_device}'
        )

        self.get_logger().info(
            f'Resolution : {actual_width}x{actual_height}'
        )

        self.get_logger().info(
            f'FPS        : {actual_fps:.1f}'
        )

        self.get_logger().info(
            'Format     : MJPEG'
        )

        self.get_logger().info(
            'Rotation   : 180 degrees'
        )

        self.get_logger().info(
            'Buffer     : 1 frame'
        )

        self.get_logger().info(
            'QoS        : BEST_EFFORT / depth 1'
        )

        self.get_logger().info(
            '========================================'
        )

        # ============================================================
        # CAMERA CAPTURE THREAD
        # ============================================================

        self.capture_thread = threading.Thread(
            target=self.capture_loop,
            daemon=True
        )

        self.capture_thread.start()

        # ============================================================
        # ROS PUBLISH TIMER
        # ============================================================

        self.timer = self.create_timer(
            1.0 / self.fps,
            self.publish_frame
        )

    # ================================================================
    # CAMERA CAPTURE THREAD
    # ================================================================

    def capture_loop(self):

        while self.running:

            ret, frame = self.cap.read()

            if not ret:

                self.get_logger().warning(
                    'Camera frame read failed'
                )

                time.sleep(0.001)

                continue

            # ========================================================
            # ROTATE IMAGE 180 DEGREES
            # ========================================================

            frame = cv2.rotate(
                frame,
                cv2.ROTATE_180
            )

            # ========================================================
            # STORE ONLY THE LATEST FRAME
            # ========================================================
            #
            # The old frame is immediately replaced.
            #
            # This prevents latency from accumulating.
            #
            # ========================================================

            with self.frame_lock:

                self.latest_frame = frame

    # ================================================================
    # ROS IMAGE PUBLISHER
    # ================================================================

    def publish_frame(self):

        # ============================================================
        # GET ONLY THE LATEST FRAME
        # ============================================================

        with self.frame_lock:

            if self.latest_frame is None:

                return

            frame = self.latest_frame.copy()

        # ============================================================
        # CONVERT OPENCV -> ROS IMAGE
        # ============================================================

        try:

            msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding='bgr8'
            )

            # ========================================================
            # TIMESTAMP
            # ========================================================

            msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            # ========================================================
            # FRAME ID
            # ========================================================

            msg.header.frame_id = (
                'vivo_camera_link'
            )

            # ========================================================
            # PUBLISH
            # ========================================================

            self.publisher.publish(msg)

        except Exception as e:

            self.get_logger().error(
                f'Image conversion error: {e}'
            )

    # ================================================================
    # CLEAN SHUTDOWN
    # ================================================================

    def destroy_node(self):

        self.get_logger().info(
            'Stopping camera...'
        )

        # Stop capture thread.
        self.running = False

        # Wait for capture thread.
        if self.capture_thread.is_alive():

            self.capture_thread.join(
                timeout=1.0
            )

        # Release camera.
        if self.cap.isOpened():

            self.cap.release()

        self.get_logger().info(
            'Camera released'
        )

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = LowLatencyCameraPublisher()

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(
            f'Camera node error: {e}'
        )

    finally:

        if node is not None:

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


# ====================================================================
# ENTRY POINT
# ====================================================================

if __name__ == '__main__':

    main()
