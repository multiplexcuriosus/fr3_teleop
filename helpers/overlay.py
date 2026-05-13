#!/usr/bin/env python3

import argparse
import h5py
import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Hdf5LiveOverlay(Node):
    def __init__(self, args):
        super().__init__("hdf5_live_overlay")

        self.bridge = CvBridge()
        self.alpha = args.alpha
        self.window_name = args.window_name

        self.ref_image = self.load_first_hdf5_image(
            hdf5_path=args.hdf5,
            data_path=args.data_path,
        )

        self.get_logger().info(f"Loaded reference image: {self.ref_image.shape}")
        self.get_logger().info(f"Subscribing to: {args.topic}")

        self.sub = self.create_subscription(
            Image,
            args.topic,
            self.image_callback,
            10,
        )

    def load_first_hdf5_image(self, hdf5_path: str, data_path: str) -> np.ndarray:
        with h5py.File(hdf5_path, "r") as f:
            if data_path not in f:
                raise KeyError(f"Data path '{data_path}' not found in {hdf5_path}")

            data = f[data_path]

            if data.ndim == 4:
                img = data[0]          # [T, H, W, C] or [T, C, H, W]
            elif data.ndim == 3:
                img = data[...]        # [H, W, C] or [C, H, W]
            else:
                raise ValueError(f"Unsupported image shape: {data.shape}")

        img = np.asarray(img)

        # Handle CHW -> HWC
        if img.ndim == 3 and img.shape[0] in (1, 3, 4) and img.shape[-1] not in (1, 3, 4):
            img = np.transpose(img, (1, 2, 0))

        # Normalize if stored as float
        if np.issubdtype(img.dtype, np.floating):
            if img.max() <= 1.0:
                img = img * 255.0
            img = np.clip(img, 0, 255).astype(np.uint8)
        else:
            img = img.astype(np.uint8)

        # Ensure 3-channel BGR for OpenCV
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif img.shape[2] == 1:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif img.shape[2] == 3:
            # Assume HDF5 images are RGB, convert to BGR for OpenCV display
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif img.shape[2] == 4:
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        else:
            raise ValueError(f"Unsupported channel count: {img.shape}")

        return img

    def image_callback(self, msg: Image):
        try:
            live = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        if live.shape[:2] != self.ref_image.shape[:2]:
            ref = cv2.resize(
                self.ref_image,
                (live.shape[1], live.shape[0]),
                interpolation=cv2.INTER_AREA,
            )
        else:
            ref = self.ref_image

        overlay = cv2.addWeighted(
            ref,
            self.alpha,
            live,
            1.0 - self.alpha,
            0.0,
        )

        overlay = cv2.rotate(overlay,cv2.ROTATE_180)

        cv2.imshow(self.window_name, overlay)
        cv2.waitKey(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Overlay first HDF5 image with live ROS2 image topic."
    )

    parser.add_argument(
        "--hdf5",
        required=True,
        help="Path to HDF5 file.",
    )

    parser.add_argument(
        "--data-path",
        default="/observations/images/rgb",
        help="Dataset path inside the HDF5 file.",
    )

    parser.add_argument(
        "--topic",
        default="/camera/camera/color/image_raw",
        help="ROS2 image topic to subscribe to.",
    )

    parser.add_argument(
        "--alpha",
        type=float,
        default=0.5,
        help="Blend weight for HDF5 image. Live image gets 1-alpha.",
    )

    parser.add_argument(
        "--window-name",
        default="HDF5 / Live Overlay",
        help="OpenCV window name.",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    if not 0.0 <= args.alpha <= 1.0:
        raise ValueError("--alpha must be between 0 and 1")

    rclpy.init()
    node = Hdf5LiveOverlay(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()