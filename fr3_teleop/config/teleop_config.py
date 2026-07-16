# NOTE: Canonical topic/service names now live in
# fr3_teleop.config.interfaces. This module only holds behavioral
# configuration (e.g. OpenMV camera driver settings) that is still
# consumed by launch files outside fr3_teleop (vision.launch.py,
# openmv_cam/launch/openmv.launch.py).
from fr3_teleop.config.interfaces import TOPICS

OPENMV_PARAMS = {
    "port": "/dev/openmvcam",
    "baud": 115200,
    "publish_fps": 30.0,
    "topic": TOPICS["openmv_mono"],
    "publish_3_channel_img": True,
    "topic_3_channel": TOPICS["openmv_3ch"],
}
