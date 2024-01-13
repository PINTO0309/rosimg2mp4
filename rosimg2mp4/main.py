#!/usr/bin/python

from argparse import ArgumentParser
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime
from typing import Tuple

RED="\033[31m"
YELLOW="\033[33m"
GREEN="\033[32m"
BLUE="\033[34m"
RESET="\033[0m"

class ImageSaverNode(Node):
    def __init__(
        self,
        rgb_image_topic_name: str,
        frame_size: Tuple[int, int],
        video_writer_fps: float,
        output_mp4_file_name: str,
    ):
        super().__init__('image_saver')
        self.subscription = \
            self.create_subscription(
                msg_type=Image,
                topic=rgb_image_topic_name,
                callback=self.image_callback,
                qos_profile=10,
            )
        self.bridge = CvBridge()
        self.video_writer = \
            cv2.VideoWriter(
                filename=output_mp4_file_name \
                    if output_mp4_file_name else f'output_{datetime.strftime(datetime.now(), "%Y%m%d%H%M%S")}.mp4',
                fourcc=cv2.VideoWriter_fourcc(*'mp4v'),
                fps=video_writer_fps,
                frameSize=frame_size,
            )
        self.frame_size = frame_size

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        resized_cv_image = cv2.resize(cv_image, self.frame_size)
        self.video_writer.write(resized_cv_image)

    def destroy_node(self):
        self.video_writer.release()
        super().destroy_node()

def main():
    parser = ArgumentParser()
    parser.add_argument(
        '-i',
        '--rgb_image_topic_name',
        type=str,
        default='/zed2i/zed_node/rgb_raw/image_raw_color',
        help='RGB image topic name.',
    )
    parser.add_argument(
        '-o',
        '--output_mp4_file_name',
        type=str,
        default='',
        help='Output MP4 file name. e.g. output.mp4',
    )
    parser.add_argument(
        '-fs',
        '--frame_size',
        type=int,
        nargs=2,
        default=(896, 512),
        help='Frame size. e.g. --frame-size {Width} {Height}',
    )
    parser.add_argument(
        '-vf',
        '--video_writer_fps',
        type=float,
        default=15.0,
        help='Video writer FPS.',
    )
    args = parser.parse_args()

    rclpy.init()
    image_saver = \
        ImageSaverNode(
            rgb_image_topic_name=args.rgb_image_topic_name,
            frame_size=args.frame_size,
            video_writer_fps=args.video_writer_fps,
            output_mp4_file_name=args.output_mp4_file_name,
        )
    try:
        print(f'{GREEN}Start recording... Ctrl+C to end recording.{RESET}')
        while rclpy.ok():
            rclpy.spin_once(image_saver)
    except KeyboardInterrupt:
        pass
    finally:
        image_saver.destroy_node()
        print('')
        print(f'{GREEN}Stop recording.{RESET}')
    try:
        rclpy.shutdown()
    except:
        pass

if __name__ == '__main__':
    main()