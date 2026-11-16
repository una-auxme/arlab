#!/usr/bin/env python3

"""
DialogNode:
Receives Whisper transcripts and forwards them to Moshi TTS.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from arlab_common_interfaces.msg import WhisperTranscript


class DialogNode(Node):

    def __init__(self):
        super().__init__('dialog')

        # Whisper STT Topic
        self.stt_topic = '/whisper/transcript'

        # Moshi TTS Topic
        self.tts_topic = '/tts_output'

        # Subscriber für Whisper
        self.subscription = self.create_subscription(
            WhisperTranscript,
            self.stt_topic,
            self.stt_callback,
            10
        )

        # Publisher für Moshi TTS
        self.tts_publisher = self.create_publisher(
            String,
            self.tts_topic,
            10
        )

        self.get_logger().info('Whisper STT -> Moshi TTS gestartet')
        self.get_logger().info(f'STT: {self.stt_topic}')
        self.get_logger().info(f'TTS: {self.tts_topic}')

    def stt_callback(self, msg):

        recognized_text = msg.transcript.strip()

        if not recognized_text:
            return

        self.get_logger().info(
            f'Whisper erkannt: {recognized_text}'
        )

        # Erkannten Text direkt an Moshi senden
        tts_msg = String()
        tts_msg.data = recognized_text

        self.tts_publisher.publish(tts_msg)

        self.get_logger().info(
            'Text an Moshi TTS gesendet.'
        )


def main(args=None):

    rclpy.init(args=args)

    node = DialogNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
