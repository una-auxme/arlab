from collections import deque

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client as BlackboardClient
from py_trees.common import Access, Status
from rclpy.node import Node
from std_msgs.msg import String

BLACKBOARD_SPEECH_KEY = "/speech_queue"


def queue_speech(blackboard: BlackboardClient, text: str):
    if not blackboard.is_registered(BLACKBOARD_SPEECH_KEY):
        blackboard.register_key(key=BLACKBOARD_SPEECH_KEY, access=Access.WRITE)
    if not blackboard.exists(BLACKBOARD_SPEECH_KEY) or blackboard.speech_queue is None:
        blackboard.speech_queue = deque()

    blackboard.speech_queue.append(text)


class QueueSpeech(Behaviour):
    def __init__(self, text: str):
        super().__init__(name=type(self).__name__)
        self.text = text
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=BLACKBOARD_SPEECH_KEY, access=Access.WRITE)

    def update(self):
        queue_speech(self.blackboard, self.text)
        return Status.SUCCESS


class SpeechOutput(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=BLACKBOARD_SPEECH_KEY, access=Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs"
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.speech_pub = self.node.create_publisher(String, "/tts_output", qos_profile=10)

    def update(self):
        if self.blackboard.exists(BLACKBOARD_SPEECH_KEY) and self.blackboard.speech_queue is not None:
            queue = self.blackboard.speech_queue
            while len(queue) > 0:
                text = queue.popleft()
                self.speech_pub.publish(String(data=f"{text} "))
            self.blackboard.speech_queue = queue
        return Status.RUNNING
