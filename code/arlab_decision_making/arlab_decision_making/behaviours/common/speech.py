"""Speech queue behaviours for py_trees with ROS2 output integration.

Provides helper functions and behaviours to:
    - enqueue speech text on the py_trees blackboard under the key
        '/speech_queue'
    - add speech output requests from behaviour tree nodes
    - publish queued speech text to the ROS2 topic '/tts_output'

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from collections import deque

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client as BlackboardClient
from py_trees.common import Access, Status
from rclpy.node import Node
from std_msgs.msg import String

BLACKBOARD_SPEECH_KEY = "/speech_queue"


def queue_speech(blackboard: BlackboardClient, text: str):
    """Append a speech text to the blackboard speech queue.

    Ensures that the speech queue key is registered at the blackboard and initialised before
    appending the given text to the queue.

    Args:
        blackboard (BlackboardClient): Blackboard client used to access the
            shared speech queue.
        text (str): Speech text to append to the queue.
    """
    if not blackboard.is_registered(BLACKBOARD_SPEECH_KEY):
        blackboard.register_key(key=BLACKBOARD_SPEECH_KEY, access=Access.WRITE)
    if not blackboard.exists(BLACKBOARD_SPEECH_KEY) or blackboard.speech_queue is None:
        blackboard.speech_queue = deque()

    blackboard.speech_queue.append(text)


class QueueSpeech(Behaviour):
    """Behaviour that appends speech text to the blackboard queue.

    This behaviour is intended for use in behaviour trees where speech output
    should be requested asynchronously through a shared queue.
    """
    def __init__(self, text: str):
        """Initialise the behaviour with the text to be queued.

        Args:
            text (str): Speech text to append to the blackboard queue when the
                behaviour is updated.
        """
        super().__init__(name=type(self).__name__)
        self.text = text
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=BLACKBOARD_SPEECH_KEY, access=Access.WRITE)

    def update(self):
        """Queue the configured speech text and return success.

        Returns:
            Status: `Status.SUCCESS` after the text has been appended to the
                speech queue.
        """
        queue_speech(self.blackboard, self.text)
        return Status.SUCCESS


class SpeechOutput(Behaviour):
    """Behaviour that publishes queued speech texts to a ROS2 topic.

    This behaviour reads pending speech entries from the blackboard queue and
    publishes them sequentially to the `/tts_output` topic as `std_msgs/String`
    messages.
    """
    def __init__(self):
        """Initialise the speech output behaviour and blackboard access."""
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=BLACKBOARD_SPEECH_KEY, access=Access.WRITE)

    def setup(self, **kwargs):
        """Set up the ROS2 publisher required for speech output.

        Args:
            **kwargs: Keyword arguments expected to contain a ROS2 node under
                the key `'node'`.

        Raises:
            KeyError: If no or a none existing ROS2 node is provided in `kwargs`.
        """
        try:
            self.node: Node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs"
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.speech_pub = self.node.create_publisher(String, "/tts_output", qos_profile=10)

    def update(self):
        """Publish all queued speech texts and keep the behaviour running.

        Returns:
            Status: `Status.RUNNING` after processing the current speech queue.
        """
        if self.blackboard.exists(BLACKBOARD_SPEECH_KEY) and self.blackboard.speech_queue is not None:
            queue = self.blackboard.speech_queue
            while len(queue) > 0:
                text = queue.popleft()
                self.speech_pub.publish(String(data=f"{text} "))
            self.blackboard.speech_queue = queue
        return Status.RUNNING
