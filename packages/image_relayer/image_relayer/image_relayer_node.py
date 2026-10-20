#!/usr/bin/env python3
"""The Image Relayer node (ROS 2).

Relays ROS 2 compressed image topics (typically the `debug/*/compressed` images published by the
perception nodes) to DTPS, so that they can be selected and displayed in the Duckietown Viewer.

The node exposes, under `switchboard/<robot>/node/image_relayer/`:
  - `topic_of_interest`: the viewer publishes `{"topic": "<robot>/node/image_relayer/<path>/jpeg"}`
    here to pick the image topic to relay;
  - one `<path>/jpeg` queue per ROS compressed image topic `/<robot>/<path>/compressed`
    (the camera's own topics are excluded, the viewer reads them from the camera driver directly).

Only the selected topic is subscribed to on the ROS side, so idle debug topics cost nothing.
This is a port of the ROS 1 node of the same name in `dt-ros-interface`.
"""
import asyncio
from threading import Thread
from typing import Dict, List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as ROS2Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.subscription import Subscription
from sensor_msgs.msg import CompressedImage as ROS2CompressedImage

from dt_node_utils import NodeType
from dt_node_utils.node import Node
from dt_robot_utils import get_robot_name
from dtps import ContextConfig, DTPSContext, PublisherInterface
from dtps_http import RawData
from duckietown_messages.sensors.compressed_image import CompressedImage
from duckietown_messages.standard.dictionary import Dictionary
from duckietown_messages.utils.exceptions import DataDecodingError

COMPRESSED_IMAGE_TYPE: str = "sensor_msgs/msg/CompressedImage"
TOPIC_OF_INTEREST: str = "topic_of_interest"
# how often the list of ROS image topics is refreshed
TOPICS_REFRESH_PERIOD: float = 1.0


class ImageRelayerNode(Node):
    """Relays images from ROS 2 to DTPS."""

    def __init__(self):
        super().__init__(
            name="image_relayer",
            kind=NodeType.DRIVER,
            description="A ROS 2 to DTPS image relayer.",
        )
        self._robot_name: str = get_robot_name()
        self._ros2 = ROS2Node("image_relayer_node")
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        # DTPS topic (e.g. 'line_detector_node/debug/segments/jpeg') -> ROS topic
        self._ros_topics: Dict[str, str] = {}
        self._queues: Dict[str, DTPSContext] = {}
        self._subscription: Optional[Subscription] = None
        self._publisher: Optional[PublisherInterface] = None
        self._relayed_topic: Optional[str] = None
        # QoS used to subscribe to the relayed ROS topic: the latest frame is all we need, and
        # BEST_EFFORT is compatible with both best-effort and reliable publishers
        self._qos = QoSProfile(depth=1)
        self._qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._qos.history = HistoryPolicy.KEEP_LAST
        self.loginfo("Initialized.")

    # --- ROS side ----------------------------------------------------------

    def _get_ros_topics(self) -> Dict[str, str]:
        """Return the relayable ROS image topics, keyed by their DTPS name.

        `/<robot>/<path>/compressed` becomes `<path>/jpeg`, as in the ROS 1 node.
        """
        prefix = f"/{self._robot_name}/"
        topics: Dict[str, str] = {}
        for ros_topic, types in self._ros2.get_topic_names_and_types():
            if COMPRESSED_IMAGE_TYPE not in types:
                continue
            if not ros_topic.startswith(prefix) or "/camera_node/" in ros_topic:
                continue
            parts = ros_topic.split("/")[2:-1]
            if not parts:
                continue
            topics["/".join(parts + ["jpeg"])] = ros_topic
        return topics

    def _on_compressed_image_message(self, message: ROS2CompressedImage) -> None:
        """Forward a ROS compressed image to the selected DTPS queue (runs on the ROS executor thread)."""
        publisher = self._publisher
        if publisher is None or self._loop is None:
            return
        raw_data = CompressedImage(format="jpeg", data=bytes(message.data)).to_rawdata()
        asyncio.run_coroutine_threadsafe(publisher.publish(raw_data), self._loop)

    def _unsubscribe(self) -> None:
        if self._subscription is not None:
            self._ros2.destroy_subscription(self._subscription)
            self._subscription = None

    # --- DTPS side ---------------------------------------------------------

    async def _stop_relaying(self) -> None:
        self._unsubscribe()
        publisher, self._publisher = self._publisher, None
        self._relayed_topic = None
        if publisher is not None:
            try:
                await publisher.terminate()
            except Exception as error:  # noqa: BLE001 - best effort cleanup
                self.logwarn(f"Error while terminating the publisher: {error}")

    async def _on_topic_of_interest_data(self, raw_data: RawData) -> None:
        """Switch the relayed topic, e.g. `{"topic": "<robot>/node/image_relayer/<path>/jpeg"}`."""
        try:
            dictionary: Dictionary = Dictionary.from_rawdata(raw_data)
        except DataDecodingError as error:
            self.logerr(f"Failed to decode a topic of interest message: {error.message}")
            return
        requested = str(dictionary.data.get("topic", ""))
        await self._stop_relaying()
        if "/image_relayer/" not in requested:
            # not one of ours (e.g. the camera itself), nothing to relay
            return
        topic = requested.split("/image_relayer/", 1)[1]
        if topic not in self._ros_topics:
            self.logwarn(f"Unknown image topic requested: '{requested}'")
            return
        ros_topic = self._ros_topics[topic]
        self._publisher = await self._queues[topic].publisher()
        self._subscription = self._ros2.create_subscription(
            ROS2CompressedImage, ros_topic, self._on_compressed_image_message, self._qos
        )
        self._relayed_topic = topic
        self.loginfo(f"Relaying '{ros_topic}' -> '{topic}'")

    async def _set_up_queues(self) -> None:
        """(Re)create one DTPS queue per relayable ROS topic, plus `topic_of_interest`."""
        self._ros_topics = self._get_ros_topics()
        self._queues = {}
        for topic in [TOPIC_OF_INTEREST, *self._ros_topics]:
            queue = await (self.context / topic).queue_create()
            self._queues[topic] = queue.configure(ContextConfig(patient=True))
        await self._queues[TOPIC_OF_INTEREST].subscribe(self._on_topic_of_interest_data)
        self.loginfo(f"Exposing {len(self._ros_topics)} image topic(s): {sorted(self._ros_topics)}")

    async def _remove_queues(self) -> None:
        for topic, queue in self._queues.items():
            try:
                await queue.remove()
            except Exception as error:  # noqa: BLE001 - best effort cleanup
                self.logwarn(f"Error while removing queue '{topic}': {error}")
        self._queues = {}

    async def worker(self) -> None:
        self._loop = asyncio.get_event_loop()
        await self.dtps_init()
        await self._set_up_queues()
        await self.dtps_expose()
        try:
            while not self.is_shutdown and rclpy.ok():
                await asyncio.sleep(TOPICS_REFRESH_PERIOD)
                if self._get_ros_topics() != self._ros_topics:
                    # the set of ROS image topics changed: rebuild the DTPS queues
                    relayed = self._relayed_topic
                    await self._stop_relaying()
                    await self._remove_queues()
                    await self._set_up_queues()
                    if relayed in self._ros_topics:
                        # keep relaying the topic the viewer had selected
                        payload = Dictionary(data={"topic": f"{self._robot_name}/node/image_relayer/{relayed}"})
                        await self._on_topic_of_interest_data(payload.to_rawdata())
        finally:
            await self._stop_relaying()
        self.loginfo("Worker has stopped.")

    # --- lifecycle -----------------------------------------------------------

    def spin(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self._ros2)
        spin_thread = Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            asyncio.run(self.worker())
        except RuntimeError:
            if not self.is_shutdown:
                self.logerr("An error occurred while running the event loop")
                raise
        finally:
            shutdown_ok = executor.shutdown(1)
            if not shutdown_ok:
                self.logwarn("ROS2 executor did not shut down within 1 second.")
            spin_thread.join(timeout=1)
            if spin_thread.is_alive():
                self.logwarn("ROS2 executor thread is still running; skipping node destruction.")
            else:
                self._ros2.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageRelayerNode()
    try:
        node.spin()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
