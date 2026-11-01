"""Drive the robot (sim or real) with a trained LeRobot policy through the ROS 2 contract.

    python -m so101_pai.policy.policy_node --ros-args -p policy_path:=outputs/act/checkpoints/last/pretrained_model

Subscribes to state, both cameras and /episode/status; while an episode is running it publishes actions on
contract.TOPIC_ACTION, the same topic the leader arm uses, so the sim bridge and follower node need no changes.
Do not run the leader node at the same time.

By default inference runs in a worker thread and overlaps execution (chunking.ChunkScheduler), so the arm does not
stop while the next chunk is predicted; -p asynchronous:=false uses LeRobot's select_action, which pauses.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from so101_pai.common import contract, episode
from so101_pai.common.ros_msgs import image_to_rgb, joint_state_msg, joint_state_to_array
from so101_pai.common.ros_node import run
from so101_pai.policy.chunking import ChunkScheduler
from so101_pai.policy.runner import ObservationBuffer, load_policy

DEFAULT_TASK = "Put the red block on the plate."


class PolicyNode(Node):
    def __init__(self, **node_kwargs) -> None:
        # node_kwargs (e.g. parameter_overrides) let tests build the node without the command line.
        super().__init__("so101_policy", **node_kwargs)
        path = self.declare_parameter("policy_path", "").value
        if not path:
            raise ValueError("set -p policy_path:=<checkpoint dir or Hub id>")
        device = self.declare_parameter("device", "cuda").value
        task = self.declare_parameter("task", DEFAULT_TASK).value
        rate_hz = self.declare_parameter("rate_hz", float(contract.FPS)).value
        n_action_steps = self.declare_parameter("n_action_steps", 0).value  # 0: the checkpoint's setting
        self.asynchronous = self.declare_parameter("asynchronous", True).value
        lead_steps = self.declare_parameter("lead_steps", 5).value  # async: start inferring this many actions early

        self.runner = load_policy(path, device, task, n_action_steps)
        self.buffer = ObservationBuffer()
        self.state_stamp = None
        self.active = False
        # Async: one worker, so inference and policy resets run one at a time and in order.
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.scheduler = ChunkScheduler(self.runner.n_action_steps, lead_steps)
        self.inference: Future | None = None
        self.obs_tick, self.inference_stamp, self.chunk_stamp = 0, None, None

        self.action_pub = self.create_publisher(JointState, contract.TOPIC_ACTION, 10)
        self.create_subscription(JointState, contract.TOPIC_STATE, self.on_state, 10)
        self.create_subscription(String, contract.TOPIC_EPISODE, self.on_status, 10)
        for camera, topic in contract.CAMERA_TOPICS.items():
            self.create_subscription(Image, topic, lambda msg, camera=camera: self.on_image(camera, msg), 10)
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f"policy {path} on {device}: n_action_steps={self.runner.n_action_steps} "
            f"asynchronous={self.asynchronous} lead_steps={lead_steps}; waiting for an episode start"
        )

    def on_state(self, msg: JointState) -> None:
        self.state_stamp = msg.header.stamp
        self.buffer.state = contract.radians_to_motor(joint_state_to_array(msg))

    def on_image(self, camera: str, msg: Image) -> None:
        self.buffer.images[camera] = image_to_rgb(msg)

    def on_status(self, msg: String) -> None:
        event = episode.decode(msg.data)
        if event.event == episode.START:
            if self.asynchronous:
                self.pool.submit(self.runner.reset)
            else:
                self.runner.reset()
        # Drop the chunk of an inference still running from the last episode.
        self.inference = None
        self.scheduler.reset()
        self.active = event.event == episode.START
        self.get_logger().info(f"episode {event.event}: policy {'running' if self.active else 'idle'}")

    def tick(self) -> None:
        if not (self.active and self.buffer.ready()):
            return
        if self.asynchronous:
            motor, stamp = self.async_action()
            if motor is None:
                return  # first chunk of the episode not ready yet
        else:
            motor, stamp = self.runner.act(self.buffer.frame()), self.state_stamp
        radians = contract.motor_to_radians(np.asarray(motor))
        # Stamp with the observation the action came from, so the sim bridge can drop actions computed before a
        # reset (inference can still be running when the episode ends).
        self.action_pub.publish(joint_state_msg(stamp, radians))

    def async_action(self) -> tuple[np.ndarray | None, object]:
        """This tick's action from the scheduler, collecting a finished chunk and starting the next inference."""
        if self.inference is not None and self.inference.done():
            chunk = self.inference.result()  # re-raises an inference error here
            self.scheduler.receive(chunk, self.obs_tick)
            self.chunk_stamp, self.inference = self.inference_stamp, None
        if self.scheduler.should_infer():
            self.obs_tick, self.inference_stamp = self.scheduler.started(), self.state_stamp
            self.inference = self.pool.submit(self.runner.predict_chunk, self.buffer.frame())
        return self.scheduler.step(), self.chunk_stamp

    def destroy_node(self) -> None:
        self.pool.shutdown(wait=True, cancel_futures=True)
        super().destroy_node()


def main() -> None:
    run(PolicyNode)


if __name__ == "__main__":
    main()
