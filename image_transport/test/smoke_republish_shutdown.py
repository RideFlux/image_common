#!/usr/bin/env python3
# Copyright (c) 2026, RideFlux, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the RideFlux, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Runtime smoke for image_transport republish shutdown and compressed relay."""

import argparse
import json
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import asdict, dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header


@dataclass
class RunResult:
    """Record readiness and termination of one republish process."""

    transport: str
    iteration: int
    ready: bool
    exit_code: Optional[int]
    timed_out: bool
    signal_sent: bool
    error: Optional[str]


class Probe(Node):
    """Observe republish endpoints and exercise its executor."""

    def __init__(self) -> None:
        """Create a process-specific probe node."""
        super().__init__(f"image_transport_shutdown_smoke_{os.getpid()}")

    def wait_ready(self, input_topic: str, output_topic: str, timeout_seconds: float) -> bool:
        """Wait for graph discovery and a completed parameter service callback."""
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            subscriptions = self.get_subscriptions_info_by_topic(input_topic)
            publishers = self.get_publishers_info_by_topic(output_topic)
            if subscriptions and publishers:
                # Graph discovery can precede spin(). Wait for an executor callback
                # before sending SIGINT so startup timing cannot cause a false failure.
                client = self.create_client(
                    GetParameters, f"/{publishers[0].node_name}/get_parameters"
                )
                try:
                    remaining = max(0.0, deadline - time.monotonic())
                    if not client.wait_for_service(timeout_sec=remaining):
                        return False
                    future = client.call_async(GetParameters.Request(names=["use_sim_time"]))
                    rclpy.spin_until_future_complete(
                        self, future, timeout_sec=max(0.0, deadline - time.monotonic())
                    )
                    return future.done() and future.exception() is None
                finally:
                    self.destroy_client(client)
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


def parse_args() -> argparse.Namespace:
    """Read the binary path and bounded test parameters."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--republish-bin",
        type=Path,
        default=Path(get_package_prefix("image_transport")) / "lib/image_transport/republish",
        help="republish executable from the baseline or fixed install prefix",
    )
    parser.add_argument("--iterations", type=int, default=8)
    parser.add_argument("--timeout-seconds", type=float, default=10.0)
    parser.add_argument("--domain-id", type=int)
    parser.add_argument(
        "--transport", choices=("all", "compressed", "ffmpeg"), default="all"
    )
    return parser.parse_args()


def process_environment(domain_id: int) -> dict[str, str]:
    """Isolate ROS discovery and remove plugin preloading workarounds."""
    env = os.environ.copy()
    env.pop("LD_PRELOAD", None)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["ROS_LOCALHOST_ONLY"] = "1"
    return env


def terminate(
    process: subprocess.Popen[bytes], timeout_seconds: float
) -> tuple[Optional[int], bool, bool]:
    """Send SIGINT and return the exit code, timeout state, and signal state."""
    if process.poll() is not None:
        return process.returncode, False, False
    try:
        process.send_signal(signal.SIGINT)
    except ProcessLookupError:
        return process.wait(), False, False
    try:
        return process.wait(timeout=timeout_seconds), False, True
    except subprocess.TimeoutExpired:
        process.kill()
        return process.wait(), True, True


def start_republish(
    executable: Path,
    transport: str,
    input_topic: str,
    output_topic: str,
    iteration: int,
    environment: dict[str, str],
    log_file: object,
) -> subprocess.Popen[bytes]:
    """Start one republisher with process-specific topic and node names."""
    return subprocess.Popen(
        [
            str(executable),
            transport,
            "raw",
            "--ros-args",
            "-r",
            f"__node:=image_transport_shutdown_{transport}_{iteration}_{os.getpid()}",
            "-r",
            f"in/{transport}:={input_topic}",
            "-r",
            f"out:={output_topic}",
        ],
        stdin=subprocess.DEVNULL,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=environment,
    )


def verify_compressed_payload(
    probe: Probe,
    executable: Path,
    environment: dict[str, str],
    timeout_seconds: float,
    namespace: str,
    log_file: object,
) -> Optional[str]:
    """Verify JPEG delivery and clean shutdown using the compressed plugin."""
    input_topic = f"{namespace}/payload/in/compressed"
    output_topic = f"{namespace}/payload/out"
    received: list[Image] = []
    publisher = probe.create_publisher(CompressedImage, input_topic, 10)
    subscription = probe.create_subscription(Image, output_topic, received.append, 10)
    process: Optional[subprocess.Popen[bytes]] = None
    error = None
    try:
        process = start_republish(
            executable, "compressed", input_topic, output_topic, 0, environment, log_file
        )
        if not probe.wait_ready(input_topic, output_topic, timeout_seconds):
            error = "compressed payload republish did not become graph-ready"
        if error is None:
            pixels = np.full((2, 3, 3), (31, 93, 177), dtype=np.uint8)
            encoded, jpeg = cv2.imencode(".jpg", pixels)
            if not encoded:
                error = "OpenCV did not encode the JPEG input"
        if error is None:
            header = Header(stamp=Time(sec=17, nanosec=29), frame_id="image-transport-smoke")
            message = CompressedImage(header=header, format="jpeg", data=jpeg.tobytes())
            deadline = time.monotonic() + timeout_seconds
            while time.monotonic() < deadline and not received:
                publisher.publish(message)
                rclpy.spin_once(probe, timeout_sec=0.1)
            if not received:
                error = "compressed JPEG was not relayed"
        if error is None:
            image = received[0]
            if image.header != header:
                error = "relayed image header differs from compressed input"
            elif (image.width, image.height, image.encoding) != (3, 2, "bgr8"):
                error = (
                    f"unexpected relayed image metadata: "
                    f"{image.width}x{image.height} {image.encoding}"
                )
            else:
                actual = np.frombuffer(image.data, dtype=np.uint8).reshape(
                    image.height, image.step
                )
                if not np.allclose(actual[0, :3], pixels[0, 0], atol=8):
                    error = f"unexpected relayed pixel: {actual[0, :3].tolist()}"
    finally:
        exit_code, timed_out, signal_sent = (
            terminate(process, timeout_seconds) if process else (None, False, False)
        )
        publisher.destroy()
        subscription.destroy()
        if timed_out:
            error = "compressed payload process ignored SIGINT"
        elif exit_code != 0:
            error = f"compressed payload process exited with {exit_code}"
        elif not signal_sent:
            error = "compressed payload process exited before SIGINT"
    return error


def run_shutdown_iterations(
    probe: Probe,
    executable: Path,
    transport: str,
    iterations: int,
    environment: dict[str, str],
    timeout_seconds: float,
    namespace: str,
    log_file: object,
) -> list[RunResult]:
    """Exercise repeated startup and SIGINT shutdown for one input transport."""
    results = []
    for iteration in range(1, iterations + 1):
        input_topic = f"{namespace}/{transport}/iteration_{iteration}/in/{transport}"
        output_topic = f"{namespace}/{transport}/iteration_{iteration}/out"
        process: Optional[subprocess.Popen[bytes]] = None
        ready = False
        exit_code: Optional[int] = None
        timed_out = False
        signal_sent = False
        error: Optional[str] = None
        try:
            process = start_republish(
                executable, transport, input_topic, output_topic, iteration, environment, log_file
            )
            ready = probe.wait_ready(input_topic, output_topic, timeout_seconds)
            if not ready:
                error = "graph not ready"
            else:
                exit_code, timed_out, signal_sent = terminate(process, timeout_seconds)
                process = None
                if timed_out:
                    error = "process ignored SIGINT"
                elif exit_code != 0:
                    error = f"expected SIGINT exit code 0, got {exit_code}"
                elif not signal_sent:
                    error = "process exited before SIGINT"
        except Exception as exception:
            error = str(exception)
        finally:
            if process is not None:
                exit_code, timed_out, signal_sent = terminate(process, timeout_seconds)
                if timed_out:
                    error = "process ignored SIGINT"
        results.append(
            RunResult(transport, iteration, ready, exit_code, timed_out, signal_sent, error)
        )
    return results


def main() -> int:
    """Run the selected checks and emit their results as JSON."""
    args = parse_args()
    executable = args.republish_bin.resolve()
    if not executable.is_file() or not os.access(executable, os.X_OK):
        print(json.dumps({"error": f"republish binary is not executable: {executable}"}))
        return 2
    if args.iterations < 1:
        print(json.dumps({"error": "--iterations must be positive"}))
        return 2
    if args.timeout_seconds <= 0:
        print(json.dumps({"error": "--timeout-seconds must be positive"}))
        return 2
    domain_id = args.domain_id
    if domain_id is None:
        domain_id = random.SystemRandom().randint(120, 200)
    if not 0 <= domain_id <= 232:
        print(json.dumps({"error": "--domain-id must be between 0 and 232"}))
        return 2
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    environment = process_environment(domain_id)
    namespace = f"/image_transport_shutdown_smoke/run_{os.getpid()}"
    transports = ("compressed", "ffmpeg") if args.transport == "all" else (args.transport,)
    rclpy.init()
    probe = Probe()
    payload_error = None
    results: list[RunResult] = []
    with tempfile.NamedTemporaryFile(
        prefix="image_transport_shutdown_", suffix=".log"
    ) as log_file:
        try:
            if "compressed" in transports:
                payload_error = verify_compressed_payload(
                    probe,
                    executable,
                    environment,
                    args.timeout_seconds,
                    namespace,
                    log_file,
                )
            for transport in transports:
                results.extend(
                    run_shutdown_iterations(
                        probe,
                        executable,
                        transport,
                        args.iterations,
                        environment,
                        args.timeout_seconds,
                        namespace,
                        log_file,
                    )
                )
        finally:
            probe.destroy_node()
            rclpy.shutdown()
        log_size = log_file.seek(0, os.SEEK_END)
        log_file.seek(max(0, log_size - 4096))
        log_tail = log_file.read().decode(errors="replace")
    failures = [result for result in results if result.error]
    summary = {
        "republish_bin": str(executable),
        "domain_id": domain_id,
        "ld_preload": environment.get("LD_PRELOAD"),
        "compressed_payload_error": payload_error,
        "runs": [asdict(result) for result in results],
        "passed": payload_error is None and not failures,
    }
    if not summary["passed"]:
        summary["republish_log_tail"] = log_tail
    print(json.dumps(summary, separators=(",", ":")))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
