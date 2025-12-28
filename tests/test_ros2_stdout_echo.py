"""
Tests ROS2 and stdout using the talker/listened nodes
"""

import pytest

pytest.importorskip("rclpy")

import asyncio
import logging
import subprocess
import sys
from os import environ
from typing import Any, AsyncGenerator, Callable, Generator

import rclpy
from base_tests import (
    test_freshness,
    test_listen_one_by_one,
    test_listen_too_fast,
    test_reliable_extremely_fast,
    test_reliable_one_by_one,
    test_reliable_too_fast,
    test_wait_for_value,
    test_wait_new,
    test_wait_next,
)
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import asyncio_for_robotics.ros2 as afor
import asyncio_for_robotics.textio as afor_io
from asyncio_for_robotics.core._logger import setup_logger

is_win = (
    sys.version_info[0] == 3
    and sys.version_info[1] >= 8
    and sys.platform.startswith("win")
)

pytestmark = pytest.mark.skipif(is_win, reason="Requires a special EvenLoop")

setup_logger(debug_path="tests")
logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="module")
def zenoh_router():
    """ """
    if environ.get("RMW_IMPLEMENTATION") != "rmw_zenoh_cpp":
        yield None
        return
    logger.info("Starting zenoh router")
    proc = subprocess.Popen(
        ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    for line in iter(proc.stdout.readline, ""):
        if "Started Zenoh router with id" in line:
            logger.info("Router started!")
            break
    yield proc
    logger.info("Closing zenoh router")
    proc.terminate()
    proc.wait()
    logger.info("Closed zenoh router")


@pytest.fixture(scope="module", autouse=True)
def session(zenoh_router) -> Generator[afor.BaseSession, Any, Any]:
    logger.info("Starting rclpy and session")
    rclpy.init()
    ses = afor.auto_session()
    yield ses
    logger.info("closing rclpy and session")
    ses.close()
    rclpy.shutdown()


@pytest.fixture(scope="module")
def node_process(zenoh_router):
    """Fixture running the ros example publisher.

    This will start a background OS process, then close it on cleanup.
    """
    logger.info("Starting ros node")
    proc = subprocess.Popen(  # runs an OS process in the background
        ["ros2", "run", "demo_nodes_py", "listener"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    yield proc
    # cleanup: we close the process
    logger.info("Closing ros node")
    proc.terminate()
    proc.wait()
    logger.info("Closed ros node")


TOPIC = afor.TopicInfo(
    topic="/chatter",
    msg_type=String,
    qos=QoSProfile(
        depth=10000,
    ),
)


@pytest.fixture(scope="module")
def pub(session: afor.BaseSession) -> Generator[Callable[[str], None], Any, Any]:
    with session.lock() as node:
        publisher = node.create_publisher(*TOPIC.as_arg())

    def write_in_proc(input: str) -> None:
        publisher.publish(String(data=input))

    yield write_in_proc
    with session.lock() as node:
        node.destroy_publisher(publisher)


@pytest.fixture
async def sub(
    node_process: subprocess.Popen[str],
) -> AsyncGenerator[afor_io.Sub[str], Any]:
    assert node_process.stdout is not None

    def pre_process(input: str):
        if "I heard: " not in input:
            return None
        input = input.strip()
        input = input.split("I heard: [")[1]
        input = input.removesuffix("]")
        return input

    s = afor_io.from_proc_stdout(node_process, pre_process)
    yield s
    s.close()


@pytest.fixture(autouse=True)
async def listener_ready(
    pub: Callable[[str], None],
    sub: afor_io.Sub[str],
):
    """Returns once the listener node is ready."""
    payload_prefix = "ready_"
    latest_payload = payload_prefix + "0"

    async def periodic_publish():
        nonlocal latest_payload
        counter = 0
        while 1:
            counter += 1
            latest_payload = payload_prefix + str(counter)
            pub(latest_payload)
            await asyncio.sleep(0.1)

    async def wait_for_ready():
        nonlocal latest_payload
        async for msg in sub.listen_reliable(fresh=True):
            if latest_payload in msg:
                return

    periodic_task = asyncio.create_task(periodic_publish())
    ready_task = asyncio.create_task(wait_for_ready())

    try:
        await asyncio.wait_for(ready_task, timeout=5)
    except TimeoutError:
        pytest.fail("Listener node process not ready in time.")
    finally:
        ready_task.cancel()
        periodic_task.cancel()
        sub.alive.clear()
        sub._value_flag.clear()
        sub._value = None
