import asyncio
import logging
import sys

import pyfiglet
import rclpy
import uvloop
from rclpy.publisher import Publisher
from std_msgs.msg import Int64

from asyncio_for_robotics import soft_timeout
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.ros2 import *
from asyncio_for_robotics.ros2.session import set_auto_session
from afor_ros2_thrd import main

if __name__ == "__main__":
    if "--world" in sys.argv:
        use_world = True
    else:
        use_world = False
    if "--hello" in sys.argv:
        use_hello = True
    else:
        use_hello = False
    if "--hello" not in sys.argv and "--world" not in sys.argv:
        use_hello = True
        use_world = True
    rclpy.init()
    set_auto_session(SynchronousSession())
    uvloop.run(main(use_hello, use_world))
    auto_session().close()
    rclpy.shutdown()

