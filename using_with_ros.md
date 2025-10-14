# Using with ROS 2

## Installing without a venv

```bash
pip install git+https://github.com/2lian/asyncio-for-robotics.git
```

## Installing with a venv

In my experience, the `uv` library handles venv with ros better. (replace `jazzy` with your ros distro)

```bash
cd <YOUR_WORKSPACE>
. /opt/ros/jazzy/setup.bash
# create your venv
uv venv --system-site-packages
# !!
# instructions to activate the venv are displayed by uv 
# usually `source .venv/bin/activate`
# !!
uv pip install git+https://github.com/2lian/asyncio-for-robotics.git
# to colcon build you should use
uv run colcon build
```

Before running a python script you should source ros and the venv.
```bash
cd <YOUR_WORKSPACE>
. /opt/ros/jazzy/setup.bash
source .venv/bin/activate
```

To build a ros package do not use raw colcon, but the one from you venv:
```bash
cd <YOUR_WORKSPACE>
. /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python3 -m colcon build
```

# Re-doing the ROS 2 tutorial

## Making an application

You do not need to create a ROS 2 package, nor need to use `ros2 run ...` `ros2 launch ...`. But if you want you can. Launch and entry point process do not change when using `afor`.

## Publisher node

You can follow the ros tutorial and just replace this section with `afor` style code: [Writing a simple publisher and subscriber --- 2 Write the publisher node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node)

### ROS tutorial code

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### `afor` equivalent code

```python
import asyncio

import rclpy
from std_msgs.msg import String

from asyncio_for_robotics.ros2 import auto_session
from asyncio_for_robotics.ros2 import TopicInfo

TOPIC = TopicInfo(msg_type=String, topic="topic")


async def main_async():
    rclpy.init()

    # create the publisher safely
    with auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)

    i = 0
    while 1:
        data = f"Hello world: {i}"
        i += 1
        print(f"Publishing: {data}")
        pub.publish(String(data=data)) # sends data (lock is not necessary)
        await asyncio.sleep(2) # non-blocking sleep

    auto_session().close()
    rclpy.shutdown()

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()
```

#### Examine the code

Main now delegates execution of our `async` function to asyncio.

```python
def main():
    asyncio.run(main_async())
```

`rclpy.init` and `rclpy.shutdown` is still required. Additionally `auto_session().close()` shuts down the background ros node automatically created by `afor`. This is a core concept, `afor` starts a "Session" made of an executor spinning a node, you can find more about it in the source code `asyncio_for_robotics.ros2.session`.

```python
async def main_async():
    rclpy.init()
    ...
    auto_session().close()
    rclpy.shutdown()
```

`afor` does not provide a publisher because you can directly the ROS 2 publisher. I believe it is important to let you the user in control and not hide everything behind wrappers. To get the node of the current `afor` session we need `with auto_session().lock() as node:`, this step is critical for ROS 2 to avoid race conditions (this is not required for Zenoh and other thread-safe transport protocol). Then we simply declare the publisher on the node with `node.create_publisher(...)`.

```python
with auto_session().lock() as node:
    pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
```

The loop is now very simple.

```python
i = 0
while 1:
    data = f"Hello world: {i}"
    i += 1
    print(f"Publishing: {data}")
    pub.publish(String(data=data)) # sends data (lock is not necessary)
    await asyncio.sleep(2) # non-blocking sleep
```

### Improving `afor`'s code

```python
import asyncio

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from asyncio_for_robotics.ros2 import (
    ThreadedSession,
    auto_session,
    set_auto_session,
    TopicInfo,
)

TOPIC = TopicInfo(msg_type=String, topic="example/talker")

async def hello_world_publisher():
    # create the publisher safely
    with auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)

    i = 0
    while 1:
        data = f"Hello world: {i}"
        i += 1
        print(f"Publishing: {data}")
        pub.publish(String(data=data)) # sends data (lock is not necessary)
        await asyncio.sleep(2) # non-blocking sleep


def main():
    rclpy.init()
    my_session = ThreadedSession(node=Node(node_name="minimal_publisher"))
    set_auto_session(my_session)
    try:
        asyncio.run(hello_world_publisher())
    finally:
        auto_session().close()
        rclpy.shutdown()
```

#### Examine the code

Global initialization is better outside functions that perform work. This time we manually create a `ThreadedSession` (default) using a node named `minimal_publisher`. This is how you can make `afor`'s session run any node you like, then set it as default with `set_auto_session(my_session)` (so you don't need to think about it anymore).

```python
my_session = ThreadedSession(node=Node(node_name="minimal_publisher"))
set_auto_session(my_session)
```

The try finally block will run your async code. After `asyncio.run` returns (due to an error or not), the `finally` block will execute the cleanup.

```python
try:
    asyncio.run(hello_world_publisher())
finally:
    auto_session().close()
    rclpy.shutdown()
```

## Subscriber node

This where `afor` shines, it is specialized in subscribing not publishing. You can follow the ros tutorial and just replace this section with `afor` style code: [Writing a simple publisher and subscriber --- 3 Write the subscriber node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node)

### ROS tutorial code

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### `afor` equivalent code

Let's build upon our improved subscriber `main` function.

```python
import asyncio

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from asyncio_for_robotics.ros2 import (
    ThreadedSession,
    auto_session,
    set_auto_session,
    TopicInfo,
    Sub,
)

TOPIC = TopicInfo(msg_type=String, topic="topic")

async def hello_world_subscriber():
    sub = Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    async for message in sub.listen_reliable():
        print(f"I heard: {message.data}")


def main():
    rclpy.init()
    my_session = ThreadedSession(node=Node(node_name="minimal_subscriber"))
    set_auto_session(my_session)
    try:
        asyncio.run(hello_world_publisher())
    finally:
        auto_session().close()
        rclpy.shutdown()
```

#### Examine the code

That's it, 3 lines, non-blocking. The subscriber is implemented by `afor` so creation with `ros2.Sub` is easier than the previous ROS 2 publisher.

```python
sub = Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
```

The `async for` loop will execute every time a new message is received. If there are not message it will wait. (the only way to exit this loop is to `break`)

```python
async for message in sub.listen_reliable():
```

## Run pubsub in the same session

The problem with ros is "what if Bob made a node, and Gary made a node? how can I fuse their work? Reuse their work?". Let's fuse our pub and sub in one script:

```python
import asyncio

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from asyncio_for_robotics.ros2 import (
    ThreadedSession,
    auto_session,
    set_auto_session,
    TopicInfo,
    Sub,
)

TOPIC = TopicInfo(msg_type=String, topic="topic")

async def hello_world_publisher():
    # create the publisher safely
    with auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)

    i = 0
    while 1:
        data = f"Hello world: {i}"
        i += 1
        print(f"Publishing: {data}")
        pub.publish(String(data=data)) # sends data (lock is not necessary)
        await asyncio.sleep(2) # non-blocking sleep

async def hello_world_subscriber():
    sub = Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    async for message in sub.listen_reliable():
        print(f"I heard: {message.data}")

async def hello_world_pubsub():
    pub_task = asyncio.create_task(hello_world_publisher())
    sub_task = asyncio.create_task(hello_world_subscriber())
    await asyncio.wait([sub_task, pub_task])

def main():
    rclpy.init()
    my_session = ThreadedSession(node=Node(node_name="minimal_pubsub"))
    set_auto_session(my_session)
    try:
        asyncio.run(hello_world_pubsub())
    finally:
        auto_session().close()
        rclpy.shutdown()
```

### Examine the code

Our previous pub and sub code did not change. (No need to touch or know what Bob or Gary's code is doing). All we need to do is execute our previous async functions in an asyncio task then wait for them to (never) finish.

```python
async def hello_world_pubsub():
    pub_task = asyncio.create_task(hello_world_publisher())
    sub_task = asyncio.create_task(hello_world_subscriber())
    await asyncio.wait([sub_task, pub_task])
```

**Note:** You can possibly run both pub and sub in separate sessions thus nodes.
