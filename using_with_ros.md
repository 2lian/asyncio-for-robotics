# Using with ROS 2

## Preamble

The advantages of `afor` are not evident for simple tasks (one timer, one publisher, one subscriber). Advantages become obvious when those objects need to be composed together. The following tutorial keep things simple, therefore advantages will not be tremendous. The reader is invited to keep this in mind.

[Example directory](./asyncio_for_robotics/example) gives more advanced applications, with more obvious advantages and less blabla.
- For a simple pub/sub: [`ros2_talker.py`](./asyncio_for_robotics/example/ros2_talker.py) and [`ros2_listener.py`](./asyncio_for_robotics/example/ros2_listener.py)
- For a simple client/server: [`ros2_service_client.py`](./asyncio_for_robotics/example/ros2_service_client.py) and [`ros2_service_server.py`](./asyncio_for_robotics/example/ros2_service_server.py)

## Installing without a venv

```bash
pip install asyncio_for_robotics
```

## Installing with a venv

This is not a tutorial on using a venv with ROS 2 (which is sadly confusing and badly documented).

Before running a python script you should source ros and the venv. To build a ros package do not use raw colcon, but the colcon from you venv:
```bash
cd <YOUR_WORKSPACE>
. /opt/ros/jazzy/setup.bash
virtualenv --system-site-packages .venv # creates venv
source .venv/bin/activate
python3 -m pip install asyncio_for_robotics
python3 -m colcon build
...
```

# Re-doing the ROS 2 tutorial

## Making an application

You do not need to create a ROS 2 package, nor need to use `ros2 run ...` `ros2 launch ...`. But if you want you can. Launch and entry point process do not change when using `afor`.

## Publisher node

`afor` does not provide a publisher because you can directly use a ROS 2 publisher. This section explains the concept of *session* and how to (safely) interact with ROS 2 nodes.

You can follow the ros tutorial, this section will rewrite only the publisher code using `afor` coding style: [Writing a simple publisher and subscriber --- 2 Write the publisher node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node)

### ROS original tutorial code

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
        await asyncio.sleep(0.5) # non-blocking sleep

    auto_session().close()
    rclpy.shutdown()

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()
```

#### Examine the code

Main delegates execution of our `async` function to asyncio.

```python
def main():
    asyncio.run(main_async())
```

`rclpy.init` and `rclpy.shutdown` is still required. Additionally `auto_session().close()` shuts down the background ROS node automatically created by `afor`. This is a core concept, `afor` starts a "Session" made of an executor spinning a node, you can find more about it in the source code `asyncio_for_robotics.ros2.session`.

```python
async def main_async():
    rclpy.init()
    ...
    auto_session().close()
    rclpy.shutdown()
```

`afor` does not provide a publisher because you can directly use the ROS 2 publisher. I believe it is important to let you -- the user -- in control, and not hide everything behind wrappers. To get the node of the current `afor` session we use `with auto_session().lock() as node:`, this step is critical to avoid race conditions in ROS 2 (this is not required for Zenoh and other thread-safe transport protocol). Then we simply declare the publisher on the node with `node.create_publisher(...)`.

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
    await asyncio.sleep(0.5) # non-blocking sleep
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
        await asyncio.sleep(0.5) # non-blocking sleep


def main():
    rclpy.init()
    my_session = ThreadedSession(node=Node(node_name="minimal_publisher"))
    set_auto_session(my_session)
    try:
        asyncio.run(hello_world_publisher())
    finally:
        auto_session().close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Examine the code

Global initialization is better outside functions that perform work, thus main is now responsible for all initialisation. 

This time we manually create a `ThreadedSession` (default) using a basic node named `minimal_publisher`. This is how you can make `afor`'s session run any node you like! Any ros node you already have written can be passed, and it will be spun in `afor`'s background thread.

If you are unfamiliar with MultiThreading, race conditions and need to interact directly with the session's node (without going through safe `afor` methods). I advise you to use the less performant `SyncSession` instead.


Then we set the session as default with `set_auto_session(my_session)`. Every time the session is not provided to `afor`, it will fallback to this default one.

```python
my_session = ThreadedSession(node=Node(node_name="minimal_publisher"))
set_auto_session(my_session)
```

The `try` block will run your async code. After `asyncio.run` returns (due to an error or not), the `finally` block will execute the cleanup.

```python
try:
    asyncio.run(hello_world_publisher())
finally:
    auto_session().close()
    rclpy.shutdown()
```

## Subscriber node

This is where `afor` shines, it is specialized in subscribing not publishing.

You can follow the ros tutorial, this section will rewrite only the subscriber code using `afor` coding style: [Writing a simple publisher and subscriber --- 3 Write the subscriber node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node)

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
        asyncio.run(hello_world_subscriber())
    finally:
        auto_session().close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Examine the code

That's it! The subscriber is safely implemented by `afor` so creation with `ros2.Sub` is easier than the previous ROS 2 publisher. The `async for` loop will execute every time a new message is received. If there are no messages it will wait. (the only way to exit this loop is to `break`)

```python
async def hello_world_subscriber():
    sub = Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    async for message in sub.listen_reliable():
        print(f"I heard: {message.data}")
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
        await asyncio.sleep(0.5) # non-blocking sleep

async def hello_world_subscriber():
    sub = Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    async for message in sub.listen_reliable():
        print(f"I heard: {message.data}")

async def hello_world_pubsub():
    sub_task = asyncio.create_task(hello_world_subscriber())
    pub_task = asyncio.create_task(hello_world_publisher())
    await asyncio.wait([pub_task, sub_task])

def main():
    rclpy.init()
    my_session = ThreadedSession(node=Node(node_name="minimal_pubsub"))
    set_auto_session(my_session)
    try:
        asyncio.run(hello_world_pubsub())
    finally:
        auto_session().close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Examine the code

Our previous main, pub and sub code did not change! (No need to touch or know what Bob or Gary's code is doing). All we need to do is execute our previous async functions in an asyncio task then wait for them to (never) finish.

```python
async def hello_world_pubsub():
    pub_task = asyncio.create_task(hello_world_publisher())
    sub_task = asyncio.create_task(hello_world_subscriber())
    await asyncio.wait([sub_task, pub_task])
```

In ROS 2 you cannot compose functions and tasks so easily.

**Note:** You can possibly run both pub and sub in separate sessions thus nodes.

# How to interface with a non async ROS node

## WARNING

With the `ThreadedSession`, ROS 2 executor and asyncio executor are not running on the same thread! Code running on different threads can lead to race conditions and crashes. 

### Simple solution:

Please use the `asyncio_for_robotics.ros2.SyncSession` to be safe and make all executors run on the same thread. (*Note:* The `SyncSession` decreases performances)

### Advanced solution:

With `ThreadedSession`:
- In asyncio thread: Code inside `with session.lock() as node:` is safe.
- In ROS 2 thread: Scheduling an asyncio callback using `asyncio_event_loop.call_soon_threadsafe(callback)` is safe.

## Safely reading node data

Following the above warning, if using `ThreadedSession`, the node can safely be interacted with using:

```python
with auto_session().lock() as node:
    pub = node.create_publisher(...)
    param = node.get_parameter(...)
    clock = node.get_clock(...)
```

## Spinning a custom node in the session

You can simply pass a node to the session, and the session will spin it.

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

async def hello_world_subscriber():
    sub = Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    async for message in sub.listen_reliable():
        print(f"I heard: {message.data}")

### VVV node from ROS 2 tutorial VVV ###
###                                  ###
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
###                                  ###
### ^^^ node from ROS 2 tutorial ^^^ ###

def main():
    rclpy.init()
    my_session = ThreadedSession(node=MinimalPublisher())
    set_auto_session(my_session)
    try:
        asyncio.run(hello_world_subscriber())
    finally:
        auto_session().close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
