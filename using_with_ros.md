# Using with ROS 2

## Preamble

The advantages of `afor` are not evident for simple tasks (one timer, one publisher, one subscriber). Advantages become obvious when those objects need to be composed together. The following tutorial keep things simple, therefore advantages will not be tremendous. The reader is invited to keep this in mind.

[Example directory](./asyncio_for_robotics/example) gives more advanced applications, with more obvious advantages and less blabla.
- For a simple pub/sub: [`ros2_talker.py`](./asyncio_for_robotics/example/ros2_talker.py), [`ros2_listener.py`](./asyncio_for_robotics/example/ros2_listener.py)
- For pub/sub fused in one node: [`ros2_pubsub.py`](./asyncio_for_robotics/example/ros2_pubsub.py)
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

## Subscriber node

This is where `afor` shines, it is specialized in subscribing not publishing.

You can follow the ros tutorial, this section will rewrite only the subscriber code using `afor` coding style: [Writing a simple publisher and subscriber --- 3 Write the subscriber node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node)

### ROS original tutorial code

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

*Note:* Run this example using `python3 -m asyncio_for_robotics.example.ros2_listener`

```python
import asyncio

from std_msgs.msg import String

import asyncio_for_robotics.ros2 as afor

TOPIC = afor.TopicInfo(msg_type=String, topic="topic")


@afor.scoped
async def hello_world_subscriber():
    sub = afor.Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    async for message in sub.listen_reliable():
        print(f"I heard: {message.data}")

def main():
    with afor.auto_context(node="minimal_subscriber"):
        asyncio.run(hello_world_subscriber())

if __name__ == '__main__':
    main()
```

#### Examine the code

The subscriber is created directly with `afor.Sub(...)`. The scope and session
handling stay outside the subscription logic.

```python
async def hello_world_subscriber():
    sub = afor.Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    ...
```

The `async for` loop runs every time a new message is received. If there are no
messages it waits. It does not poll.

```python
    ...
    async for message in sub.listen_reliable():
        print(f"I heard: {message.data}")
```

`afor.auto_context(...)` gives this block a ROS session. The session owns the
executor, the node, and ROS init/shutdown when needed. Inside this block,
`afor.auto_session()` resolves to that lexical session. `@scoped` around the `hello_world_subscriber` will ensure cleanup of any `afor` objects created inside the function.

```python
@afor.scoped
async def hello_world_subscriber():
    ...

def main():
    with afor.auto_context(node="minimal_subscriber"):
        asyncio.run(hello_world_subscriber())
```

## Publisher node

`afor` does not provide a publisher because you can directly use a ROS 2
publisher. This section explains the concept of *session* and how to safely
interact with ROS 2 nodes.

You can follow the ros tutorial, this section will rewrite only the publisher
code using `afor` coding style: [Writing a simple publisher and subscriber --- 2 Write the publisher node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node)

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

*Note:* Run this example using `python3 -m asyncio_for_robotics.example.ros2_talker`

```python
import asyncio

from std_msgs.msg import String

import asyncio_for_robotics.ros2 as afor

TOPIC = afor.TopicInfo(msg_type=String, topic="topic")

@afor.scoped
async def hello_world_publisher():
    # create the publisher safely
    with afor.auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)

    i = 0
    while 1:
        data = f"Hello world: {i}"
        i += 1
        print(f"Publishing: {data}")
        pub.publish(String(data=data)) # sends data (lock is not necessary)
        await asyncio.sleep(0.5) # non-blocking sleep

def main():
    with afor.auto_context(node="minimal_publisher"):
        asyncio.run(hello_world_publisher())

if __name__ == '__main__':
    main()
```

#### Examine the code

The outer `main()` delegates execution to asyncio and gives the block a lexical
ROS session.

```python
def main():
    with afor.auto_context(node="minimal_publisher"):
        asyncio.run(hello_world_publisher())
```

`afor.auto_context(...)` gives this block a ROS session. The session owns the
executor, the node, and ROS init/shutdown when needed. Inside this block,
`afor.auto_session()` resolves to that lexical session.

```python
with afor.auto_context(node="minimal_publisher"):
    asyncio.run(hello_world_publisher())
```

`afor` does not provide a publisher because you can directly use the ROS 2
publisher. I believe it is important to let you, the user, stay in control and
not hide everything behind wrappers. To get the node of the current `afor`
session we use `with afor.auto_session().lock() as node:`. This step is
critical to avoid race conditions in ROS 2. Then we simply declare the
publisher on the node with `node.create_publisher(...)`.

```python
with afor.auto_session().lock() as node:
    pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
```

If you are unfamiliar with multithreading, race conditions, or need to
interact directly with the session's node beyond the safe `afor` methods, use
the less performant `SyncSession` instead.

## Run pubsub in the same session

*Note:* Run this example using `python3 -m asyncio_for_robotics.example.ros2_pubsub`

The problem with ros is "what if Bob made a node, and Gary made a node? how can I fuse their work? Reuse their work?". Let's fuse our pub and sub in one script:

```python
import asyncio

from std_msgs.msg import String

import asyncio_for_robotics.ros2 as afor

TOPIC = afor.TopicInfo(msg_type=String, topic="topic")

async def hello_world_publisher():
    # create the publisher safely
    with afor.auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)

    i = 0
    while 1:
        data = f"Hello world: {i}"
        i += 1
        print(f"Publishing: {data}")
        pub.publish(String(data=data)) # sends data (lock is not necessary)
        await asyncio.sleep(0.5) # non-blocking sleep

async def hello_world_subscriber():
    sub = afor.Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    async for message in sub.listen_reliable():
        print(f"I heard: {message.data}")

@afor.scoped
async def hello_world_pubsub():
    tg = afor.Scope.current().task_group
    assert tg is not None
    tg.create_task(hello_world_subscriber())
    tg.create_task(hello_world_publisher())
    await asyncio.Future()

def main():
    with afor.auto_context(node="minimal_pubsub"):
        asyncio.run(hello_world_pubsub())

if __name__ == '__main__':
    main()
```

### Examine the code

Our previous main, pub and sub code did not change. All we need to do is run
our previous async functions inside the current scope task group, then keep the
scope alive.

```python
async def hello_world_pubsub():
    tg = afor.Scope.current().task_group
    assert tg is not None
    tg.create_task(hello_world_publisher())
    tg.create_task(hello_world_subscriber())
    await asyncio.Future()
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
- In the asyncio thread: Code inside `with session.lock() as node:` is safe (the ros2 thread is *paused* inside this block).
- In the ROS 2 thread: Scheduling an asyncio callback using `asyncio_event_loop.call_soon_threadsafe(callback)` is safe.

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

from rclpy.node import Node
from std_msgs.msg import String

import asyncio_for_robotics.ros2 as afor

async def hello_world_subscriber():
    sub = afor.Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
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
    try:
        with afor.session_context(afor.ThreadedSession(node=MinimalPublisher())):
            asyncio.run(hello_world_subscriber())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This explicit `rclpy.init()` is needed because `MinimalPublisher()` is created
before the session exists. If you only want to choose the node name, prefer
the leaner `with afor.auto_context(node="minimal_publisher"):` form shown
earlier.
