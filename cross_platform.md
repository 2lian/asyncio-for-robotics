# Cross-Platform deployment

| Python <br>`3.10`, `3.11`, <br>`3.12`, `3.13`, <br>`3.14`  | ROS 2 `humble`, <br>`jazzy`, `kilted` |
|:---:|:---:|
| `ubuntu`, <br>`windows`, <br>`macos` | `ubuntu`,<br>`windows` |
| [![Python - crossplatform](https://github.com/2lian/asyncio-for-robotics/actions/workflows/python-pixi.yml/badge.svg)](https://github.com/2lian/asyncio-for-robotics/actions/workflows/python-pixi.yml) | [![ROS 2 - crossplatform](https://github.com/2lian/asyncio-for-robotics/actions/workflows/ros-pixi.yml/badge.svg)](https://github.com/2lian/asyncio-for-robotics/actions/workflows/ros-pixi.yml) |

## How?

Asyncio is platform-agnostic, thus `asyncio_for_robotics` should be able to run on any Python... which is the case! Our cross-platform CI tests are verifying that.

Wouldn't it be great if you could just install ROS 2 in a virtual environment on any machine? Then install `asyncio_for_robotics` to be left with only a native python interface able to send messages on ROS? 

[The Pixi package manager](https://pixi.sh) allows you to do that! Yes you can use ROS `humble`, `jazzy`, `kilted` on any Linux and even Windows. Pixi shares our vision of making ROS 2 and robotics more accessible.

## Try it out

(You might have PTSD from the ROS installation process, it's okay, don't be scared anymore)

1. Install pixi: https://pixi.sh/latest/installation/
  - Linux/Mac: `curl -fsSL https://pixi.sh/install.sh | sh`
  - Windows: `powershell -ExecutionPolicy ByPass -c "irm -useb https://pixi.sh/install.ps1 | iex"`
2. Clone this repo and `cd` inside with you terminal
3. Run the examples! `pixi run -e jazzy python3 -m asyncio_for_robotics.example.ros2_pubsub`
  - Installation `pixi install` is done automatically before running.
  - You can replace `jazzy` with other ROS distros: `humble`, `jazzy`, `kilted`. Or if you use `zenoh` you will only have zenoh.


## For your project

Do not import, depend or inherit from our `pixi.toml`. Our work is a pure Python package distributed through `PyPI`. However, please take inspiration from our `pixi.toml`, and read the Pixi + Robostack tutorials for your ROS project.

To add ROS 2 and `asyncio_for_robotics` to your Pixi environment, simply to add our `PyPI` dependency to a [ROS Pixi environment](https://pixi.sh/latest/tutorials/ros2/):

```toml
[pypi-dependencies]
asyncio-for-robotics = "*"
```
