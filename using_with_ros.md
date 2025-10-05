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
# follow the instructions of uv to activate (source .venv/bin/activate)
# !!
uv pip install git+https://github.com/2lian/asyncio-for-robotics.git
# to colcon build you should use
python3 -m colcon build
```

Before running a python script you should source ros and the venv.
```bash
cd <YOUR_WORKSPACE>
. /opt/ros/jazzy/setup.bash
# !!
# follow the instructions of uv to activate the venv
# generally `source .venv/bin/activate`
# !!
```

To build a ros package do not use raw colcon, but:
```bash
cd <YOUR_WORKSPACE>
. /opt/ros/jazzy/setup.bash
# !!
# follow the instructions of uv to activate the venv
# generally `source .venv/bin/activate`
# !!
python3 -m colcon build
```

## Making an application

You do not need to create a ROS 2 package, nor need to use `ros2 run ...` `ros2 launch ...`. But if you want you can. The ROS 2 run process is the same is just python. You specify your entry-point in your `setup.py`, and python will execute the entry-point function.

## Using the node as usual while also using asyncio-for-robotics

...
