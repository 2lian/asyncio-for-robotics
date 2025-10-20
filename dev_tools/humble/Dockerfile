FROM python:3.10-slim-trixie
FROM osrf/ros:humble-desktop
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
# FROM ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest
SHELL ["/bin/bash", "-c"]

# RUN apt update && apt upgrade -y

# RUN apt install -y ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rmw-zenoh-cpp

# Workspace
WORKDIR /asyncio_for_robotics/
RUN uv -h

COPY pyproject.toml pyproject.toml 
RUN uv venv --system-site-packages
RUN uv pip install .[dev,zenoh]
COPY asyncio_for_robotics asyncio_for_robotics
RUN uv pip install .[dev,zenoh]
COPY tests tests
