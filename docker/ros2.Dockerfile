# ROS 2 Humble image with the actuator driver, the console core (no GUI)
# and the firmware simulator. Works on Apple-silicon Macs (arm64) and x86.
#
#   docker build -f docker/ros2.Dockerfile -t actuator-ros2 .
#   docker run --rm -it actuator-ros2 ros2 launch actuator_driver driver.launch.py \
#       url:=tcp://host.docker.internal:29536
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake python3-yaml python3-colcon-common-extensions \
        ros-humble-diagnostic-msgs \
    && rm -rf /var/lib/apt/lists/*

# Console core (pure Python, only needs PyYAML) and the DBC it decodes with.
COPY protocol /ws/repo/protocol
COPY console /ws/repo/console
ENV PYTHONPATH=/ws/repo/console \
    BRINGUP_DBC=/ws/repo/protocol/actuator.dbc

# Firmware simulator, so the whole chain can run inside the container (CI).
COPY firmware /ws/repo/firmware
RUN cmake -S /ws/repo/firmware -B /ws/repo/firmware/build -DBUILD_TESTS=OFF \
    && cmake --build /ws/repo/firmware/build -j"$(nproc)" --target actuator_sim

# The ROS 2 package.
COPY ros2_driver /ws/src
WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select actuator_driver

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /ws/src/actuator_driver/test/ci_check.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "actuator_driver", "driver.launch.py", "url:=tcp://host.docker.internal:29536"]
