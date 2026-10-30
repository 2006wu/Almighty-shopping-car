FROM ros:noetic-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-rosserial \
    ros-noetic-rosserial-python \
    ros-noetic-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*
COPY launch/ /winter/launch/
CMD ["roslaunch", "/winter/launch/winter_rosserial.launch"]
