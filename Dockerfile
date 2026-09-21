FROM ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu/|http://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' /etc/apt/sources.list && \
    sed -i 's|http://packages.ros.org/ros2/ubuntu|http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/|g' /etc/apt/sources.list.d/ros2.list 2>/dev/null || true
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config python3-colcon-common-extensions \
    libgflags-dev nlohmann-json3-dev libgoogle-glog-dev libdw-dev libssl-dev \
    libusb-1.0-0-dev libeigen3-dev libopencv-dev libyaml-cpp-dev libgl1 \
    libegl1 libegl-mesa0 libgl1-mesa-dri libglx-mesa0 libgles2 \
    ros-humble-image-transport ros-humble-image-transport-plugins \
    ros-humble-image-publisher ros-humble-camera-info-manager \
    ros-humble-camera-calibration-parsers ros-humble-cv-bridge \
    ros-humble-diagnostic-updater ros-humble-diagnostic-msgs \
    ros-humble-statistics-msgs ros-humble-backward-ros \
    ros-humble-tf2-sensor-msgs ros-humble-rclcpp-components \
    && rm -rf /var/lib/apt/lists/*
COPY src/ /ros2_ws/src/
COPY repair_sdk_links.py /tmp/repair_sdk_links.py
# Windows checkout represents SDK symlinks as small text files.
RUN python3 /tmp/repair_sdk_links.py /ros2_ws/src/orbbec_camera/SDK/lib
ARG BUILD_JOBS=2
RUN source /opt/ros/humble/setup.bash && cd /ros2_ws && \
    CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} MAKEFLAGS="-j${BUILD_JOBS}" \
    colcon build --executor sequential \
      --packages-select orbbec_camera_msgs orbbec_camera \
      --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
COPY camera.launch.py entrypoint.sh verify_topics.py /opt/femto/
RUN sed -i 's/\r$//' /opt/femto/entrypoint.sh && chmod +x /opt/femto/entrypoint.sh
ENV ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ENTRYPOINT ["/opt/femto/entrypoint.sh"]
CMD ["camera"]
