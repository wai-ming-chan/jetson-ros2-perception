# ROS 2 Humble for JetPack 5.1.3 / L4T R35.
#
# Base image note: the highest dustynv tag is r35.4.1 while this host is r35.5.0. The skew
# is fine because the NVIDIA container runtime mounts host L4T libraries in, but it is the
# first thing to suspect on any unexplained CUDA/driver error.
#
# The base is Ubuntu 20.04 with ROS 2 Humble built FROM SOURCE -- there are no official
# Humble binaries for focal. Do not try `apt install ros-humble-*`; it will not resolve.
FROM dustynv/ros:humble-desktop-l4t-r35.4.1

# The ROS apt repository baked into this image has an EXPIRED signing key
# (EXPKEYSIG F42ED6FBAB17C654), which makes `apt-get update` fail outright. Drop the
# source rather than re-keying it: this image builds ROS from source precisely because
# packages.ros.org has no Humble binaries for focal, so the repo can supply nothing we
# need. Everything below comes from Ubuntu's own archive.
RUN rm -f /etc/apt/sources.list.d/ros*.list

# GStreamer runtime ships in the base image, but the -dev packages do not, and the camera
# node compiles against appsink. v4l-utils is for introspecting the raw V4L2 path.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Source ROS and, when present, the workspace overlay, for every shell.
RUN echo 'source /opt/ros/humble/install/setup.bash' >> /root/.bashrc && \
    echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' \
        >> /root/.bashrc

CMD ["/bin/bash"]
