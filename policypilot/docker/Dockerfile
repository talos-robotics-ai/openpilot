# syntax=docker/dockerfile:1.4
#
# Build context: the policypilot/ repo root (one level above docker/).
#   docker build -t policypilot:latest -f docker/Dockerfile .
#
# This is the same image flavor that g1pilot used (ROS 2 Humble + a
# policypilot-runtime conda env, formerly called robojudov2), with two differences:
#   * The vendored policy framework now lives in /opt/policypilot/policy_runtime
#     (formerly /opt/controlBoard/RoboJuDo).
#   * There is no /opt/controlBoard/teleop — VR/arm teleop is out of scope.

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    ROS_DISTRO=humble \
    DISPLAY=:0 \
    LIBGL_ALWAYS_INDIRECT=0 \
    # policypilot tree
    POLICYPILOT_ROOT=/opt/policypilot \
    POLICY_RUNTIME=/opt/policypilot/policy_runtime \
    # Conda env that runs the policy subprocess.
    POLICYPILOT_RT_PREFIX=/opt/policypilot-runtime \
    POLICYPILOT_RT_PYTHON=/opt/policypilot-runtime/bin/python \
    # Names kept as ROBOJUDO_* because the upstream RoboJuDo source
    # consumes them by these exact names (don't rename without patching
    # policy_runtime/robojudo/).
    ROBOJUDO_ROOT=/opt/policypilot/policy_runtime \
    ROBOJUDO_TASK_DIR=/data/policypilot_runs \
    MAMBA_ROOT_PREFIX=/opt/micromamba

# ---------------------------------------------------------------------------
# OS / ROS packages
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        libpcl-dev \
        terminator \
        git \
        gedit \
        iproute2 \
        iputils-ping \
        wget \
        ca-certificates \
        bzip2 \
        cyclonedds-dev \
        libegl1 libgl1 libosmesa6 \
        libsm6 libxext6 libxrender1 \
        libxkbcommon-x11-0 libxcb-cursor0 libxcb-icccm4 libxcb-image0 \
        libxcb-keysyms1 libxcb-render-util0 libxcb-xinerama0 libxcb-xinput0 libxcb-shape0 \
        python3-gi gstreamer1.0-tools \
        gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip

# ---------------------------------------------------------------------------
# Unitree SDK 2 (Python bindings)
# ---------------------------------------------------------------------------
RUN git clone https://github.com/lnotspotl/unitree_sdk2_python.git /opt/unitree_sdk2_python \
    && cd /opt/unitree_sdk2_python \
    && git checkout 7c661d27f4ae064ffd0dd633fd9d5b518ef0b508 \
    && pip3 install -e .

# ---------------------------------------------------------------------------
# Micromamba (used to install the policypilot-runtime conda env)
# ---------------------------------------------------------------------------
RUN wget -qO- https://micro.mamba.pm/api/micromamba/linux-64/latest \
        | tar -xvj -C /usr/local/bin --strip-components=1 bin/micromamba

# ---------------------------------------------------------------------------
# Vendored RoboJuDo (policy_runtime/) — the policy framework that
# policy_manager spawns as a subprocess.
# ---------------------------------------------------------------------------
COPY policy_runtime ${POLICY_RUNTIME}
COPY docker/vendor /tmp/policypilot-runtime-vendor
COPY docker/policypilot-runtime.environment.yml /tmp/policypilot-runtime.environment.yml
COPY docker/policypilot-runtime.requirements.txt /tmp/policypilot-runtime.requirements.txt

RUN micromamba create -y -p ${POLICYPILOT_RT_PREFIX} -f /tmp/policypilot-runtime.environment.yml \
    && cp -f /tmp/policypilot-runtime-vendor/libddsc.so /usr/local/lib/libddsc.so \
    && ln -sf /usr/local/lib/libddsc.so /usr/local/lib/libddsc.so.0 \
    && ldconfig \
    && cp -a /tmp/policypilot-runtime-vendor/cyclonedds ${POLICYPILOT_RT_PREFIX}/lib/python3.11/site-packages/ \
    && cp -a /tmp/policypilot-runtime-vendor/cyclonedds-0.10.2.dist-info ${POLICYPILOT_RT_PREFIX}/lib/python3.11/site-packages/ \
    && ${POLICYPILOT_RT_PYTHON} -m pip install --upgrade pip \
    && ${POLICYPILOT_RT_PYTHON} -m pip install --no-cache-dir -r /tmp/policypilot-runtime.requirements.txt \
    && ${POLICYPILOT_RT_PYTHON} -m pip install --no-cache-dir --no-deps -e /opt/unitree_sdk2_python \
    && ${POLICYPILOT_RT_PYTHON} -m pip install --no-cache-dir --no-deps -e ${POLICY_RUNTIME} \
    && CYCLONEDDS_HOME=/usr/local PYTHONPATH=${POLICY_RUNTIME} MPLCONFIGDIR=/tmp/matplotlib \
        ${POLICYPILOT_RT_PYTHON} -c "import matplotlib; import pinocchio; from pinocchio import casadi as cpin; import cyclonedds; import unitree_sdk2py; import robojudo" \
    && micromamba clean --all --yes \
    && rm -rf /tmp/policypilot-runtime-vendor \
              /tmp/policypilot-runtime.environment.yml \
              /tmp/policypilot-runtime.requirements.txt

RUN mkdir -p ${ROBOJUDO_TASK_DIR}

# ---------------------------------------------------------------------------
# ROS workspace: livox + astroviz
# ---------------------------------------------------------------------------
RUN mkdir -p /ros2_ws/src \
    && git clone https://github.com/Livox-SDK/livox_ros_driver2.git /ros2_ws/src/livox_ros_driver2

WORKDIR /
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd Livox-SDK2 && mkdir build && cd build \
    && cmake .. && make -j && make install

WORKDIR /ros2_ws/src/livox_ros_driver2
RUN rm -f package.xml \
    && cp package_ROS2.xml package.xml \
    && cp -rf launch_ROS2 launch

WORKDIR /ros2_ws/src
RUN git clone https://github.com/CDonosoK/astroviz_interfaces.git

WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

# ---------------------------------------------------------------------------
# Extra Python deps for the dashboard / IK / camera tools
# ---------------------------------------------------------------------------
RUN pip install pyqt6 pin meshcat evdev pyrealsense2 opencv-python

# ---------------------------------------------------------------------------
# MOLA (LiDAR odometry) — optional, but bundled because the locomotion
# stack relies on /lidar_odometry/pose_fixed.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-mola \
        ros-${ROS_DISTRO}-mola-state-estimation \
        ros-${ROS_DISTRO}-mola-lidar-odometry \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Login shell setup
# ---------------------------------------------------------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash"                  >> /root/.bashrc \
 && echo "source /ros2_ws/install/setup.bash"                        >> /root/.bashrc \
 && echo "export POLICYPILOT_ROOT=${POLICYPILOT_ROOT}"               >> /root/.bashrc \
 && echo "export POLICYPILOT_RT_PREFIX=${POLICYPILOT_RT_PREFIX}"     >> /root/.bashrc \
 && echo "export POLICYPILOT_RT_PYTHON=${POLICYPILOT_RT_PYTHON}"     >> /root/.bashrc \
 && echo "export ROBOJUDO_ROOT=${ROBOJUDO_ROOT}"                     >> /root/.bashrc \
 && echo "export ROBOJUDO_TASK_DIR=${ROBOJUDO_TASK_DIR}"             >> /root/.bashrc \
 && echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH"   >> /root/.bashrc \
 && echo "export ROS_DOMAIN_ID=1"                                    >> /root/.bashrc \
 && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"              >> /root/.bashrc \
 && echo "export CYCLONEDDS_HOME=/usr/local"                         >> /root/.bashrc

CMD ["bash"]
