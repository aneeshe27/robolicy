FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    dbus-x11 \
    ffmpeg \
    git \
    libcppunit-dev \
    mesa-utils \
    python3-catkin-tools \
    python3-future \
    python3-opencv \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-wstool \
    ros-${ROS_DISTRO}-combined-robot-hw \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-joint-state-controller \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-libfranka \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-commander \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-rospy-message-converter \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-xacro \
    sudo \
    swig \
    x11-apps \
    xauth \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

WORKDIR /opt/robolicy_ws/src

COPY external/panda_simulator /opt/robolicy_ws/src/panda_simulator
COPY external/pick-and-place/pick_and_place /opt/robolicy_ws/src/pick_and_place

RUN wstool init . \
    && wstool merge panda_simulator/dependencies.rosinstall \
    && wstool up

RUN git clone --depth 1 https://github.com/boost-ext/sml /tmp/boost_sml_vendor \
    && mkdir -p /opt/robolicy_ws/src/boost_sml/include/boost_sml \
    && cp /tmp/boost_sml_vendor/include/boost/sml.hpp /opt/robolicy_ws/src/boost_sml/include/boost_sml/sml.hpp \
    && printf '%s\n' \
        '<?xml version="1.0"?>' \
        '<package format="2">' \
        '  <name>boost_sml</name>' \
        '  <version>1.0.0</version>' \
        '  <description>Header-only Boost.SML wrapper for catkin builds.</description>' \
        '  <maintainer email="support@example.com">Robolicy</maintainer>' \
        '  <license>MIT</license>' \
        '  <buildtool_depend>catkin</buildtool_depend>' \
        '  <export />' \
        '</package>' \
        > /opt/robolicy_ws/src/boost_sml/package.xml \
    && printf '%s\n' \
        'cmake_minimum_required(VERSION 3.0.2)' \
        'project(boost_sml)' \
        'find_package(catkin REQUIRED)' \
        'catkin_package(INCLUDE_DIRS include)' \
        'include_directories(include ${catkin_INCLUDE_DIRS})' \
        'install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})' \
        > /opt/robolicy_ws/src/boost_sml/CMakeLists.txt \
    && rm -rf /tmp/boost_sml_vendor

RUN cd /opt/robolicy_ws/src/orocos_kinematics_dynamics \
    && git checkout b35c424e

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install -y --from-paths /opt/robolicy_ws/src --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys "python-sip libfranka boost_sml joint_trajectory_controller"

RUN pip3 install --no-cache-dir \
    msgpack \
    osrf-pycommon \
    typing_extensions \
    websockets \
    -r /opt/robolicy_ws/src/pick_and_place/requirements.txt

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd /opt/robolicy_ws \
    && catkin build

COPY docker/ros_entrypoint_pickplace.sh /ros_entrypoint.sh
COPY scripts/run_external_pickplace_stack.sh /usr/local/bin/run_external_pickplace_stack.sh
COPY scripts/record_external_pickplace.sh /usr/local/bin/record_external_pickplace.sh
COPY scripts/capture_external_pickplace_image.sh /usr/local/bin/capture_external_pickplace_image.sh

RUN chmod +x /ros_entrypoint.sh \
    /usr/local/bin/run_external_pickplace_stack.sh \
    /usr/local/bin/record_external_pickplace.sh \
    /usr/local/bin/capture_external_pickplace_image.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
