###########################################
# docker build
#   -t user/repo:tag      # assign user, repo and tag. if tag is omitted, will default to 'latest'
#   -t user/repo:latest   # (optional) assigning another tag (e.g the 'latest' tag)
#   --target dev          # (optional) build only as far as the specified layer (e.g. dev, custom)
#   --rm                  # (optional) clean up intermediate images
# EITHER
#   - < DockerfileName    # to build from file with no context
# OR
#   -f Dockerfilename .   # build from file with context (. is current pwd)
###########################################
# docker push user/repo:tag
###########################################

ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop AS ros_user_setup

# Create a non-root user
ARG ROS_USER=ros
ARG ROS_USER_UID=1000
ARG ROS_USER_GID=$ROS_USER_UID

# Create and switch to user
RUN groupadd --gid $ROS_USER_GID $ROS_USER \
    && useradd -s /bin/bash --uid $ROS_USER_UID --gid $ROS_USER_GID -m $ROS_USER \
    # Add sudo support for non-root user.
    && apt-get update \
    && apt-get install -y --no-install-recommends sudo \
    && echo $ROS_USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$ROS_USER \
    && chmod 0440 /etc/sudoers.d/$ROS_USER

USER $ROS_USER

###########################################

FROM ros_user_setup AS dev

# Install some dependencies packages
RUN sudo apt-get update && sudo apt-get upgrade -y \
    && sudo apt-get install -y --no-install-recommends \
    # install build tools
    alsa-utils \
    apt-rdepends \
    build-essential \
    git \
    # install useful tools
    ccache \
    wget \
    curl \
    htop \
    vim \
    clang-format \
    # install ros specifics
    libpoco-dev \
    libprotobuf-dev \
    protobuf-compiler \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    # apt clean-up
    && sudo apt-get autoremove -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Install tools for linting
RUN sudo pip3 install --no-cache \
    black==23.9.1 \
    isort==5.12.0 \
    flake8==6.1.0 \
    pep257==0.6.0 \
    colorama \
    colcon-lint
    # colorama is required by flake8 for colorization

# solve issue where the numpy version is binary incompatible
RUN sudo pip3 install --no-cache --upgrade numpy

RUN rosdep update

###########################################

FROM dev as simulation

# Install gazebo and corresponding ros packages.
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
    gz-fortress \
    ros-${ROS_DISTRO}-ign-ros2-control \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-ign-bridge \
    ros-${ROS_DISTRO}-ros-ign-gazebo \
    # apt clean-up
    && sudo apt-get autoremove -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Setup entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bin/bash"]
