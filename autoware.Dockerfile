FROM ubuntu:22.04

RUN apt update
RUN apt full-upgrade -y
RUN apt install -y git

WORKDIR /autoware
ADD https://github.com/autowarefoundation/autoware.git#2421b2bf2a10c22b103cfe1e0157f198d44d9e1f .
RUN ./setup-dev-env.sh -y --no-nvidia --no-cuda-drivers

RUN mkdir src
RUN vcs import src < autoware.repos
RUN vcs import src < simulator.repos
RUN . /opt/ros/humble/setup.sh && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


RUN wget -qO - 'https://proget.makedeb.org/debian-feeds/prebuilt-mpr.pub' | gpg --dearmor | tee /usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg 1> /dev/null
RUN echo "deb [arch=all,$(dpkg --print-architecture) signed-by=/usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg] https://proget.makedeb.org prebuilt-mpr $(lsb_release -cs)" | tee /etc/apt/sources.list.d/prebuilt-mpr.list
RUN apt update
RUN apt install -y just

WORKDIR /ros_ws
