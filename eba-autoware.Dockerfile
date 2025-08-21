FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN <<EOF
apt-get update
apt-get -y full-upgrade
apt-get install -y --no-install-recommends software-properties-common curl lsb-release
apt-get install -y --no-install-recommends git python3-pip
pip install -U vcstool
rm -rf /var/lib/apt/lists/*
EOF

RUN <<EOF
add-apt-repository universe -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
dpkg -i /tmp/ros2-apt-source.deb

sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

apt update
apt install -y ros-humble-desktop python3-rosdep python3-colcon-common-extensions

rm -rf /var/lib/apt/lists/*
EOF

WORKDIR /autoware
ADD https://github.com/autowarefoundation/autoware.git#7914486293cad67543f36a4edd59dbc6dee56156 /autoware


RUN <<EOF
mkdir src
vcs import src < autoware.repos
vcs import src < simulator.repos
EOF

RUN <<EOF
rosdep init
rosdep update
EOF

RUN <<EOF
. /opt/ros/humble/setup.sh
apt update
rosdep update
rosdep install -yr --from-paths src --ignore-src --rosdistro "$ROS_DISTRO"
rm -rf /var/lib/apt/lists/*
EOF

RUN <<EOF
. /opt/ros/humble/setup.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
EOF

