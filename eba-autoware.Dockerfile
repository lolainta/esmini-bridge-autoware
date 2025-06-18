FROM ubuntu:22.04 AS autoware

RUN <<EOF
apt-get update
apt-get full-upgrade
apt-get install -y --no-install-recommends git
rm -rf /var/lib/apt/lists/*
EOF

WORKDIR /autoware
#ADD https://github.com/autowarefoundation/autoware.git#2421b2bf2a10c22b103cfe1e0157f198d44d9e1f .
ADD https://github.com/autowarefoundation/autoware.git#1bafa650e6df28c2e087a9664f6b375cd52cf059 .
RUN ./setup-dev-env.sh -y --no-nvidia --no-cuda-drivers

RUN <<EOF
mkdir src
vcs import src < autoware.repos
vcs import src < simulator.repos
EOF

RUN <<EOF
. /opt/ros/humble/setup.sh
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro "$ROS_DISTRO"
EOF

RUN <<EOF
. /opt/ros/humble/setup.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
EOF
