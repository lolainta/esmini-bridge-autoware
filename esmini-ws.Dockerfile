FROM tonychi/eba-autoware:latest

RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/local/bin/

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /esmini

ADD https://github.com/esmini/esmini.git .

RUN <<EOF
apt-get update
apt-get -y full-upgrade
apt-get install -y --no-install-recommends \
  build-essential gdb ninja-build git pkg-config libgl1-mesa-dev libpthread-stubs0-dev libjpeg-dev libxml2-dev libpng-dev libtiff5-dev libgdal-dev libpoppler-dev libdcmtk-dev libgstreamer1.0-dev libgtk2.0-dev libcairo2-dev libpoppler-glib-dev libxrandr-dev libxinerama-dev curl cmake black ccache
rm -rf /var/lib/apt/lists/*
EOF

RUN cmake -B build/ -S . && cmake --build build/ --config Release --target install -j

RUN <<EOF
apt-get update
apt-get -y full-upgrade
apt-get install -y --no-install-recommends \
  ros-humble-rmw-cyclonedds-cpp
rm -rf /var/lib/apt/lists/*
EOF

WORKDIR /ros_ws
