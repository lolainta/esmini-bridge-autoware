FROM tonychi/eba-autoware:latest

RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/local/bin/

WORKDIR /esmini

ADD https://github.com/esmini/esmini.git .

RUN <<EOF
apt-get update
apt-get -y full-upgrade
apt-get install -y --no-install-recommends \
  build-essential gdb ninja-build git pkg-config libgl1-mesa-dev libpthread-stubs0-dev libjpeg-dev libxml2-dev libpng-dev libtiff5-dev libgdal-dev libpoppler-dev libdcmtk-dev libgstreamer1.0-dev libgtk2.0-dev libcairo2-dev libpoppler-glib-dev libxrandr-dev libxinerama-dev curl cmake black ccache
rm -rf /var/lib/apt/lists/*
EOF

RUN <<EOF
cmake -B build/ -S . \
  -DENABLE_COLORED_DIAGNOSTICS=ON \
  -DENABLE_WARNINGS_AS_ERRORS=ON \
  -DDOWNLOAD_EXTERNALS=OFF \
  -DENABLE_CCACHE=ON \
  -DBUILD_EXAMPLES=OFF \
  -DUSE_IMPLOT=OFF \
  -DUSE_OSG=OFF -DUSE_OSI=OFF -DUSE_SUMO=OFF -DUSE_GTEST=OFF
cmake --build build/ --config Release --target install -j
EOF

WORKDIR /ros_ws
