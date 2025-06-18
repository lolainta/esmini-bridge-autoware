FROM eba-autoware

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /esmini
ADD https://github.com/esmini/esmini.git .
RUN <<EOF
apt-get update
apt-get full-upgrade
apt-get install -y --no-install-recommends \
  build-essential gdb ninja-build git pkg-config libgl1-mesa-dev libpthread-stubs0-dev libjpeg-dev libxml2-dev libpng-dev libtiff5-dev libgdal-dev libpoppler-dev libdcmtk-dev libgstreamer1.0-dev libgtk2.0-dev libcairo2-dev libpoppler-glib-dev libxrandr-dev libxinerama-dev curl cmake black ccache
rm -rf /var/lib/apt/lists/*
EOF

RUN cmake -B build/ -S . && cmake --build build/ --config Release --target install

RUN <<EOF
wget -qO - 'https://proget.makedeb.org/debian-feeds/prebuilt-mpr.pub' | gpg --dearmor | tee /usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg 1> /dev/null
echo "deb [arch=all,$(dpkg --print-architecture) signed-by=/usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg] https://proget.makedeb.org prebuilt-mpr $(lsb_release -cs)" | tee /etc/apt/sources.list.d/prebuilt-mpr.list
apt-get update
apt-get install -y --no-install-recommends just
rm -rf /var/lib/apt/lists/*
EOF

WORKDIR /ros_ws
ENTRYPOINT ["sleep", "infinity"]
