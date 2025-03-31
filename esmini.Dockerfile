FROM ros:jazzy

RUN apt update

RUN apt install -y build-essential gdb ninja-build git pkg-config libgl1-mesa-dev libpthread-stubs0-dev libjpeg-dev libxml2-dev libpng-dev libtiff5-dev libgdal-dev libpoppler-dev libdcmtk-dev libgstreamer1.0-dev libgtk2.0-dev libcairo2-dev libpoppler-glib-dev libxrandr-dev libxinerama-dev curl cmake black

ADD https://github.com/esmini/esmini.git /esmini

WORKDIR /esmini

RUN cmake -B build/ -S .
RUN cmake --build build/ --config Release --target install

# RUN apt install -y python3-pip
RUN apt install -y python3-protobuf

RUN apt install -y just

RUN apt install -y ccache ros-jazzy-rmw-cyclonedds-cpp
RUN /usr/sbin/update-ccache-symlinks


WORKDIR /ros_ws

ENTRYPOINT ["sleep", "infinity"]


