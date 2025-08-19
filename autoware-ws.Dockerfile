FROM tonychi/eba-autoware:latest

RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/local/bin/

ENV YQ_VERSION=v4.2.0
ENV YQ_BINARY=yq_linux_amd64
RUN <<EOF
wget https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/${YQ_BINARY}.tar.gz -O - |
tar xz
mv ${YQ_BINARY} /usr/bin/yq
EOF

#COPY ./resources /resources
#COPY ./autoware_ws /autoware_ws
WORKDIR /autoware_ws
