FROM eba-autoware

RUN <<EOF
wget -qO - 'https://proget.makedeb.org/debian-feeds/prebuilt-mpr.pub' | gpg --dearmor | tee /usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg 1> /dev/null
echo "deb [arch=all,$(dpkg --print-architecture) signed-by=/usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg] https://proget.makedeb.org prebuilt-mpr $(lsb_release -cs)" | tee /etc/apt/sources.list.d/prebuilt-mpr.list
apt-get update
apt-get install -y --no-install-recommends just
rm -rf /var/lib/apt/lists/*
EOF

ENV YQ_VERSION=v4.2.0
ENV YQ_BINARY=yq_linux_amd64
RUN <<EOF
wget https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/${YQ_BINARY}.tar.gz -O - |
tar xz
mv ${YQ_BINARY} /usr/bin/yq
EOF

COPY ./resources /resources
COPY ./autoware_ws /autoware_ws
WORKDIR /autoware_ws
