FROM nvidia/cuda:13.1.0-runtime-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN <<"EOF"
	apt-get update
	apt-get install -y locales
	locale-gen en_US en_US.UTF-8
	update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
EOF

WORKDIR /tmp/

# PX4 빌드 의존성 (PX4-Autopilot/Tools/setup/ubuntu.sh 참고)
RUN <<"EOF"
	apt-get update
	apt-get -y --quiet --no-install-recommends install \
		ca-certificates gnupg gosu software-properties-common sudo wget curl
	apt-get -y --quiet --no-install-recommends install \
		astyle build-essential ccache cmake cppcheck file g++ gcc gdb git \
		lcov libssl-dev libxml2-dev libxml2-utils make ninja-build \
		python3 python3-dev python3-pip python3-setuptools python3-wheel \
		rsync shellcheck unzip zip
	apt-get -y --quiet --no-install-recommends install \
		autoconf automake bison bzip2 flex genromfs gperf \
		libncurses-dev libncurses6 libncursesw6 libtool pkg-config \
		uncrustify vim-common
	apt-get -y --quiet --no-install-recommends install \
		g++-multilib gcc-arm-none-eabi gcc-multilib

	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

# Python 패키지 (PX4 빌드용)
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --break-system-packages -r /tmp/requirements.txt

# Gazebo Harmonic
RUN <<"EOF"
	curl https://packages.osrfoundation.org/gazebo.gpg \
		--output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
	echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
		https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
		| tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
	apt-get update
	apt-get -y --quiet --no-install-recommends install \
		gz-harmonic libunwind-dev cppzmq-dev dmidecode \
		gstreamer1.0-plugins-bad gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly \
		gstreamer1.0-libav libeigen3-dev \
		libgstreamer-plugins-base1.0-dev libimage-exiftool-perl \
		libopencv-dev libxml2-utils pkg-config protobuf-compiler

	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

# Micro-XRCE-DDS Agent (PX4 <-> ROS2 통신 브릿지)
RUN <<"EOF"
	git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
	sed -i '98s|2.12|2.13|' Micro-XRCE-DDS-Agent/CMakeLists.txt
	sed -i '99s|2.12.x|2.13.3|' Micro-XRCE-DDS-Agent/CMakeLists.txt
	cd Micro-XRCE-DDS-Agent && \
		mkdir build && cd build && \
		cmake .. && make -j$(nproc) && make install && \
		ldconfig /usr/local/lib/
EOF

WORKDIR /app/

COPY ./entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# 편의 alias
RUN <<"EOF"
cat << 'BASHRC' >> /etc/skel/.bashrc

# PX4 SITL 실행 (drobot 커스텀 모델은 추후 설정)
alias dds="MicroXRCEAgent udp4 -p 8888"
BASHRC
EOF

# Gazebo 리소스 경로 (PX4 빌드 후 활성화)
# PX4-Autopilot은 컨테이너 내에서 별도로 클론/빌드 필요:
#   git clone --recursive https://github.com/PX4/PX4-Autopilot.git
#   cd PX4-Autopilot && make px4_sitl gz_x500
ENV GZ_SIM_RESOURCE_PATH=/app/PX4-Autopilot/Tools/simulation/gz/worlds/:/app/PX4-Autopilot/Tools/simulation/gz/models/:/app/gz/worlds/:/app/gz/models/
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/app/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins/

ENTRYPOINT ["/usr/local/bin/entrypoint.sh", "bash"]
