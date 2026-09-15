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

# ROS2 Jazzy 설치
RUN <<"EOF"
	apt-get update
	apt-get -y --quiet --no-install-recommends install \
		software-properties-common curl gosu
	add-apt-repository universe

	export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
		| grep -F "tag_name" | awk -F'"' '{print $4}')
	curl -L -o /tmp/ros2-apt-source.deb \
		"https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
	dpkg -i /tmp/ros2-apt-source.deb
	apt-get update

	apt-get -y --quiet --no-install-recommends install \
		ros-dev-tools \
		ros-jazzy-ros-base \
		ros-jazzy-rviz2 \
		ros-jazzy-rqt-gui \
		ros-jazzy-rqt-common-plugins

	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

# Nav2 + SLAM + Localization + Gazebo 브릿지
RUN <<"EOF"
	apt-get update
	apt-get -y --quiet --no-install-recommends install \
		ros-jazzy-nav2-bringup \
		ros-jazzy-nav2-common \
		ros-jazzy-slam-toolbox \
		ros-jazzy-robot-localization \
		ros-jazzy-ros-gz-sim \
		ros-jazzy-ros-gz-bridge \
		ros-jazzy-xacro \
		ros-jazzy-teleop-twist-keyboard

	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

# Python 패키지
RUN <<"EOF"
	apt-get update
	apt-get -y --quiet --no-install-recommends install python3 python3-pip
	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --break-system-packages -r /tmp/requirements.txt

# GStreamer (Gazebo 렌더링용)
RUN <<"EOF"
	apt-get update
	apt-get -y --quiet --no-install-recommends install \
		vim python3-gst-1.0 libgstreamer1.0-0 \
		gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
		gstreamer1.0-libav gstreamer1.0-tools \
		gstreamer1.0-x gstreamer1.0-gl
	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

ENV PYTHONUNBUFFERED=1

WORKDIR /app/

COPY ./entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

COPY bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /etc/skel/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh", "bash"]
