# syntax=docker/dockerfile:1
# ^ heredoc(RUN <<EOF) 문법을 쓰려면 이 지시어가 필요하다.
#   없으면 구형 파서가 heredoc 안의 명령을 Dockerfile 명령으로 오해한다.
# CUDA 12.6 을 쓰는 이유:
# 랩실 데스크탑 드라이버가 570.133.20 (CUDA 12.8 까지 지원)이라
# 13.1 이미지는 nvidia-container-cli 가 'unsatisfied condition: cuda>=13.1' 로 거부한다.
# 드라이버를 올리려면 재부팅이 필요하고 공용 머신이라 이미지 쪽을 낮췄다.
FROM nvidia/cuda:12.6.3-runtime-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN <<EOF
#!/bin/bash
set -e
	# apt 재시도 설정 — ROS2 저장소에서 500MB 넘게 받으므로
	# 패키지 하나가 타임아웃되면 빌드 전체가 날아간다.
	# 실제로 ros-jazzy-rqt-gui 에서 Connection timed out 으로 실패한 적이 있다.
	cat > /etc/apt/apt.conf.d/80-retries <<'CONF'
Acquire::Retries "5";
Acquire::http::Timeout "60";
Acquire::ftp::Timeout "60";
CONF

	apt-get update
	apt-get install -y locales
	locale-gen en_US en_US.UTF-8
	update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
EOF

WORKDIR /tmp/

# ROS2 Jazzy 설치
RUN <<EOF
#!/bin/bash
set -e
	apt-get update
	apt-get -y --quiet --no-install-recommends install \
		software-properties-common curl gosu
	add-apt-repository universe

	# heredoc 안에서는 $ 를 이스케이프하지 않는다.
	# Dockerfile 파서가 heredoc 내용을 쉘에 그대로 넘기므로,
	# \$ 로 쓰면 리터럴 백슬래시가 남아 "Syntax error: ( unexpected" 가 난다.
	export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
		| grep -F "tag_name" | awk -F'"' '{print $4}')
	. /etc/os-release
	CODENAME=${UBUNTU_CODENAME:-${VERSION_CODENAME}}
	curl -L -o /tmp/ros2-apt-source.deb \
		"https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
	dpkg -i /tmp/ros2-apt-source.deb
	apt-get update

	# apt-get install 전체를 재시도한다.
	# Acquire::Retries 는 개별 URL 재시도라, 특정 서버가 계속 응답하지
	# 않으면 소용이 없다. 실제로 546MB 중 ros-jazzy-rqt-gui 하나에서만
	# Connection timed out 이 반복됐다.
	# 이미 받은 패키지는 apt 캐시에 남으므로 재시도 비용이 작다.
	for attempt in 1 2 3 4 5; do
		apt-get -y --quiet --no-install-recommends install \
			ros-dev-tools \
			ros-jazzy-ros-base \
			ros-jazzy-rviz2 \
			ros-jazzy-rqt-gui \
			ros-jazzy-rqt-common-plugins && break
		echo "=== apt 설치 실패, 재시도 ${attempt}/5 (20초 대기) ==="
		sleep 20
		apt-get update || true
	done
	# 마지막 시도까지 실패하면 여기서 멈춘다 (set -e 는 위 루프를 통과하므로 명시적으로 확인)
	dpkg -s ros-jazzy-ros-base > /dev/null 2>&1

	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

# Nav2 + SLAM + Localization + Gazebo 브릿지
RUN <<EOF
#!/bin/bash
set -e
	apt-get update
	for attempt in 1 2 3 4 5; do
		apt-get -y --quiet --no-install-recommends install \
			ros-jazzy-nav2-bringup \
			ros-jazzy-nav2-common \
			ros-jazzy-slam-toolbox \
			ros-jazzy-robot-localization \
			ros-jazzy-ros-gz-sim \
			ros-jazzy-ros-gz-bridge \
			ros-jazzy-xacro \
			ros-jazzy-teleop-twist-keyboard && break
		echo "=== Nav2 설치 실패, 재시도 ${attempt}/5 (20초 대기) ==="
		sleep 20
		apt-get update || true
	done
	dpkg -s ros-jazzy-nav2-bringup > /dev/null 2>&1

	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

# Python 패키지
RUN <<EOF
#!/bin/bash
set -e
	apt-get update
	apt-get -y --quiet --no-install-recommends install python3 python3-pip
	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --break-system-packages -r /tmp/requirements.txt

# GStreamer (Gazebo 렌더링용)
RUN <<EOF
#!/bin/bash
set -e
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
