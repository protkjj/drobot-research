#!/bin/bash

# 호스트 사용자 UID/GID와 컨테이너 사용자를 맞춤 (파일 권한 문제 방지)
USER_ID=${LOCAL_USER_ID:-1000}
GROUP_ID=${LOCAL_GROUP_ID:-1000}

groupmod -g $GROUP_ID ubuntu
usermod -u $USER_ID -g $GROUP_ID ubuntu
cp -f /etc/skel/.bashrc /home/ubuntu/.bashrc
usermod -aG sudo,video,audio,plugdev,dialout ubuntu 2>/dev/null || true

exec gosu ubuntu "$@"
