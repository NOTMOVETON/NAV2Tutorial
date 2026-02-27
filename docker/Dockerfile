# -- user args
ARG UID=1000
ARG GID=1000
ARG DEV_USER=ubuntu

# -- ros args
ARG ROS_DISTRO=jazzy
ARG REQ_DIR=docker/requirements

# -- sys args
ARG DEBIAN_FRONTEND=noninteractive
ARG LANG=en_US.UTF-8
ARG TZ=Etc/UTC


### === BASE SYSTEM STAGE ===
FROM ubuntu:24.04 AS base

ARG REQ_DIR
ARG DEBIAN_FRONTEND
ARG TZ
ARG LANG

ENV LANG=$LANG
ENV TZ=$TZ
ENV DEBIAN_FRONTEND=$DEBIAN_FRONTEND

# 1) Base packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common curl gnupg lsb-release sudo \
        locales tzdata python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen "${LANG}" && \
    update-locale LC_ALL="${LANG}" LANG="${LANG}"

RUN ln -snf "/usr/share/zoneinfo/${TZ}" /etc/localtime && \
    echo "${TZ}" > /etc/timezone

# 2) ROS2 keyring + repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# 3) APT requirements
COPY --link ${REQ_DIR}/apt-requirements.txt /tmp/apt-requirements.txt

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        $(cat /tmp/apt-requirements.txt) && \
    rm -rf /var/lib/apt/lists/* /tmp/apt-requirements.txt

# 4) ROS requirements
COPY --link ${REQ_DIR}/ros-requirements.txt /tmp/ros-requirements.txt

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        $(cat /tmp/ros-requirements.txt) && \
    rm -rf /var/lib/apt/lists/* /tmp/ros-requirements.txt

# 5) Python requirements
COPY --link ${REQ_DIR}/python-requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir \
    --break-system-packages -r /tmp/requirements.txt && \
    rm /tmp/requirements.txt


### === RUNTIME STAGE ===
FROM base AS runtime

ARG DEV_USER
ARG ROS_DISTRO
ARG WORKDIR=/ros2_ws

ENV ROS_DISTRO=$ROS_DISTRO
ENV DEV_USER=$DEV_USER

# ---------- Create workspace ----------
RUN mkdir -p $WORKDIR/src
WORKDIR $WORKDIR

# ---------- rosdep ----------
RUN rosdep init || true && \
    for i in 1 2 3 4 5; do \
        rosdep update && break || (echo "rosdep update failed, retry $i/5" && sleep 5); \
    done

# ---------- Create user & permissions ----------
RUN (id -u ${DEV_USER} >/dev/null 2>&1 || \
    useradd -m -s /bin/bash ${DEV_USER}) && \
    (getent group video   || groupadd -g 44 video) && \
    (getent group dialout || groupadd -g 20 dialout) && \
    usermod -aG video,dialout,sudo ${DEV_USER} && \
    echo "${DEV_USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${DEV_USER} && \
    chown -R ${DEV_USER}:${DEV_USER} ${WORKDIR}

# ---------- Final setup ----------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
RUN apt-get update && apt-get install -y \
    gosu && \
    rm -rf /var/lib/apt/lists/*

# --- Add ROS sources to dev user ---
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${DEV_USER}/.bashrc && \
    echo "source $WORKDIR/install/setup.bash || true" >> /home/${DEV_USER}/.bashrc && \
    chown ${DEV_USER}:${DEV_USER} /home/${DEV_USER}/.bashrc

ENTRYPOINT ["/usr/bin/env", "bash", "/entrypoint.sh"]
CMD ["bash"]
