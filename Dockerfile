FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/*

COPY config/ /site_config/

RUN groupadd --gid $USER_GID $USERNAME || true \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir -p /home/$USERNAME/.config \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME/.config
