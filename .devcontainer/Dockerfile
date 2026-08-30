FROM ros:jazzy

ARG USERNAME=admininstrator
ARG USER_UID=1000
ARG USER_GID=1000

# Rename the existing UID 1000 user from ubuntu to admininstrator
RUN usermod --login ${USERNAME} ubuntu && \
    usermod --home /home/${USERNAME} --move-home ${USERNAME} && \
    groupmod --new-name ${USERNAME} ubuntu && \
    usermod --gid ${USERNAME} ${USERNAME}

# Install development tools
RUN apt-get update && \
    apt-get install -y \
        sudo \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Passwordless sudo for the development user
RUN echo "${USERNAME} ALL=(root) NOPASSWD:ALL" \
    > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

ENV SHELL=/bin/bash

USER ${USERNAME}