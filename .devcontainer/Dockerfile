# Use an official Ubuntu runtime as the base image
FROM osrf/ros:foxy-desktop

# Install necessary packages for C++ development
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    g++ \
    gdb \
    git \
    python3-pip \
    sudo \
    clang-tidy \
    clang-format \
    && rm -rf /var/lib/apt/lists/*

ARG USER_NAME

# Create a non-root user
RUN useradd -m -s /bin/bash ${USER_NAME} && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} && \
    chmod 0440 /etc/sudoers.d/${USER_NAME}

# Switch to the non-root user
USER ${USER_NAME}



