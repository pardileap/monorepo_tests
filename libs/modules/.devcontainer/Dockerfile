# Use an official Ubuntu runtime as the base image
FROM ubuntu:jammy

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

# # Create a non-root user
# RUN useradd -m $USER_NAME && echo "$USER_NAME:$USER_NAME" | chpasswd && adduser $USER_NAME sudo
# RUN echo "$USER_NAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Install SOEM v1.4.0 (--branch v1.4.0 seems to be broken)
RUN mkdir deps
RUN cd deps && git clone --depth 1  https://github.com/OpenEtherCATsociety/SOEM.git
RUN cd deps/SOEM && mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j4 && sudo make install

WORKDIR /app
COPY . /app


# USER $USER_NAME

# ENV CURRENT_USER=${USEdestR_NAME}


