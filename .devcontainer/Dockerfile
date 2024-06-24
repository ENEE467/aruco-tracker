FROM ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
  build-essential \
  cmake \
  ccache \
  git \
  doxygen \
  gdb \
  wget \
  libeigen3-dev \
  libcppunit-dev \
  python3-pip \
  python3-numpy \
  python3-matplotlib \
  gnuplot \
  unzip

ARG USERNAME=467-dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt update \
  && apt install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && chown $USER_GID:$USER_GID -R /home/$USERNAME

ENV DEBIAN_FRONTEND=noninteractive
