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
ARG USER_UID=1003
ARG USER_GID=1004

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt update \
  && apt install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

ENV DEBIAN_FRONTEND=noninteractive
