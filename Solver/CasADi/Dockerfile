FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    gcc \
    g++ \
    gfortran \
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    pkg-config \
    git \
    swig \
    liblapack-dev \
    coinor-libipopt-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/src

# ------------------------------------------------------------
# CasADi
# ------------------------------------------------------------

RUN git clone https://github.com/casadi/casadi.git -b main casadi
WORKDIR /usr/src/casadi

RUN mkdir build && cd build && \
    cmake -DPYTHON_EXECUTABLE=/usr/bin/python3 -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON .. && \
    make && \
    make install && \
    ldconfig

# ------------------------------------------------------------
# BVP
# ------------------------------------------------------------
RUN git clone https://github.com/conorleo/QuadControl.git -b main control
WORKDIR /usr/src/control

RUN mkdir build && cd build && \
    cmake .. && \
    make

WORKDIR /usr/src/control/build
# RUN ./multipleShooting
