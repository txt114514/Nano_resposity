FROM ros:humble-perception-jammy 

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    net-tools nautilus bash-completion clangd git gdb nano software-properties-common \
    wget curl build-essential cmake 

RUN mkdir -p part1_ws/src && cd part1_ws/src

WORKDIR /part1_ws/


RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd ./Livox-SDK2/ \
    && mkdir build \
    && cd build \
    && cmake .. && make -j \
    && make install 
WORKDIR /part1_ws/src

RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git 

RUN git clone https://github.com/Ericsii/FAST_LIO.git --recursive 
 
COPY ./part1_pkg ./part1_pkg