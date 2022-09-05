
FROM debian:buster-slim as x86_builder

RUN apt-get update && apt-get install -y git cmake g++ libgtest-dev

RUN apt-get install -y sudo doxygen python3-pip graphviz
RUN pip3 install --upgrade pip && pip3 install sphinx_rtd_theme breathe myst_parser


# Compile hiredis. This is done here instead of in the project because redis-plus-plus expects to find this
# library pre-installed on the system
RUN cd /opt && \
  git clone https://github.com/redis/hiredis.git && \
  cd hiredis && \
  mkdir build && \
  cd build && \
  cmake .. && \
  make && \
  make install


# ARG UNAME=builduser
# ARG UID=1000
# ARG GID=1000
# RUN groupadd -g $GID -o $UNAME
# RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME
# USER $UNAME
# CMD /bin/bash

# Copies your code file from your action repository to the filesystem path `/` of the container
COPY entrypoint.sh /entrypoint.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]

# Multi-stage build for cross compiling flyMS
FROM x86_builder as arm_builder

RUN apt-get update && apt-get install -y gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf

ENV CC=/usr/bin/arm-linux-gnueabihf-gcc
ENV CXX=/usr/bin/arm-linux-gnueabihf-g++

# Cross compile hiredis. This will overwrite the x86 version
RUN cd /opt/hiredis/build && \
  rm -rf * && \
  cmake -D CMAKE_C_COMPILER=/usr/bin/arm-linux-gnueabihf-gcc .. && \
  make && \
  make install
