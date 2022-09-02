FROM debian:buster-slim

RUN apt-get update && apt-get install -y git cmake gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf

ENV CC=/usr/bin/arm-linux-gnueabihf-gcc
ENV CXX=/usr/bin/arm-linux-gnueabihf-g++

# Cross compile hiredis. This is done here instead of in the project because redis-plus-plus expects to find this
# library pre-installed on the system
RUN cd /opt && \
  git clone https://github.com/redis/hiredis.git && \
  cd hiredis && \
  mkdir build && \
  cd build && \
  cmake -D CMAKE_C_COMPILER=/usr/bin/arm-linux-gnueabihf-gcc -D CMAKE_CXX_COMPILER=/usr/bin/arm-linux-gnueabihf-g++ .. && \
  make && \
  make install

RUN apt-get install -y sudo doxygen python3-pip graphviz
RUN pip3 install --upgrade pip && pip3 install sphinx_rtd_theme breathe myst_parser

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