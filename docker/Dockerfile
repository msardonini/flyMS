FROM debian:buster-slim

RUN apt-get update && apt-get install -y cmake gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf

ARG UNAME=builduser
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME
USER $UNAME
CMD /bin/bash
