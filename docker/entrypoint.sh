#!/bin/bash

function print_usage {
  echo "entrypoint.sh"
  echo "  Entrypoint for the flyMS docker container"
  echo "Usage:"
  echo "  ./entrypoint.sh [OPTIONS] "
  echo "    OPTIONS:"
  echo "      -d                        Builds with debugging symbols on"
  echo "      --source-dir {source_dir} Specify the source directory to build from"
  echo "      -h                        Prints this help menu and exit"
}

# Argument default values
SOURCE_DIR=/opt
CMAKE_BUILD_TYPE=Release
BUILD_AND_RUN_TESTS=0

# Parse command line arguments
# Call getopt to validate the provided input.
options=$(getopt -o dh --long source-dir: --long test -- "$@")
[ $? -eq 0 ] || {
  echo "Incorrect options provided"
  exit 1
}
eval set -- "$options"
while true; do
  case "$1" in
  -d)
    CMAKE_BUILD_TYPE=Debug
    ;;
  -h)
    # prints the help menu and exit
    print_usage
    exit 0
    ;;
  --source-dir)
    shift; # The arg is next in position args
    SOURCE_DIR=$1
    ;;
  --test)
    BUILD_AND_RUN_TESTS=1
    ;;
  --)
    shift
    break
    ;;
  esac
  shift
done


if [ $BUILD_AND_RUN_TESTS -eq 1 ]; then
  cmake -S $SOURCE_DIR -B $SOURCE_DIR/build_x86 -D CMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE -D BUILD_TESTS=ON -D CROSS_COMPILE=OFF
  make -j$('nproc') -C $SOURCE_DIR/build_x86
  $SOURCE_DIR/build_x86/bin/flyMS_tests

else
  cmake -S $SOURCE_DIR -B $SOURCE_DIR/build -D CMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE -D CMAKE_BUILD_TYPE=$CXX
  make -j$('nproc') -C $SOURCE_DIR/build
fi
