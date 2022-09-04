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


echo "args are $@"

print_usage



# Argument default values
SOURCE_DIR=/opt
CMAKE_BUILD_TYPE=Release

# Parse command line arguments
# Call getopt to validate the provided input.
options=$(getopt -o dh --long source-dir: -- "$@")
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
  --)
    shift
    break
    ;;
  esac
  shift
done


cmake -S $SOURCE_DIR -B $SOURCE_DIR/build -D CMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE
make -j$('nproc') -C $SOURCE_DIR/build
