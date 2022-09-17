#!/bin/bash

# TODO: clean up other scripts in this directory

function print_usage {
  echo "flyMS_build.sh"
  echo "  Builds the flyMS flight program from a docker container. Build artifacts are placed in the local 'build'"
  echo "  directory."
  ehco ""
  echo "Usage:"
  echo "  ./flyMS_build.sh [OPTIONS] "
  echo "    OPTIONS:"
  echo "      -c                      Cleans the build directory before making again"
  echo "      -d                      Builds with debugging symbols on"
  echo "      -b                      Builds Docker Image for Build Environment. "
  echo "                              This needs to be run once before flyMS can be built"
  echo "      -h                      Prints this help menu and exits"
  echo "      --test                  Builds and runs the unit tests. This option will build for x84_64"
  echo "                              architecture and save the outputs in a local 'build_x86' directory"
  echo "      --address {address}     Specify the IP address of the beaglebone to send outputs to."
  echo "                              Outputs will be sent to: debian@<address>:/home/debian/bin/"
  echo "      --config {config_file}  Path to a config file to use for flight. This "
  echo "                              will overwrite an existing config file"
}

function build_docker_image {
  docker build --build-arg UID=`id -u` --build-arg GID=`id -g` --target x86_runner -t flyms_builder_x86:buster \
    --network=host ./docker
  docker build --build-arg UID=`id -u` --build-arg GID=`id -g` --target arm_runner -t flyms_builder:buster \
    --network=host ./docker

  if [ $? -ne 0 ]; then
    echo "docker build failed! Exiting"
    exit -1
  fi
}

function dev_flyMS {
  docker run --name flyMS_dev --rm -it -v `pwd`:/opt/flyMS -w /opt/flyMS --entrypoint bash flyms_builder:buster
}

function build_flyMS {
  if [ $BUILD_AND_RUN_TESTS_LOCALLY -ne 0 ]; then
    TEST_COMMAND="--test"
    BASE_DOCKER_IMAGE="flyms_builder_x86:buster"
  else
    TEST_COMMAND=""
    BASE_DOCKER_IMAGE="flyms_builder:buster"
  fi
  SOURCE_DIR=/opt/flyMS

  docker run -v `pwd`:$SOURCE_DIR $BASE_DOCKER_IMAGE --source-dir $SOURCE_DIR $DEBUG_COMMAND $TEST_COMMAND

  if [ $? -ne 0 ]; then
    echo "flyMS build failed! Exiting"
    exit -1
  fi
}

function prep_output_folder {
  [ -d "build" ] && rm -rf products
  mkdir products
  mkdir products/.config
  cp -r build/bin products
  cp -r scripts/* products/bin
  cp -r webserver products/bin
  [ ! -z $1 ] && cp $1 products/.config/flyMSConfig.yaml
}

function rm_output_folder {
  rm -rf products
}

# Defaults
CLEAN=0
CONFIG_FILE=""
ADDRESS=""
BUILD_DOCKER_IMAGE=0
DEBUG_COMMAND=""
BUILD_AND_RUN_TESTS_LOCALLY=0
BUILD_FOLDER="build"

# Parse command line arguments
# Call getopt to validate the provided input.
options=$(getopt -o cbhd --long address: --long config: --long test --long dev -- "$@")
[ $? -eq 0 ] || {
  echo "Incorrect options provided"
  exit 1
}
eval set -- "$options"
while true; do
  case "$1" in
  -c)
    # clean the build directory before compiling
    CLEAN=1
    ;;
  -b)
    BUILD_DOCKER_IMAGE=1
    ;;
  -d)
    DEBUG_COMMAND="-d"
    ;;
  -h)
    # prints the help menu and exit
    print_usage
    exit 0
    ;;
  --config)
    shift;
    CONFIG_FILE=$1
    echo "config $CONFIG_FILE"
    ;;
  --address)
    shift; # The arg is next in position args
    ADDRESS=$1
    ;;
  --test)
    # Compile for x86_64 architecture and run unit tests locally
    BUILD_AND_RUN_TESTS_LOCALLY=1
    BUILD_FOLDER="build_x86"
    ;;
  --dev)
    dev_flyMS
    exit 0
    ;;
  --)
    shift
    break
    ;;
  esac
  shift
done

# Build the Docker Image if requested
if [ $BUILD_DOCKER_IMAGE -ne 0 ]; then
  build_docker_image
fi

# Delete the build folder if it exists if the user requested to clean
[ "$CLEAN" -eq 1 ] && [ -d $BUILD_FOLDER ] && rm -rf $BUILD_FOLDER
# Make a build folder if not present
[ ! -d $BUILD_FOLDER ] && mkdir $BUILD_FOLDER


# Run the function for building flyMS
build_flyMS

# Determine if we should send the outputs to the beaglebone
if [ -n "$ADDRESS" ] && [ $BUILD_AND_RUN_TESTS_LOCALLY -eq 0 ]; then

  if [ -n "$ADDRESS " ] && [ $BUILD_AND_RUN_TESTS_LOCALLY -ne 0 ]; then
    echo "Error! Cannot send outputs to beaglebone when building for x86_64"
    exit -1
  fi

  # Make a folder to store things to send
  prep_output_folder $CONFIG_FILE

  ARCH="$(uname -m)"
  if [ $ARCH == "x86_64" ]; then
    # insctructions for copying files over to the embedded device
    DESTINATION="debian@$ADDRESS:/home/debian"
  elif [ $ARCH == "armv7l" ]; then
    DESTINATION="/home/debian"
  else
      echo "Unsupported CPU architecture!"
      exit 1
  fi

  # rsync the files over
  rsync -auv --info=progress2 products/ debian@$ADDRESS:/home/debian/

  rm_output_folder
fi
