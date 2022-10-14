#!/usr/bin/env bash

cd "$(dirname "$0")"

BUILD_CORES=$(nproc)

ROOT_DIR="$(pwd)/.."

VERBOSE=false

# Parse command line arguments
# Source: https://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -f|--force)
    FORCE=true
    shift # past argument
    ;;
    -c|--clean)
    CLEAN=true
    shift # past argument
    ;;
    -d|--debug)
    DEBUG=true
    shift # past argument
    ;;
    -v|--verbose)
    VERBOSE=true
    shift # past argument
    ;;
    *)
    echo "Unrecognized option $key"
    exit 2
esac
done

(#set -x;
set -e
mkdir -p ../build
cd ../build
if [ $DEBUG ]; then
    mkdir -p debug
    cd debug
else
    mkdir -p release
    cd release
fi

BUILD_DIR="$(pwd)"

if [ $FORCE ]; then
    rm -rf *
    if [ $DEBUG ]; then
        cmake -DCMAKE_BUILD_TYPE=Debug ../.. || true
        cmake .
    else
        cmake -DCMAKE_BUILD_TYPE=Release ../.. || true
        cmake .
    fi
fi

if [ $FORCE ] || [ $CLEAN ]; then
    cd $BUILD_DIR && [ -f Makefile ] && make clean
fi

MAKE_FLAGS=" -j $BUILD_CORES"
[ $DEBUG ] && MAKE_FLAGS+=" DEBUG=1" || MAKE_FLAGS+=" DEBUG=0"
[ $VERBOSE ] && MAKE_FLAGS+=" VERBOSE=1"

cd $BUILD_DIR && cmake . && make $MAKE_FLAGS

)

