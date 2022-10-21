#!/bin/zsh

cd "$(dirname "$0")" || exit 1

set -e

echo "Installing build dependencies ..."

sudo apt-get update
sudo apt-get install -y zsh git build-essential
sudo apt-get install -y cmake gcc-9 g++-9 libomp-dev libboost-all-dev

echo "Done"
echo


if [[ ! -f ../abseil-cpp/CMakeLists.txt ]]; then
    echo "Pulling abseil-cpp submodule ..."

    pushd ../abseil-cpp
    git submodule init
    git submodule update
    popd

    echo "Done"
    echo
fi


echo "Building simulator ..."

./build.sh -f

echo "Done"
echo

[ -x "../build/release/pnet/pnet_sim" ] || { echo 'Simulator not found in build output!'; exit 1 }
echo "Setup done. Simulator is now ready to use."
