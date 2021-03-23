#! /bin/bash

if [[ -d "llvm-source-39" ]]; then
    echo "Hello there! We just upgraded AirSim to Unreal Engine 4.18."
    echo "Here are few easy steps for upgrade so everything is new and shiny :)"
    echo "https://github.com/Microsoft/AirSim/blob/master/docs/unreal_upgrade.md"
    exit 1
fi

set -x
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

#Parse command line arguments
downloadHighPolySuv=true
if [[ $1 == "--no-full-poly-car" ]]; then
    downloadHighPolySuv=false
fi

#give user perms to access USB port - this is not needed if not using PX4 HIL
#TODO: figure out how to do below in travis
if [ "$(uname)" == "Darwin" ]; then # osx
    if [[ ! -z "${whoami}" ]]; then #this happens when running in travis
        sudo dseditgroup -o edit -a `whoami` -t user dialout
    fi

    #below takes way too long
    # brew install llvm@3.9
    brew tap llvm-hs/homebrew-llvm
    brew install llvm-5.0

    brew install wget
    brew install coreutils

    export C_COMPILER=/usr/local/opt/llvm-5.0/bin/clang-5.0
    export COMPILER=/usr/local/opt/llvm-5.0/bin/clang++-5.0
else #linux
    if [[ ! -z "${whoami}" ]]; then #this happens when running in travis
        sudo /usr/sbin/useradd -G dialout $USER
        sudo usermod -a -G dialout $USER
    fi

    #install clang and build tools
    sudo apt-get install -y build-essential
    VERSION=$(lsb_release -rs | cut -d. -f1)
    # Since Ubuntu 17 clang-5.0 is part of the core repository
    # See https://packages.ubuntu.com/search?keywords=clang-5.0
    if [ "$VERSION" -lt "17" ]; then
        wget -O - http://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
        sudo apt-get update
    fi
    sudo apt-get install -y clang-5.0 clang++-5.0
    sudo apt-get install -y unzip

    export C_COMPILER=clang-5.0
    export COMPILER=clang++-5.0
fi

#download cmake - we need v3.9+ which is not out of box in Ubuntu 16.04
if [[ ! -d "cmake_build/bin" ]]; then
    echo "Downloading cmake..."
    wget https://cmake.org/files/v3.10/cmake-3.10.2.tar.gz \
        -O cmake.tar.gz
    tar -xzf cmake.tar.gz
    rm cmake.tar.gz
    rm -rf ./cmake_build
    mv ./cmake-3.10.2 ./cmake_build
    pushd cmake_build
    ./bootstrap
    make
    popd
fi

if [ "$(uname)" == "Darwin" ]; then
    CMAKE="$(greadlink -f cmake_build/bin/cmake)"
else
    CMAKE="$(readlink -f cmake_build/bin/cmake)"
fi

# Download rpclib
if [ ! -d "external/rpclib/rpclib-2.2.1" ]; then
    echo "*********************************************************************************************"
    echo "Downloading rpclib..."
    echo "*********************************************************************************************"

    wget  https://github.com/rpclib/rpclib/archive/v2.2.1.zip

    # remove previous versions
    rm -rf "external/rpclib"

    mkdir -p "external/rpclib"
    unzip v2.2.1.zip -d external/rpclib
    rm v2.2.1.zip
fi

# Download high-polycount SUV model
if [ ! -d "Unreal/Plugins/AirSim/Content/VehicleAdv" ]; then
    mkdir -p "Unreal/Plugins/AirSim/Content/VehicleAdv"
fi
if [ ! -d "Unreal/Plugins/AirSim/Content/VehicleAdv/SUV/v1.2.0" ]; then
    if $downloadHighPolySuv; then
        echo "*********************************************************************************************"
        echo "Downloading high-poly car assets.... The download is ~37MB and can take some time."
        echo "To install without this assets, re-run setup.sh with the argument --no-full-poly-car"
        echo "*********************************************************************************************"
        
        if [ -d "suv_download_tmp" ]; then
            rm -rf "suv_download_tmp"
        fi
        mkdir -p "suv_download_tmp"
        cd suv_download_tmp
        wget  https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip
        if [ -d "../Unreal/Plugins/AirSim/Content/VehicleAdv/SUV" ]; then
            rm -rf "../Unreal/Plugins/AirSim/Content/VehicleAdv/SUV"
        fi        
        unzip car_assets.zip -d ../Unreal/Plugins/AirSim/Content/VehicleAdv
        cd ..
        rm -rf "suv_download_tmp" 
    else
        echo "Not downloading high-poly car asset. The default unreal vehicle will be used."
    fi
fi

# Below is alternative way to get cland by downloading binaries
# get clang, libc++
# sudo rm -rf llvm-build
# mkdir -p llvm-build/output
# wget "http://releases.llvm.org/4.0.1/clang+llvm-4.0.1-x86_64-linux-gnu-debian8.tar.xz"
# tar -xf "clang+llvm-4.0.1-x86_64-linux-gnu-debian8.tar.xz" -C llvm-build/output

# #other packages - not need for now
# #sudo apt-get install -y clang-3.9-doc libclang-common-3.9-dev libclang-3.9-dev libclang1-3.9 libclang1-3.9-dbg libllvm-3.9-ocaml-dev libllvm3.9 libllvm3.9-dbg lldb-3.9 llvm-3.9 llvm-3.9-dev llvm-3.9-doc llvm-3.9-examples llvm-3.9-runtime clang-format-3.9 python-clang-3.9 libfuzzer-3.9-dev

#get libc++ source
if [[ ! -d "llvm-source-50" ]]; then 
    git clone --depth=1 -b release_50  https://github.com/llvm-mirror/llvm.git llvm-source-50
    git clone --depth=1 -b release_50  https://github.com/llvm-mirror/libcxx.git llvm-source-50/projects/libcxx
    git clone --depth=1 -b release_50  https://github.com/llvm-mirror/libcxxabi.git llvm-source-50/projects/libcxxabi
else
    echo "folder llvm-source-50 already exists, skipping git clone..."
fi

#build libc++
rm -rf llvm-build
mkdir -p llvm-build
pushd llvm-build >/dev/null


"$CMAKE" -DCMAKE_C_COMPILER=${C_COMPILER} -DCMAKE_CXX_COMPILER=${COMPILER} \
      -LIBCXX_ENABLE_EXPERIMENTAL_LIBRARY=OFF -DLIBCXX_INSTALL_EXPERIMENTAL_LIBRARY=OFF \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=./output \
            ../llvm-source-50

make cxx

#install libc++ locally in output folder
make install-libcxx install-libcxxabi

popd >/dev/null

#install EIGEN library

rm -rf ./AirLib/deps/eigen3/Eigen
echo "downloading eigen..."
wget http://bitbucket.org/eigen/eigen/get/3.3.2.zip
unzip 3.3.2.zip -d temp_eigen
mkdir -p AirLib/deps/eigen3
mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3
rm -rf temp_eigen
rm 3.3.2.zip

popd >/dev/null

set +x
echo ""
echo "************************************"
echo "AirSim setup completed successfully!"
echo "************************************"
