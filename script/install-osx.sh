#!/bin/sh

#
# Usage: sh install-osx.sh
#

echo "UsTK installation Script"
echo "Installing dependencies..."

brew install cmake
brew install opencv
brew install libxml2
brew install vtk —-with-qt
brew install fftw
brew install armadillo

if [ ! -v USTK_WS ] 
then
    echo "USTK_WS is unset, please set it to point on a directory to put ViSP/UsTK sources and binaries"
elif [ -z "$USTK_WS" ]
then
    echo "USTK_WS is empty, please set it to point on a directory to put ViSP/UsTK sources and binaries"
else
    echo "Getting ViSP source..."
    if [ ! -d "$USTK_WS/visp" ]; then
       git clone https://github.com/lagadic/visp $USTK_WS/visp
    else
       cd $USTK_WS/visp
       git pull origin master
    fi

    if [ ! -d "$USTK_WS/ustk-build" ]; then
       echo "Creating Build directory: $USTK_WS/ustk-build"
       mkdir $USTK_WS/ustk-build
    fi
	
    cd $USTK_WS/ustk-build

    echo "Configuring project with CMake..."
    cmake $USTK_WS/visp -DVISP_CONTRIB_MODULES_PATH=$USTK_WS/ustk -DBUILD_MODULE_visp_ar=OFF -DBUILD_MODULE_visp_blob=OFF -DBUILD_MODULE_visp_detection=OFF -DBUILD_MODULE_visp_klt=OFF -DBUILD_MODULE_visp_mbt=OFF -DBUILD_MODULE_visp_me=OFF -DBUILD_MODULE_visp_tt=OFF -DBUILD_MODULE_visp_tt_mi=OFF -DBUILD_MODULE_visp_vision=OFF -DBUILD_MODULE_visp_visual_features=OFF -DBUILD_MODULE_visp_vs=OFF
	 
    echo "Compiling project"
    make -j4

    echo "Importing ustk-dataset"
    if [ ! -d "$USTK_WS/ustk-dataset" ]; then
       git clone https://github.com/lagadic/ustk-dataset $USTK_WS/ustk-dataset
    else
       cd $USTK_WS/ustk-dataset
       git pull origin master
    fi
    export USTK_DATASET_PATH=$USTK_WS/ustk-dataset
fi

