#!/bin/bash
# A script to create the location files of all the dataset in their respective parent directory

# Color to make it look better
LIGHT_GREEN='\033[1;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

cd $(dirname "$0")

#DATA_DIR=$(dirname "$0")"/../cachedir/dataset/NYUv2_data"
#DATA_DIR=$(dirname "$0")"/../cachedir/dataset/AirSim_data"
#DATA_DIR="/data/mittalm/NYUv2_data"
DATA_DIR="/mnt/MM_Seagate_BUP/Fount_Landing/dataset"
OUT_HOME="/home/mayankm/ais_valada/fount_landing/airsim_ws/src"

echo -e "${LIGHT_GREEN}[INFO] Creating location files"
./file_loc.sh ${DATA_DIR}/depth ${OUT_HOME}/zed_depth_path_list.txt *.png
./file_loc.sh ${DATA_DIR}/left ${OUT_HOME}/zed_rgb_path_list.txt *.jpeg
