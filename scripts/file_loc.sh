#!/bin/bash
# A script to append the locations of all the files of given extension in a given folder into a txt file

# Color to make it look better
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

if [ "$#" -ne 3 ]; then
    echo -e "${RED}[ERROR] Usage: ./file_loc.sh [folder-location] [txt-filename] [extension] ${NC}"
    exit 1
fi

#Argument 1 is the folder where the images are stored
cd "$1"

#Argument 2 is the txt file name
if [ -z "$2" ]; then
	echo -e "${YELLOW}[WARNING] using default filename: loc.txt ${NC}";
	txtname="loc.txt";
else
	txtname="$2";
	echo "[STATUS] using provided filename: $txtname";
fi

#Argument 3 is for the files whose locations need to be saved
if [ -z "$3" ]; then
	echo -e "${YELLOW}[WARNING] writing all files present in the folder ${NC}";
	file_ext="*";
else
	file_ext="$3";
	echo "[STATUS] using provided extensions to save files present in the folder: $file_ext";
fi

for filename in $file_ext
do echo $(pwd)"/"${filename} >> $txtname ;
done

echo -e "${GREEN}[INFO] Run \"$ gedit $txtname\" to view the file ${NC}"
