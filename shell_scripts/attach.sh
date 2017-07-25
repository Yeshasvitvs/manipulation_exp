#!/bin/bash
#This script opens the yarp rpc port to object attacher gazebo_yarp_plugin and attaches 1R1P object to iCub

ATTACH_DIR=$(pwd)
echo ${ATTACH_DIR}
ATTACH_DIR+="/attach_scripts"
echo ${ATTACH_DIR}
cd ${ATTACH_DIR}
FILES=$(ls)
echo ${FILES}

for f in ${FILES}
do
  echo ./${f}
done
