#!/bin/bash
#This script opens the yarp rpc port to object attacher gazebo_yarp_plugin and attaches 1R1P object to iCub

CMD="attachUnscoped floating_base_1R1P_2Link second_link_handle iCub r_m_dummy"

yarp rpc /icubSim/oa/rpc:i ${CMD} & 
PID=$!
echo ${PID}

