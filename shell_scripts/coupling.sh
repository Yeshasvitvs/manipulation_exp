#! /bin/bash

echo "********************************************************************************"
echo "Coupling iCub and the Manipulator"

echo "attachUnscoped 1r_2link first_link_handle iCub l_hand " | yarp rpc  /icubSim/oa/rpc:i
sleep 5
echo "attachUnscoped 1r_2link second_link_handle iCub r_hand " | yarp rpc  /icubSim/oa/rpc:i

#sleep 5
#echo "enableGravity 1r_2link 1" | yarp rpc  /icubSim/oa/rpc:i
