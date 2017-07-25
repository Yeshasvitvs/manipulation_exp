#! /bin/bash

OBJECT_NAME=1r_2link

rev_one()
{
  echo "running one revolute force control"
  echo "log start fonetestrdata5.txt" | yarp rpc /manipulationModule/rpc:i
  sleep 0.1
  echo "second_link_handle 0.55 0 0 0 0 0 5" | yarp rpc /${OBJECT_NAME}/applyMultiExternalWrench/rpc:i
  sleep 4.8
  echo "log stop" | yarp rpc /manipulationModule/rpc:i
}


#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################
echo "********************************************************************************"
echo ""

$1 "$2"

if [[ $# -eq 0 ]] ; then
    echo "[Revolute Force Control] No options were passed!"
    echo ""
    usage
    exit 1
fi
