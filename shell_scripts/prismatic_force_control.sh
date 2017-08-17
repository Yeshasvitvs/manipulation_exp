#! /bin/bash

OBJECT_NAME=1p_2link

pri_one()
{
  echo "running one prismatic force control"
  echo "log start fonetestpdata5.txt" | yarp rpc /manipulationModule/rpc:i
  sleep 0.1
  echo "second_link_handle 0.225 0 0 0 0 0 5" | yarp rpc /${OBJECT_NAME}/applyMultiExternalWrench/rpc:i
  sleep 3
  echo "log stop" | yarp rpc /manipulationModule/rpc:i
}


#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################
echo "********************************************************************************"
echo ""

$1 "$2"

if [[ $# -eq 0 ]] ; then
    echo "[Prismatic Force Control] No options were passed!"
    echo ""
    usage
    exit 1
fi
