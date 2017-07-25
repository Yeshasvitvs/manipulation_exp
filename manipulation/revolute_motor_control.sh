#! /bin/bash

OBJECT_NAME=1r_2link

rev_slow()
{
  echo "running slow revolute motor control"
  echo "set vel 0 25" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  echo "log start looptestrdata5.txt" | yarp rpc /manipulationModule/rpc:i
  sleep 5
  COUNT=0
  while [ ${COUNT} -lt 5 ];
  do
    echo "set pos 0 95" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 5
    echo "set pos 0 5" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 5
    let COUNT=COUNT+1
  done
  echo "log stop" | yarp rpc /manipulationModule/rpc:i
}

rev_one()
{
  echo "running one revolute motor control once"
  echo "set vel 0 5" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  echo "log start slowonetestrdata1.txt" | yarp rpc /manipulationModule/rpc:i
  sleep 2
  echo "set pos 0 95" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  sleep 25
  echo "log stop" | yarp rpc /manipulationModule/rpc:i
}

#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################
echo "********************************************************************************"
echo ""

$1 "$2"

if [[ $# -eq 0 ]] ; then
    echo "[Revolute Motor Control] No options were passed!"
    echo ""
    usage
    exit 1
fi
