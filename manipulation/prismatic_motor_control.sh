#! /bin/bash

OBJECT_NAME=1p_2link

pri_slow()
{
  echo "running slow prismatic motor control"
  echo "set vel 0 5" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 5 ];
  do
    echo "set pos 0 0.14" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 12
    echo "set pos 0 0.04	" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 12
    let COUNT=COUNT+1
  done
}

pri_one()
{
  echo "running one prismatic motor control"
  echo "set vel 0 0.037" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  echo "log start onetestpdata5.txt" | yarp rpc /manipulationModule/rpc:i
  sleep 2
  echo "set pos 0 0.14" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  sleep 8
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
