#! /bin/bash

OBJECT_NAME=1p_2link

pri_slow()
{
  echo "running slow prismatic motor control"
  echo "set vel 0 10" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 10 ];
  do
    echo "set pos 0 0.15" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 12
    echo "set pos 0 0.01" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 12
    let COUNT=COUNT+1
  done
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
