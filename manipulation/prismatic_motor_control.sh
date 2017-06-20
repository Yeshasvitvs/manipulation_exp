#! /bin/bash

OBJECT_NAME=1p_2link

pri_slow()
{
  echo "running slow prismatic motor control"
  echo "set vel 0 10" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 20 ];
  do
    echo "set pos 0 0.1" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 6.75
    echo "set pos 0 0" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 6.75
    let COUNT=COUNT+1
  done
}

pri_medium()
{
  echo "running medium prismatic motor control"
  echo "set vel 0 500" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 20 ];
  do
    echo "set pos 0 0.11" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 4
    echo "set pos 0 0" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 4
    let COUNT=COUNT+1
  done
}

pri_fast()
{
  echo "running fast prismatic motor control for ${OBJECT_NAME}"
  echo "set vel 0 100" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 20 ];
  do
    echo "set pos 0 0.1" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 5
    echo "set pos 0 0" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 5
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
