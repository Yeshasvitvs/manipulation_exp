#! /bin/bash

OBJECT_NAME=1r_2link

rev_slow()
{
  echo "running slow revolute motor control"
  echo "set vel 0 10" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 20 ];
  do
    echo "set pos 0 -180" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 30
    echo "set pos 0 0" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 30
    let COUNT=COUNT+1
  done
}

rev_medium()
{
  echo "running medium revolute motor control"
  echo "set vel 0 50" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 20 ];
  do
    echo "set pos 0 -180" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 15
    echo "set pos 0 0" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 15
    let COUNT=COUNT+1
  done
}

rev_fast()
{
  echo "running fast revolute motor control for ${OBJECT_NAME}"
  echo "set vel 0 100" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 20 ];
  do
    echo "set pos 0 -180" | yarp rpc /${OBJECT_NAME}/body/rpc:i
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

echo "OBJECT NAME : ${OBJECT_NAME}"
if [[ $# -eq 0 ]] ; then
    echo "[Revolute Motor Control] No options were passed!"
    echo ""
    usage
    exit 1
fi
