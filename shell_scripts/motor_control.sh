#! /bin/bash



rev() {
  echo "running revolute motor control"
  echo "set vel 0 20" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 10 ]; do
    echo "set pos 0 180" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 16
    echo "set pos 0 0" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 16
    let COUNT=COUNT+1
  done
}


rev_slow() {
  echo "running revolute motor control"
  echo "set vel 0 10" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 10 ]; do
    echo "set pos 0 140" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 5.5
    echo "set pos 0 80" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 5.5
    let COUNT=COUNT+1
  done
}

both_rev_fast1() {
  echo "running revolute motor control"
  echo "set vel 0 200" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 20 ]; do
    echo "set pos 0 140" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 1.65
    echo "set pos 0 80" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 1.65
    let COUNT=COUNT+1
  done
}

both_rev_fast2() {
  echo "running revolute motor control"
  echo "set vel 0 100" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 20 ]; do
    echo "set pos 0 148" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 3.13
    echo "set pos 0 68" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 3.13
    let COUNT=COUNT+1
  done
}

both_rev_fast3() {
  echo "running revolute motor control"
  echo "set vel 0 100" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 20 ]; do
    echo "set pos 0 50" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 1.65
    echo "set pos 0 10" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 1.65
    let COUNT=COUNT+1
  done
}

pri() {
  echo "running prismatic motor control"
  echo "set vel 0 20" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 10 ]; do
    echo "set pos 0 0.1" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 20
    echo "set pos 0 0" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 20
    let COUNT=COUNT+1
  done
}

both_pri_slow() {
  echo "running prismatic motor control"
  echo "set vel 1 20" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 10 ]; do
    echo "set pos 1 0.1" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 20
    echo "set pos 1 0" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 20
    let COUNT=COUNT+1
  done
}

both_pri_fast() {
  echo "running prismatic motor control"
  echo "set vel 1 200" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 20 ]; do
    echo "set pos 1 0.1" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 15
    echo "set pos 1 0" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 15
    let COUNT=COUNT+1
  done
}

both_pri_ext() {
  echo "running prismatic motor control"
  echo "set vel 1 200" | yarp rpc /${OBJECT_NAME}/body/rpc:i
  COUNT=0
  while [ ${COUNT} -lt 20 ]; do
    echo "set pos 1 0.15" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 15
    echo "set pos 1 0" | yarp rpc /${OBJECT_NAME}/body/rpc:i
    sleep 15
    let COUNT=COUNT+1
  done
}

#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################
echo "********************************************************************************"
echo ""

$1 "$2"
OBJECT_NAME=1r_2link
echo ${OBJECT_NAME}
if [[ $# -eq 0 ]] ; then
    echo "[MotorControl] No options were passed!"
    echo ""
    usage
    exit 1
fi
