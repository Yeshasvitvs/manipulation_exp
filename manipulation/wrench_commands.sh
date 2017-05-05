#! /bin/bash
c1() {
  echo "running c1 wrenches"
  echo "first_link_handle 100 0 0 0 0 0 100" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
  echo "second_link_handle -100 0 0 0 0 0 100" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
}

c1_r() {
  echo "running c1_r wrenches"
  echo "first_link_handle -100 0 0 0 0 0 100" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
  echo "second_link_handle 100 0 0 0 0 0 100" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
}

c_down() {
  echo "running c_down wrenches"
  echo "first_link_handle 0 0 -10 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
  echo "second_link_handle 0 0 -10 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
}

c2() {
  echo "running c2 wrenches"
  echo "first_link_handle 10 0 0 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
  echo "second_link_handle -10 0 0 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
}

c2_r() {
  echo "running c2_r wrenches"
  echo "first_link_handle -10 0 0 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
  echo "second_link_handle 10 0 0 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
}

couple_xy() {
  echo "running couple_xy wrenches"
  echo "first_link_handle -12 0 0 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
  echo "second_link_handle 10 0 0 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
}

couple_xy_r() {
  echo "running couple_xy_r wrenches"
  echo "first_link_handle  11 0 0 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
  echo "second_link_handle -13 0 0 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
}

couple_xz() {
  echo "running couple_xz wrenches"
  echo "first_link_handle -5 0 0.5 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
  echo "second_link_handle 6 0 -1 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
}

couple_xz_r() {
  echo "running couple_xz_r wrenches"
  echo "first_link_handle  7 0 -0.5 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
  echo "second_link_handle -4 0 1 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
}

run() {
  couple_xz
  sleep 2
  couple_xz_r
  sleep 2
  couple_xy
  sleep 2
  couple_xy_r
  sleep 2
}


wall_couple(){
  echo "running wall_couple wrenches"
  echo "second_link_handle  0 -13 0 0 0 0 3" | yarp rpc /wall_fb_1r1p/applyMultiExternalWrench/rpc:i
   sleep 10
  echo "second_link_handle  0  10 0 0 0 0 3" | yarp rpc /wall_fb_1r1p/applyMultiExternalWrench/rpc:i
}

fb_Rcouple(){
  echo "running wall_couple wrenches"
  echo "second_link_handle  0  100 0 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
  sleep 8
  echo "second_link_handle  0 -100 0 0 0 0 10" | yarp rpc /floating_base_1R1P_2Link/applyMultiExternalWrench/rpc:i
}


#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################
echo "********************************************************************************"
echo ""

$1 "$2"

if [[ $# -eq 0 ]] ; then 
    echo "[BlinkingScript] No options were passed!"
    echo ""
    usage
    exit 1
fi

