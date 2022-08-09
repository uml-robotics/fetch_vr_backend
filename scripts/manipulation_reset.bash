#!/bin/bash
echo "RESETTING FOR MANIPULATION..."
bash $(rospack find fetch_vr_backend)/scripts/gripper_reset.bash
bash $(rospack find fetch_vr_backend)/scripts/head_reset.bash
bash $(rospack find fetch_vr_backend)/scripts/torso_up.bash
sleep 5
rosrun common_manipulation test_prepare_arm