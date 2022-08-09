#!/bin/bash
echo "RESETTING FOR NAVIGATION..."
bash $(rospack find fetch_vr_backend)/scripts/gripper_reset.bash
bash $(rospack find fetch_vr_backend)/scripts/head_reset.bash
bash $(rospack find fetch_vr_backend)/scripts/torso_up.bash
sleep 5
rosrun fetch_teleop tuck_arm.py
bash $(rospack find fetch_vr_backend)/scripts/torso_down.bash