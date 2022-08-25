#!/bin/bash
echo -e "\n\n\nResetting for navigation.\nPlease move the robot to a safe spot to tuck the arm. Then Reset."
select val in "Reset" "Exit"; do
  case $val in
    Reset )
        bash $(rospack find fetch_vr_backend)/scripts/gripper_reset.bash
        bash $(rospack find fetch_vr_backend)/scripts/head_reset.bash
        bash $(rospack find fetch_vr_backend)/scripts/torso_up.bash
        sleep 5
        rosrun fetch_teleop tuck_arm.py
        bash $(rospack find fetch_vr_backend)/scripts/torso_down.bash
        break
        ;;
    Exit ) exit;;
  esac
done

echo -e "\nRobot is reset."
bash $(rospack find fetch_vr_backend)/scripts/navigation_localize.bash
echo -e "\n\n\nRobot is ready!"
