#!/bin/bash
echo -e "\n\n\nResetting for manipulation.\nPlease move the robot to a safe spot to reset the arm. Then Reset."
select val in "Reset" "Exit"; do
  case $val in
    Reset )
        bash $(rospack find fetch_vr_backend)/scripts/gripper_reset.bash
        bash $(rospack find fetch_vr_backend)/scripts/head_reset.bash
        bash $(rospack find fetch_vr_backend)/scripts/torso_up.bash
        sleep 5
        rosrun common_manipulation test_prepare_arm
        break
        ;;
    Exit ) exit;;
  esac
done

echo -e "\nRobot is reset."
bash $(rospack find fetch_vr_backend)/scripts/manipulation_localize.bash
echo -e "\n\n\nRobot is ready!"
