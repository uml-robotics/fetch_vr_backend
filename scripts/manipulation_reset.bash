#!/bin/bash
echo "\n\n\nResetting for manipulation.\nPlease move the robot to a safe spot to reset the arm. Then Reset."
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

echo "\n\n\nRobot is reset.\nPlease move the robot to the starting square. Then Continue."
select val in "Continue" "Exit"; do
  case $val in
    Continue )
       rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
           seq: 0
           stamp:
             secs: 0
             nsecs: 0
           frame_id: 'map'
         pose:
           pose:
             position: {x: 1.67415261269, y: 2.00741744041, z: 0.0}
             orientation: {x: 0.0, y: 0.0, z: 0.700711730177, w: 0.713444511642}
           covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        break
        ;;
    Exit ) exit;;
  esac
done

echo "\n\n\nRobot is ready!."