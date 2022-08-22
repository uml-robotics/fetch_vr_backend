#!/bin/bash
echo "\n\n\nResetting for navigation.\nPlease move the robot to a safe spot to tuck the arm. Then Reset."
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

echo "\n\n\nRobot is reset.\nPlease move the robot to the starting square. Then please select the run number."
select val in "1" "2" "3" "Exit"; do
  case $val in
    1 )
      rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
          seq: 0
          stamp:
            secs: 0
            nsecs: 0
          frame_id: 'map'
        pose:
          pose:
            position: {x: 2.74453091621, y: 0.512610197067, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: -0.999975106018, w: 0.00705601479506}
          covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
        break
        ;;
    2 )
      rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
          seq: 0
          stamp:
            secs: 0
            nsecs: 0
          frame_id: 'map'
        pose:
          pose:
            position: {x: 1.67415261269, y: 2.00741744041, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: -0.999975106018, w: 0.00705601479506}
          covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
      break
      ;;
    3 )
      rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
          seq: 0
          stamp:
            secs: 0
            nsecs: 0
          frame_id: 'map'
        pose:
          pose:
            position: {x: 1.64061737061, y: 0.722720503807, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.700711730177, w: 0.713444511642}
          covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
      break
      ;;
    Exit ) exit;;
  esac
done

echo "\n\n\nRobot is ready!."