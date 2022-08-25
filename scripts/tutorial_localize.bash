#!/bin/bash
echo -e "\n\n\nPlease move the robot to the starting square. Then Continue."
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
      position: {x: 1.64061737061, y: 0.722720503807, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: -0.999992128143, w: 0.00396782707989}
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
        break
        ;;
    Exit ) exit;;
  esac
done
echo -e "\nRobot is localized."