#!/bin/bash
rostopic pub -1 /torso_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  trajectory:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    joint_names:
    - 'torso_lift_joint'
    points:
    - positions: [0]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
  path_tolerance:
  - {name: 'torso_lift_joint', position: 0.5, velocity: 1.0, acceleration: 1.0}
  goal_tolerance:
  - {name: 'torso_lift_joint', position: 0.1, velocity: 1.0, acceleration: 1.0}
  goal_time_tolerance: {secs: 0, nsecs: 0}"