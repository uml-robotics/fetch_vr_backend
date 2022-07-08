#!/bin/bash
rostopic pub -1 /head_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal "header:
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
    - 'head_pan_joint'
    - 'head_tilt_joint'
    points:
    - positions: [0,0]
      velocities: [1,1]
      accelerations: [1,1]
      effort: [0,0]
      time_from_start: {secs: 0, nsecs: 0}
  path_tolerance:
  - {name: '', position: 0.0, velocity: 0.0, acceleration: 0.0}
  goal_tolerance:
  - {name: '', position: 0.0, velocity: 0.0, acceleration: 0.0}
  goal_time_tolerance: {secs: 0, nsecs: 0}"
