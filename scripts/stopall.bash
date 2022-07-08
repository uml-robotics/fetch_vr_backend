#!/bin/bash

rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "stamp:
  secs: 0
  nsecs: 0
id: ''"

rostopic pub -1 /move_group/cancel actionlib_msgs/GoalID "stamp:
  secs: 0
  nsecs: 0
id: ''"
