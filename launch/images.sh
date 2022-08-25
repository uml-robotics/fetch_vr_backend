#!/bin/bash
rosrun image_transport republish raw in:=/spot/camera/frontleft/image compressed out:=/spot/camera/frontleft/image &
rosrun image_transport republish raw in:=/spot/camera/frontright/image compressed out:=/spot/camera/frontright/image &
rosrun image_transport republish raw in:=/spot/camera/left/image compressed out:=/spot/camera/left/image &
rosrun image_transport republish raw in:=/spot/camera/right/image compressed out:=/spot/camera/right/image &
rosrun image_transport republish raw in:=/spot/camera/back/image compressed out:=/spot/camera/back/image &
rosrun image_transport republish raw in:=/spot/camera/hand_color/image compressed out:=/spot/camera/hand_color/image &
echo Loaded images!