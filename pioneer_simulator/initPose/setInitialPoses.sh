#!/bin/bash

rostopic pub -r 3 -f Alfa_initialPose.yaml /Alfa/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -r 3 -f Bravo_initialPose.yaml /Bravo/initialpose geometry_msgs/PoseWithCovarianceStamped


