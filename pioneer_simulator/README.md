Pioneer simulator does not have any external dependencies
After unpacking the pioneer_simulator, move it 
to directory on the ROS path (for example ~/catkin_ws/src) 
and rosmake it:
		
    $ cd ~/catkin_ws
    $ catkin_make
	
Upon successful building, the pioneer_simulator is ready for use. 


## NODES: ##

						
### pioneerSimulator: ###
This node estimates the motion of a vehicle by receiving velocity
commands via "/cmd_vel" topic and by calculating new vehicle's position 
and orientation. 
The node has been developed only for the simulation purposes and should be
disabled in real applications since the real ROS-based AGV 
vehicles provide their own nodes for execution of velocity commands 
and estimation of the current vehicle pose.

## MAIN TOPICS AND MESSAGES: ##

    
### /initialPose ###
Each vehicle subscribes to this topic in 
the vehicle's namespace in order to recive an initial pose 
information via "geometry_msgs::PoseWithCovarianceStamped" message.

### /cmd_vel ###
Each vehicle subscribes to velocity commands
to this topic in the vehicle's namespace via "geometry_msgs::Twist" 
messages.
	

