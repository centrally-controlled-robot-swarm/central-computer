'''
Set up a basic ROS2 package for this
Run a stripped version of nav2_controller running RPP
Convert the output of the local planner to actual wheel velocities
Publish these messages so that the command transmission node can subscribe to them. 
Or, restructure the command transmission node as a callback function that simply send these messages out at every timestamp. 
This restructuring would make things easier, but would also prevent parallelization which would make things slower.
'''