The code allows to estimate the normal with respect to the robot frame.

In order to build the code:
 catkin_make (catkin_ws)
 
In order to execute the code:
 ./run.sh (/src/local_modal_estimation)

In this case, from the side view of the robot, its frame in the RCM point is 

     ---------------> x
     | \   
     |  \
     |   \
     |    \
     |     y
     z
     

The estimation of the distance need to be used in addition to the robot end effector motion to estimate the plan.

Note: You can visualize the estimation by using a Rviz ros interface.
the name of the rostopic for that purpose:  /plane_normal
**You can launch Rviz by tiping Rviz in the terminal, then you can add, by topics a marker, named /plane_normal

Information:
The code was tested by caspar, however, we can notice that the distance estimation is not working well that makes the convergence not an eary tasks.
Another thing is related to the fact that the distance is limited to 3.8mm so in case of any other application use the dedicated distance limitation


Enjoy!
 

