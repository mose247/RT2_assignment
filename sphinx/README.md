# RT_final_assignment

Documentation: https://mose247.github.io/RT_final_assignment/

## Exercise requirements
The assignment requires to develop a software architecture to control a mobile robot inside a simulation environment in ROS. In particular, both the modules that simulate the robot and the ones used for localization, mapping and path planning are already available. Therefore, it is asked to implement just the user interface that interacts with the robot controller in order to allow the user to switch between one of the following driving modes:
 1) **_Authonomous drive_**: makes the robot reach autonomously a position (x, y) in the map;
 2) **_Manual drive_**: makes the user directly drive the robot by using the keyboard;
 3) **_Assisted drive_**: makes the user directly drive the robot by using the keyboard while a collision system is running in the background to avoid crashes;


## Installing and runnning
The simulation requires to install [ROS Noetic](http://wiki.ros.org/noetic/Installation), in particular the _ros-noetic-desktop-full_ integration is raccomended so that all the necessary packages to use Gazebo and Rviz are already available. Moreover, you will require the [slam_gmapping](https://github.com/CarmineD8/slam_gmapping) package.

Once that you've everything ready, you have to include the package published in this repository in the src folder of your ROS workspace and build it again. 
Then, provided that you have firstly run the master node with `roscore`, you can start the simulation with the following command:

``
$ roslaunch final_assignment.launch
``

The command launches both the files **_simulation_gmapping_** and **_move_base_**, plus the nodes **_teleop_**, **_user_interface_** and **_collision_avoidance_**. In particular, the latter two are the ones that have been specifically developed for the assignment.


## Software architecture
The architecture of the user interface that was required is highlighted in the following image.

![Assignment_3-11](https://user-images.githubusercontent.com/91455159/149918889-30514938-7aa4-4f80-b46e-69c1e1144db9.jpg)

The idea that lays behind it is that the nodes **_move_base_**, **_teleop_** and **_collision_avoidance_**, that are responsable of setting the robot's velocity, do not publish it directly on the **_/cmd_vel_** topic, which is the one used to move the robot in the simulated environment. Instead, the following intermediate topics have been defined:
 - **_/automatic/cmd_vel_**: on which the node **_move_base_** publishes the velocity computed authomatically to reach an arbitrary point in the map;
 - **_/manual/cmd_vel_**: on which the node **_teleop_** publishes the velocity resulting from the user's input;
 - **_/assisted/cmd_vel_**: on which the node **_collision_avoidance_** publishes the velocity resulting from the user's input, properly updated to avoid collisions (if necessary);

Then, the **_user_interface_** node, that is subscribed to each of the topics above, is in charge of deciding which velocity to publish on **_/cmd_vel_** according to the driving mode selected by the user.


## user_interface node
As it can be noticed from the previous paragraph, this node is the core of the user interface. To implement the functionalities that we've previously stated, after being initialized, the node subscribes to the following topics:
 - **_/automatic/cmd_vel_**
 - **_/manual/cmd_vel_**
 - **_/assisted/cmd_vel_**

Moreover, it is declared a publisher to the **_/cmd_vel_** topic and an action client to tell the node **_move_base_** the goal to be reached when the automatic mode has been selected.

#### main() pseudo-code:
```
initialize two global variables cmd and pub

function main()
  initialize node
  
  sub1= define subscriber to /automatic/cmd_vel that executes callback_1()
  sub2= define subscriber to /manual/cmd_vel that executes callback_2()
  sub3= define subscriber to /assisted/cmd_vel that executes callback_3()
  
  pub= define publisher to /cmd_vel
  
  client= define action client for MoveBasicAction
  set the parameters of the action message that remain constant
  
  while (1)
    print user interface
    cmd= get input from stdin
    
    if (cmd == 1)
      execute automatic mode loop
    else if (cmd == 2)
      execute manual mode loop
    else if (cmd == 3)
      execute assisted mode loop
    else if (cmd == 'q')
      exit from the infinite loop and quit the node
    else
      invalid command error
    end if
    
  end while
end function
```

The callbacks executed by the subscribers are similar: they get the message that was published on the topic they refer to and pusblish it on **_/cmd_vel_**
only when the corresponding driving mode was selected.
```
function callback_i(msg)
  if (cmd == mode i)
    make pub publish msg on /cmd_vel
  end if
end function
```

Moreover, the loop executed whenever the automatic driving mode is selected looks like the following:
```
initialize nav_cmd to navigate in the sub menu

while (nav_cmd is not 'b')
  get the coordinates of the target point via get_goal()
  insert the coordinates in the action message
  make the client sends the action message
  
  while (nav_cmd is not 'g' or 'b')
    nav_cmd= get input from stdin
  end while
  
  cancel current goal
end while

publish a null velocity on /cmd_vel to stop the robot
```

The two loops executed in correspondence of the manual or assisted mode are even simpler. They consists in a busy waiting during which the user can insert commands through the **_teleop_** node to move the robot:
```
initialize nav_cmd to navigate in the sub menu

while (nav_cmd is not 'b')
    nav_cmd= get input from stdin
end while

publish a null velocity on /cmd_vel to stop the robot
```


## collision_avoidance node
The purpose of this node is monitoring the velocity resulting from the user's input and, if necessary, updating it in order to avoid collisions with the external environment. In order to do so, after being initialized, the node subscribes to the following topics:
- **_/manual/cmd_vel_**: to get the velocity resulting from the user's input;
- **_/scan_**: to get information about the position of the robot with respect to surrounding obstacles;

Moreover, after having updated the input velocity, the node publishes it on the **_/assisted/cmd_vel_** topic. 

#### main() pseudo-code:
```
initialize two global variables pub and ass_vel

function main()
  initialize node
  
  sub1= define subscriber to /scan that executes callback_1()
  sub2= define subscriber to /manual/cmd_vel that executes callback_2()
  
  pub= define publisher to /assisted/cmd_vel
  
  spin to execute continuosly the callbacks
end function
```
The two callback function are very simple, here it is the pseudo-code that describes them.
```
function callback_1(msg)
  regions= divide the ranges vector in 5 sections and get the minimum per each
  execute the custom function avoid_collision(regions)
end function
```

```
function callback_2(msg)
  copy the Twist message msg into the global variable ass_vel
end function
```

The real logic of the node is implemented by the utility function _avoid_collision_, which is in charge of updating the input velocity, when necessary, every time that a new LaserScan message is published on the **_/scan_** topic. Its pseudo-code is the following:
```
function avoid_collision(regions)

  lin= get linear component from ass_vel
  ang= get angular component from ass_vel
  
  if (no obstacle detected)
    do nothing
    
  else if (obstacle in the front)
    if (the user wants to go straight)
      lin= 0
      ang= 0
    end if
    
  else if (obstacle in the front-right)
    if (the user wants to turn front-right)
      lin= 0
      ang= 0
    end if
    
  else if (obstacle in the front-left)
    if (the user wants to turn front-left)
      lin= 0
      ang= 0
    end if
    
  else if (obstacle in the front and front-right)
    if (the user wants to go straight or turn front-right)
      lin= 0
      ang= 0
    end if
    
  else if (obstacle in the front and front-left)
    if (the user wants to go straight or turn front-left)
      lin= 0
      ang= 0
    end if
    
  else if (obstacle in the front, front-right and front-left)
    if (the user wants to go straight, turn front-right or front-left)
      lin= 0
      ang= 0
    end if
    
  else if ((obstacle in the front-right and front-left)
    if (the user wants to turn front-right or front-left)
      lin= 0
      ang= 0
    end if
    
  else
    unknown case
  end if
  
  set the linear component of ass_vel to lin
  set the angular component of ass_vel to ang
  publish ass_vel on /assisted/cmd_vel
  
end function
```
