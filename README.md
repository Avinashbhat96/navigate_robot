# navigate_robot
navigate robot calls move base action to reach goal points

Navigation can be started using server or we can directly start it  using client which  is available.

## Server:
 command to launch the server:
 ```bash
  roslaunch navigate_robot start_server.launch map:="map"
  ```
 if map is different then can be changed from command line.
 Goals  are manually given in the script at the moment
 
 then you can call this action using:
 ```bash
 rostopic pub /navigate_robot/goal navite_robot/NavigateRobotActionGoal "header:
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
  num_goals: 5"
 ```
 here you have to mention how many goals you want to reach. Accordingly it will navigate. you can given any number and if the given number exceeds available goals then it will exit and moves back to starting position.
 
 
 ## Client:
command to start client:
```bash
roslaunch navigate_robot start.launch goals:="2"
```

here goals represents num of goals to be reached. default value is 2.

### To cancel goals

command:
```bash
rostopic pub /navigate_robot/cancel actionlib_msgs/GoalID "stamp:
  secs: 0
  nsecs: 0
id: ''"
```
