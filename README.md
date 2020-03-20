# UAV_zigzag_upgrade

## Upgrade Feature
Add speed limit for no wall condition.

Add Truncate algorithm when maximum height is reached.

## Action to do

1. Please update the previous zigzag script with this new one.
  
    Replace
    
    ```catkin_ws/src/Onboard-SDK-ROS/dji_sdk_demo/src/demo_flight_control_one_plane_zigzag.cpp```
    
    with 
    
    ```demo_flight_control_one_plane_zigzag.cpp```

2. Catkin_make the workspace with the new script.

3. Please make sure the GPS signal is at level 4 before run control_taker.

4. Make sure all the settings are within the range specified in the below section.

## More clarification on setting

```Formula to calculate maximum height:```

```maximum height for the task = initial_height + height_steps * maximum_height ```

**You can adjust setting in catkin_ws/src/configuration.txt to desired setting**


**distance_to_wall**

Description: Distance to maintain to the wall

Restriction: **(2m < distance_to_wall < 5m)** 

If smaller than 2m, the dji app will forbid action. If larger than 5m, the camera fails to find a wall.


**initial_height**

Description: desired starting height of the UAV drone


**height_change_distance**

Description: Horziontal distance change for one zigzag movement

Restriction: **(Less than actually horizontal distance of the wall)**


**height_steps**

Description: height change for each step of the movement


**maximum_height**

Description: Maximum height steps to perform

Restriction: **(Has to be Integer)**

** Words in highlight are the restrictions for the input setting to obey**


**Please let me know if there is more guide that you need: yicui@andrew.cmu.edu **
