# UAV_zigzag_upgrade

## Upgrade Feature
Add speed limit for no wall condition.

Add Truncate algorithm when maximum height is reached.

## Action to do

Please update the previous zigzag script with this new one.

Catkin_make the workspace with the new script.

Please make sure the GPS signal is at level 4 before run control_taker.

## More clarification on setting


```maximum height for the task = initial_height + height_steps * maximum_height ```

distance_to_wall **(2m < setting < 5m)**:distance to maintain to wall

initial_height : starting height of the UAV drone

height_change_distance **(Less than actually horizontal distance of the wall)**: horziontal distance change

height_steps : height change for each step of the movement

maximum_height **(Integer)**: maximum height steps to perform


** Words in highlight are the restrictions for the input setting **


**Please let me know if there is more guide that you need: yicui@andrew.cmu.edu **
