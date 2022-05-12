### Motion Manager Package 

## Motion_manager.py
idea: a node to help coordinate and translate motion requensts from decision tree, navigation and HRI nodes.

currently: converts /xyz topic messages to /move_base_simple/goal topic messages.

packages/nodes of intrest: 
* /move_base
* navigation
* moveit

can be run using:
```
rosrun motion_manager motion_manager.py
```

## random_position_generator.py

a debug node to generate a random position orientation around the origin ot the map.

is run using:  
```
rosrun motion_manager random_position_generator.py
```
