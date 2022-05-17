# This is the README for the Human Robot Interaction package
*Author: Stijn Lafontaine*

This package contains all files for the TIAGo robot operating at FESTO as a host. It consists of the following folders:
 - action 
 - include
 - launch
 - scripts
 - speeches
 - src

In order to test the working of the hri speech, run the following commands (assuming you are in the TIAGo workspace):
> source /opt/ros/melodic/setup.bash

> source ./devel/setup.bash

> roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=titanium world:=empty

> roslaunch hri speech.launch

This launches an empty world, the speech_node and the test_speech_node. 
After 15 seconds of simulation time the test_speech_node will send an action to the speech_node.

### Action
This contains all action files that are used to communicate between nodes of other packages. 
There are two actions:
 - gesture.action
 - speech.action

#### gesture.action
The **goal** has an integer named *situation*, the values have the following meaning:
- 0 : welcome
- 1 : introduction
- 2 : point of interest 1
- 3 : point of interest 2
- 4 : point of interest 3
- 5 : ending
- 6 : not working properly

The **result** has an integer named *result*, the values have the following meaning:
- 0 : failure
- 1 : success


The **feedback** has an integer named *status*, the values have the following meaning:
- 0 : gathering necessary data
- 1 : initiating movement

#### speech.action
The **goal** has an integer named *gesture*, the values have the following meaning:
- 0 : welcome
- 1 : introduction
- 2 : point of interest 1
- 3 : point of interest 2
- 4 : point of interest 3
- 5 : ending
- 6 : not working properly

The **result** has an integer named *result*, the values have the following meaning:
- 0 : failure
- 1 : success


The **feedback** has an integer named *status*, the values have the following meaning:
- 0 : currently speaking

### Include
This contains the header file of node that controls what gestures should be performed, make_gesture.h

### Launch
This contains all launch files that can be used to test and use the package properly.
 - speech.launch
 - world.launch

#### speech.launch
Launches four nodes, test_speech_node, speech_node and tts_to_soundplay and soundplay_node

#### world.launch
Launches the FESTO world launch file from cor_mdp_tiago_gazebo

### Scripts
This contains all python files. The python files implement a node that controls the speech of TIAGo. 
Contains two scripts:
 - speech.py
 - test.py

#### speech.py
Implements all speech functions, is both a server and client
#### test.py
Implements a client that sends actions to speech.py

### Speeches
This contains all text files. These text files contain different speeches that the speech node can call. All files contain a certain speech that can be called

### Src
This contains all cpp files. The cpp files implement a node that controls what gestures TIAGo should make. 
This is still in the making due to recent project changes
