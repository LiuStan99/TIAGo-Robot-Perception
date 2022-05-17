# This is the README for the Human Robot Interaction package
*Author: Stijn Lafontaine*

This package contains all files for the TIAGo robot operating at FESTO as a host. It consists of the following folders:
 - action, this contains all action files that are used to communicate between nodes of other packages
 - include, this contains the header file of node that controls what gestures should be performed
 - launch, this contains all launch files that can be used to test and use the package properly
 - scripts, this contains all python files. The python files implement a node that controls the speech of TIAGo
 - speeches, this contains all text files. These text files contain different speeches that the speech node can call.
 - src, this contains all cpp files. The cpp files implement a node that controls what gestures TIAGo should make 
