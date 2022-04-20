#!/bin/bash

SCRIPT=`realpath $0`
SCRIPT_PATH=`dirname $SCRIPT`
PACKAGE_PATH=`dirname $SCRIPT_PATH`
echo "export GAZEBO_RESOURCE_PATH=$PACKAGE_PATH:\$GAZEBO_RESOURCE_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$PACKAGE_PATH/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$PACKAGE_PATH/models/AH_hagelslag_melk:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$PACKAGE_PATH/models/AH_hagelslag_puur:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$PACKAGE_PATH/models/AH_thee_mango:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$PACKAGE_PATH/models/AH_thee_bosvruchten:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$PACKAGE_PATH/models/AH_thee:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$PACKAGE_PATH/models/AH_store:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$PACKAGE_PATH/models/legacy_models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$PACKAGE_PATH/models/coffee_cups:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
