#!/bin/bash

echo "export GAZEBO_RESOURCE_PATH=${PWD%/*}:\$GAZEBO_RESOURCE_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${PWD%/*}/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${PWD%/*}/models/AH_hagelslag_melk:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${PWD%/*}/models/AH_hagelslag_puur:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${PWD%/*}/models/AH_thee_mango:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${PWD%/*}/models/AH_thee_bosvruchten:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${PWD%/*}/models/AH_thee:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${PWD%/*}/models/AH_store:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${PWD%/*}/models/legacy_models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${PWD%/*}/models/coffee_cups:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
