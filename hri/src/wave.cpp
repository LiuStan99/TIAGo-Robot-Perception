#include "wave.h"

// Constructor HRI class
hri::hri(ros::NodeHandle& nh){
    n = nh;
	sub = n.subscribe("/hri_input", 100, &hri::callback, this);
	pub = n.advertise<std_msgs::Bool>("/hri_output", 1000);
}

void hri::callback(const std_msgs::Bool::ConstPtr& do_wave){
    bool ready = do_wave->data;
    ROS_INFO("Message received!");
    if (ready){
        hri::wave_to_clients();
    }
}

void hri::wave_to_clients(){
    actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> client("/play_motion", true);

    ROS_INFO("Waiting for Action Server ...");
    client.waitForServer();

    play_motion_msgs::PlayMotionGoal goal;

    goal.motion_name = "wave";
    goal.skip_planning = false;
    goal.priority = 0;

    ROS_INFO_STREAM("Sending goal with motion: " << "wave");
    client.sendGoal(goal);

    ROS_INFO("Waiting for result ...");
    bool actionOk = client.waitForResult(ros::Duration(30.0));

    actionlib::SimpleClientGoalState state = client.getState();

    if ( actionOk )
    {
        ROS_INFO_STREAM("Action finished successfully with state: " << state.toString());
    }   
    else
    {
        ROS_ERROR_STREAM("Action failed with state: " << state.toString());
    }  
}

void hri::talk_to_clients(){
    ROS_INFO_STREAM("just a placeholder for now, this should make an action file and sent it to the speach node");
}