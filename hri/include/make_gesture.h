#ifndef WAVE_H
#define WAVE_H


// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>

// C++ standard headers
#include <cstdlib>
#include <std_msgs/Bool.h>


// Initialize HRI class
class hri {
	private:
		// Initialize class variables for subscribing and publishing
		ros::Subscriber sub;
		ros::Publisher pub;
		ros::NodeHandle n;

	public:
		// Initialize the constructor and callback function
		hri(ros::NodeHandle& nh);
		void callback(const std_msgs::Bool::ConstPtr&);
        void wave_to_clients();
		void talk_to_clients();	
};

#endif