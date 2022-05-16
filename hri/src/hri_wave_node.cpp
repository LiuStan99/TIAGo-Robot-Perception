#include "wave.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "HRI_wave");
  
  
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }
  hri node(nh);
  ROS_INFO("We are up and running");

  ros::spin();
  return 0;
}
