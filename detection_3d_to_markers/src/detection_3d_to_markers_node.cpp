#include "detection_3d_to_markers.hpp"

int main(int argc, char * * argv) {
	ros::init(argc, argv, "detection_to_3d_markers");

	ros::NodeHandle node_handle;

	robotics_practicals::Detection3dToMarkers detection_3d_to_markers(node_handle);

	ros::spin();

	return 0;
}
