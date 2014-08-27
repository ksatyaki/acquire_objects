#include <ros/ros.h>
#include <acquire_objects/AcquireObjects.h>

int main(int argn, char *args[])
{
	ros::init(argn, args, "acquire_objects_client_test");

	ros::NodeHandle nh;
	ros::ServiceClient sc = nh.serviceClient <acquire_objects::AcquireObjects> ("acquire_objects");

	acquire_objects::AcquireObjects message;
	message.request.signature.id = "all";

	//message.request.signature.cluster_size = 6000.00;
	message.request.signature.centroid.x = 0.23;
	message.request.signature.centroid.y = 0.17;
	message.request.signature.centroid.z = 0.86;

	message.request.signature.centroid_tolerance.x = 0.55;
	message.request.signature.centroid_tolerance.y = 0.55;
	message.request.signature.centroid_tolerance.z = 0.05;

	ROS_INFO("Client_calls");
	if (sc.call(message))
	{
		std::cout<<message.response;
	}

	else
	{
		ROS_ERROR("Failed to call service get_objects_from_image");
		return 1;
	}

	return 0;

}
