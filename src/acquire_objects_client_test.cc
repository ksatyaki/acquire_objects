#include <ros/ros.h>
#include <acquire_objects/AcquireObjects.h>

int main(int argn, char *args[])
{
	ros::init(argn, args, "acquire_objects_client_test");

	ros::NodeHandle nh;
	ros::ServiceClient sc = nh.serviceClient <acquire_objects::AcquireObjects> ("acquire_objects");

	acquire_objects::AcquireObjects message;
	message.request.name = "all";

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
