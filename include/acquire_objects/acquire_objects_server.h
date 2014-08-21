/*
 * acquire_objects_server.h
 *
 *  Created on: Aug 20, 2014
 *      Author: ace
 */

#ifndef ACQUIRE_OBJECTS_SERVER_H_
#define ACQUIRE_OBJECTS_SERVER_H_

#include <math.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

#include <acquire_objects/AcquireObjects.h>
#include <doro_msgs/RecognizedObjectArray.h>
#include <doro_msgs/ClusterArray.h>

#define DIST2D(x1, y1, x2, y2) sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) )
/**
 * The class that uses the output of obj_detection
 * and cluster_extraction to create signatures for all
 * objects in view.
 */
class AcquireObjectsServer
{
protected:

	/**
	 * A common nodehandle.
	 */
	ros::NodeHandle nh_;

	/**
	 * A nodehandle to use for the clusters message.
	 */
	ros::NodeHandle nh1_;

	/**
	 * A nodehandle to use for the recognized objects message.
	 */
	ros::NodeHandle nh2_;

	/**
	 * A callback queue for everything else.
	 */
	ros::CallbackQueue q_;

	/**
	 * A callback queue to associate to the clusters message.
	 */
	ros::CallbackQueue q1_;

	/**
	 * A callback queue to associate to the recognized objects message.
	 */
	ros::CallbackQueue q2_;

	/**
	 * The Server.
	 */
	ros::ServiceServer server_;

	/**
	 * A subscriber for the clusters.
	 */
	ros::Subscriber clusters_sub_;

	/**
	 * A Subscriber for the recognized objects.
	 */
	ros::Subscriber recognized_objects_sub_;

	/**
	 * A variable to hold the clusters.
	 */
	doro_msgs::ClusterArrayPtr clusters_ptr_;

	/**
	 * A variable to hold the recognized objects.
	 */
	doro_msgs::RecognizedObjectArrayPtr recognized_objects_ptr_;

	/**
	 * The server callback function.
	 */
	bool serverCB(acquire_objects::AcquireObjectsRequest& request,
			acquire_objects::AcquireObjectsResponse& response);

	/**
	 * The callback function for clusters.
	 */
	void clustersCB(const doro_msgs::ClusterArrayConstPtr& _clusters);

	/**
	 * The callback function for Sift detected object array.
	 */
	void recognizedObjectsCB(const doro_msgs::RecognizedObjectArrayConstPtr& _recognized_objects);

public:

	/**
	 * The spinning thread.
	 */
	//static void* spinThread(void* _this);

	/**
	 * A global spinner.
	 */
	void spin();

	/**
	 * Default constructor
	 */
	AcquireObjectsServer();

	/**
	 * Default destructor.
	 */
	virtual ~AcquireObjectsServer();
};

#endif /* ACQUIRE_OBJECTS_SERVER_H_ */
