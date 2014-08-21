/*
 * acquire_objects_server.cpp
 *
 *  Created on: Aug 20, 2014
 *      Author: ace
 */

#include "acquire_objects/acquire_objects_server.h"

AcquireObjectsServer::AcquireObjectsServer()
{
	clusters_sub_ = nh1_.subscribe("/clusters", 1, &AcquireObjectsServer::clustersCB, this);
	recognized_objects_sub_ = nh2_.subscribe("/recognized_objects", 1, &AcquireObjectsServer::recognizedObjectsCB, this);

	//nh_.setCallbackQueue(&q_);
	nh1_.setCallbackQueue(&q1_);
	nh2_.setCallbackQueue(&q2_);

	server_ = nh_.advertiseService("acquire_objects", &AcquireObjectsServer::serverCB, this);

	ROS_INFO("Server Started!");
}

bool AcquireObjectsServer::serverCB(acquire_objects::AcquireObjectsRequest& request, acquire_objects::AcquireObjectsResponse& response)
{
	ROS_INFO("Server Called!");
	bool return_value;
	if(request.name == "all" || request.name.empty())
	{
		ROS_INFO("Processing for name:= \"all\".");

		while( (!recognized_objects_ptr_ || !clusters_ptr_) && ros::ok())
		{
			if(!clusters_ptr_)
			{
				ROS_INFO("Waiting for clusters.");
				q1_.callOne(ros::WallDuration(1.0));
			}
			else
				ROS_INFO("We have the clusters.");

			sleep(1);

			if(!recognized_objects_ptr_)
			{
				ROS_INFO("Waiting for sift recognized objects.");
				q2_.callOne(ros::WallDuration(1.0));
			}
			else
				ROS_INFO("We have the sift recognized objects.");
		}

		ros::param::set("/cluster_extraction_enable", false);
		ros::param::set("/sift_extraction_enable", false);

		for(std::vector<doro_msgs::Cluster>::iterator it_clust = clusters_ptr_->clusters.begin();
				it_clust != clusters_ptr_->clusters.end();
				it_clust++)
		{
			bool associated = false;

			for(std::vector<doro_msgs::RecognizedObject>::iterator it_obj = recognized_objects_ptr_->recognized_objects.begin();
					it_obj != recognized_objects_ptr_->recognized_objects.end();
					it_obj++)
			{
				if( fabs(DIST2D(it_obj->x, it_obj->y, it_clust->x, it_clust->y) ) < 100)
				{
					doro_msgs::TableObject __object;
					__object.centroid = it_clust->centroid.point;
					__object.id = it_obj->id;
					__object.size = it_clust->size;

					associated = true;
					response.objects.table_objects.push_back(__object);
					// This cluster has been associated. Remove the corresponding object.
					recognized_objects_ptr_->recognized_objects.erase(it_obj);
					break;
				}
			}

			if(!associated)
			{
				doro_msgs::TableObject __object;
				__object.centroid = it_clust->centroid.point;
				__object.size = it_clust->size;

				response.objects.table_objects.push_back(__object);
			}
		}

		return_value = true;
	}
	else
		return_value = false;

	clusters_ptr_.reset();
	recognized_objects_ptr_.reset();

	ROS_INFO("Response sent.");
	ros::param::set("/cluster_extraction_enable", true);
	ros::param::set("/sift_extraction_enable", true);
	return return_value;
}

void AcquireObjectsServer::clustersCB(const doro_msgs::ClusterArrayConstPtr& _clusters)
{
	// Create space for new message
	clusters_ptr_ = doro_msgs::ClusterArrayPtr (new doro_msgs::ClusterArray);

	ROS_INFO("Fetching clusters.");
	// Copy
	*clusters_ptr_ = *_clusters;
}

void AcquireObjectsServer::recognizedObjectsCB(const doro_msgs::RecognizedObjectArrayConstPtr& _recognized_objects)
{
	// Create space for new message
	recognized_objects_ptr_ = doro_msgs::RecognizedObjectArrayPtr (new doro_msgs::RecognizedObjectArray);

	ROS_INFO("Fetching sift recognized objects.");
	// Copy
	*recognized_objects_ptr_ = *_recognized_objects;
}


AcquireObjectsServer::~AcquireObjectsServer()
{
	clusters_sub_.shutdown();
	recognized_objects_sub_.shutdown();
	server_.shutdown();
	ROS_INFO("Subscriptions cancelled.");
	ROS_INFO("Server shutdown.");
}

int main(int argn, char* args[])
{
	ros::init(argn, args, "acquire_objects_server");

	AcquireObjectsServer A_O_S;
	//pthread_t id;
	//pthread_create(&id, NULL, &AcquireObjectsServer::spinThread, (void *) &A_O_S);

	ros::param::set("/cluster_extraction_enable", true);
	ros::param::set("/sift_extraction_enable", true);

	ros::spin();
}


