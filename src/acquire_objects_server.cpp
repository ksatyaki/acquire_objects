/*
 * acquire_objects_server.cpp
 *
 *  Created on: Aug 20, 2014
 *      Author: ace
 */

#include "acquire_objects/acquire_objects_server.h"

AcquireObjectsServer::AcquireObjectsServer()
{
	clusters_sub_ = nh_.subscribe("/clusters", 1, &AcquireObjectsServer::clustersCB, this);
	recognized_objects_sub_ = nh_.subscribe("/recognized_objects", 1, &AcquireObjectsServer::recognizedObjectsCB, this);

	//nh_.setCallbackQueue(&q_);
	//nh1_.setCallbackQueue(&q1_);
	//nh2_.setCallbackQueue(&q2_);

	server_ = nh_.advertiseService("acquire_objects", &AcquireObjectsServer::serverCB, this);

	ROS_INFO("Server Started!");
}

bool AcquireObjectsServer::serverCB(acquire_objects::AcquireObjectsRequest& request, acquire_objects::AcquireObjectsResponse& response)
{
	ROS_INFO("Server Called!");
	bool return_value;
	std::string object_name = "unknown_object_";

	clusters_ptr_.reset();
	recognized_objects_ptr_.reset();

	ros::param::set("/cluster_extraction_enable", true);
	ros::param::set("/sift_extraction_enable", true);

	while( (!recognized_objects_ptr_ || !clusters_ptr_) && ros::ok())
	{
		if(!clusters_ptr_)
			ROS_INFO("Waiting for clusters.");
		else
			ROS_INFO("We have the clusters.");

		if(!recognized_objects_ptr_)
			ROS_INFO("Waiting for sift recognized objects.");
		else
			ROS_INFO("We have the sift recognized objects.");

		sleep(1);
	}

	ros::param::set("/cluster_extraction_enable", false);
	ros::param::set("/sift_extraction_enable", false);

	ROS_INFO("Processing...");
	int unknown_no = 0;
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
				__object.cluster_size = it_clust->cluster_size;

				associated = true;
				response.objects.table_objects.push_back(__object);

				// This cluster has been associated. Remove the corresponding object.
				recognized_objects_ptr_->recognized_objects.erase(it_obj);
				it_obj --;
				break;
			}
		}
		if(!associated)
		{
			doro_msgs::TableObject __object;
			char object_number[4];
			sprintf(object_number, "%d", unknown_no);

			__object.id = object_name + object_number;

			unknown_no++;
			__object.centroid = it_clust->centroid.point;
			__object.cluster_size = it_clust->cluster_size;

			response.objects.table_objects.push_back(__object);
		}
	}

	response.objects.number_of_objects = response.objects.table_objects.size();

	if(request.signature.id == "signature")
	{
		if(request.signature.cluster_size.size() != 0)
		{
			for(std::vector<doro_msgs::TableObject>::iterator iter_obj = response.objects.table_objects.begin();
					iter_obj != response.objects.table_objects.end();
					iter_obj++)
			{
				if(iter_obj->cluster_size[0] > (0.025 + request.signature.cluster_size[0]) ||
					iter_obj->cluster_size[0] < (request.signature.cluster_size[0] - 0.025) ||
					iter_obj->cluster_size[1] > (0.025 + request.signature.cluster_size[1]) ||
					iter_obj->cluster_size[1] < (request.signature.cluster_size[1] - 0.025) ||
					iter_obj->cluster_size[2] > (0.025 + request.signature.cluster_size[2]) ||
					iter_obj->cluster_size[2] < (request.signature.cluster_size[2] - 0.025) )
				{
					// If the size is out of bounds, remove it.
					response.objects.table_objects.erase(iter_obj);
					iter_obj--;
				}
			}
			ROS_INFO("We pruned for size. There are %d objects now.", response.objects.table_objects.size());
		}
		else
			ROS_INFO("Size = 0. Size Ignored.");


		if(request.signature.color.size() == 3)
		{
			//int color = (request.signature.color[0] << 16) | (request.signature.color[1] << 8) | (request.signature.color[2]);

			for(std::vector<doro_msgs::TableObject>::iterator iter_obj = response.objects.table_objects.begin();
								iter_obj != response.objects.table_objects.end();
								iter_obj++)
			{
				if( (iter_obj->color[0] > (50 + request.signature.color[0]) || iter_obj->color[0] < (request.signature.color[0] - 50) ) ||
						(iter_obj->color[1] > (50 + request.signature.color[1]) || iter_obj->color[1] < (request.signature.color[1] - 50) ) ||
								(iter_obj->color[2] > (50 + request.signature.color[2]) || iter_obj->color[2] < (request.signature.color[2] - 50) ) )
				{
					response.objects.table_objects.erase(iter_obj);
					iter_obj--;
				}
			}

			ROS_INFO("We pruned for color. There are %d objects now.", response.objects.table_objects.size());

		}
		else
			ROS_INFO("color_size = 0. Color Ignored.");

		if(request.signature.centroid.x != 0.0 && request.signature.centroid.y != 0.0 && request.signature.centroid.z != 0.0)
		{
			for(std::vector<doro_msgs::TableObject>::iterator iter_obj = response.objects.table_objects.begin();
											iter_obj != response.objects.table_objects.end();
											iter_obj++)
			{
				if( ( (iter_obj->centroid.x > (request.signature.centroid.x + request.signature.centroid_tolerance.x) ) ||
						(iter_obj->centroid.x < (request.signature.centroid.x - request.signature.centroid_tolerance.x) ) ) ||

					( (iter_obj->centroid.y > (request.signature.centroid.y + request.signature.centroid_tolerance.y) ) ||
						(iter_obj->centroid.y < (request.signature.centroid.y - request.signature.centroid_tolerance.y) ) ) ||

					( (iter_obj->centroid.z > (request.signature.centroid.z + request.signature.centroid_tolerance.z) ) ||
						(iter_obj->centroid.z < (request.signature.centroid.z - request.signature.centroid_tolerance.z) ) ) )
				{
					ROS_INFO("IS HERE");
					ROS_INFO("We pruned for location. There are %d objects now.", response.objects.table_objects.size());
					response.objects.table_objects.erase(iter_obj);
					iter_obj--;
				}
			}
			ROS_INFO("We pruned for location. There are %d objects now.", response.objects.table_objects.size());
		}
		else
			ROS_INFO("centroid = 0,0,0. Location Ignored.");
	}

	else if(request.signature.id.empty())
		response.objects.table_objects.clear();

	else if(request.signature.id != "all")
	{
		// In this case return only the element requested.
		// That is, we return only that element which has the same id as in the signature.
		for(std::vector<doro_msgs::TableObject>::iterator iter_obj = response.objects.table_objects.begin();
				iter_obj != response.objects.table_objects.end();
				iter_obj++)
		{
			if(iter_obj->id.empty() || iter_obj->id.find("unknown") != std::string::npos)
			{
				response.objects.table_objects.erase(iter_obj);
				iter_obj--;
				continue;
			}

			else if( (request.signature.id.compare("known") != 0) && (iter_obj->id.compare(request.signature.id) != 0))
			{
				ROS_INFO("%s != %s", iter_obj->id.c_str(), request.signature.id.c_str());
				response.objects.table_objects.erase(iter_obj);
				iter_obj--;
				continue;
			}
		}

		ROS_INFO("We pruned for id. There are %d objects now.", response.objects.table_objects.size());
	}

	else
		ROS_INFO("Returning all objects.");

	return_value = true;

	ROS_INFO("Response sent.");

	clusters_ptr_.reset();
	recognized_objects_ptr_.reset();
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
	ros::MultiThreadedSpinner m_t_spinner(4);
	//pthread_t id;
	//pthread_create(&id, NULL, &AcquireObjectsServer::spinThread, (void *) &A_O_S);

	//ros::param::set("/cluster_extraction_enable", true);
	//ros::param::set("/sift_extraction_enable", true);

	m_t_spinner.spin();
}


