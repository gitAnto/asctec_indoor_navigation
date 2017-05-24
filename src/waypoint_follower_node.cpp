/*********************************************************************
 *
 *  Copyright (c) 2015.
 *  All rights reserved.
 *
 *  Software License Agreement (BSD License)
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Authors nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Antonio Petitti on Oct, 2015
 *********************************************************************/


#include <asctec_hl_comm/WaypointAction.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include "waypoint_follower.hpp"
#include <dynamic_reconfigure/server.h>
#include <asctec_mav/asctec_mavConfig.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <math.h>



//structure
struct wp
{
  float x;
  float y;
  float yaw;
};

//Global Variables
double maxSpeed;
double accuracy;
double height;
float dist_from_wp;
float UAV_current_x = 100.0, UAV_current_y = 100.0, UAV_current_z = 100.0;

void callback(asctec_mav::asctec_mavConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f ", 
            config.maxSpeed, 
            config.accuracy,
	    config.height);

  maxSpeed = config.maxSpeed;
  accuracy = config.accuracy;
  height   = config.height;
}

/*void wpFeedbackCB(const asctec_hl_comm::WaypointFeedbackConstPtr& msg)
{
  //get status
  std::string status = msg->status;
  //find length of distance string
  std::size_t pos = status.find(" dist_yaw");
  //get the distance string
  std::string dist_str = status.substr(6,pos);
  //trick for converting string in double
  std::stringstream   data(dist_str);
  data >> dist_from_wp;

  ROS_INFO("distance of UAV from the current wp: %f",dist_from_wp);
} */

void feedbackCB(const asctec_hl_comm::WaypointFeedbackConstPtr & msg)
{
//const geometry_msgs::Point32 & wp = fb->current_pos;
//ROS_INFO("got feedback: %fm %fm %fm %f° ", wp.x, wp.y, wp.z, fb->current_yaw*180/M_PI);
//m_position_frame.data            = fb->header.frame_id;
  //get status
  std::string status = msg->status;
  //find length of distance string
  std::size_t pos = status.find(" dist_yaw");
  //get the distance string
  std::string dist_str = status.substr(6,pos);
  //trick for converting string in double
  std::stringstream   data(dist_str);
  data >> dist_from_wp;

  //ROS_INFO("distance of UAV from the current wp: %f",dist_from_wp);
}

void UAVpositionCB(const geometry_msgs::TransformStamped &ts)
{
	UAV_current_x       = ts.transform.translation.x;
	UAV_current_y 	    = ts.transform.translation.y;
	UAV_current_z 	    = ts.transform.translation.z;
}

void activeCb()
{
//ROS_INFO("Goal just went active");
}

void doneCb(const actionlib::SimpleClientGoalState& state, const asctec_hl_comm::WaypointResultConstPtr & result)
{
//if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
//ROS_INFO("Finished in state [%s]", state.toString().c_str());
//else
//ROS_WARN("Finished in state [%s]", state.toString().c_str());

//const geometry_msgs::Point32 & wp = result->result_pos;
//ROS_INFO("Reached waypoint: %fm %fm %fm %f°",wp.x, wp.y, wp.z, result->result_yaw*180/M_PI);
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "waypoint_client");
  ros::NodeHandle nh("~");

  std::string frame_id;
  std::string filename;
  int rate;
  double height_search, tollerance, height_decr, tracking_time;
  bool searching_flag;

  nh.param<int>("rate", rate, 1);
  nh.param<std::string>("frame_id", frame_id, "/odom");
  nh.param<double>("height_search", height_search, 1.0);
  nh.param<double>("tollerance", tollerance, 0.3);
  nh.param<double>("height_decr", height_decr, 0.1);
  nh.param<double>("tracking_time", tracking_time, 15.0);
  nh.param<bool>("searching_flag", searching_flag, false);
  std::string default_path = ros::package::getPath("asctec_mav") + "/cfg/waypoints.csv";
  nh.param<std::string>("wp_file_input", filename, default_path);

  ROS_INFO("Node started with following parameters");
  ROS_INFO("rate: %d", rate);
  ROS_INFO("frame_id; %s", frame_id.c_str());
  ROS_INFO("height_search: %f", height_search);
  ROS_INFO("tollerance: %f", tollerance);
  ROS_INFO("height_decr: %f", height_decr);
  ROS_INFO("searching_flag: %s", searching_flag ? "true" : "false");
  ROS_INFO("wp_file_input: %s", filename.c_str());

  dynamic_reconfigure::Server<asctec_mav::asctec_mavConfig> server;
  dynamic_reconfigure::Server<asctec_mav::asctec_mavConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  /*double maxSpeed;
  double accuracy;
  double height ;

  //nh.getParam("rate", rate);
  nh.getParam("maxSpeed", maxSpeed);
  nh.getParam("accuracy", accuracy);
  nh.getParam("height", height);*/

  mav::WaypointFollower waypoint_follower;

  ros::Rate loop_rate(rate);
  ROS_INFO("Set rate: %d Hz",rate);

  ros::Duration timeout(15);

  /*if (argc == 5)
  {
    maxSpeed = 0.5;
    accuracy = 0.3;
  }
  else if (argc == 6)
  {
    maxSpeed = atof(argv[5]);
    accuracy = 0.3;
  }
  else if (argc == 7)
  {
    maxSpeed = atof(argv[5]);
    accuracy = atof(argv[6]);
  }
  else if (argc == 8)
  {
    maxSpeed = atof(argv[5]);
    accuracy = atof(argv[6]);
    timeout = ros::Duration(atof(argv[7]));
  }
  else
  {
    std::cout << "Wrong number of arguments! \nusage:" << "\twpclient x y z yaw \n"
        << "\twpclient x y z yaw max_speed\n" << "\twpclient x y z yaw max_speed accuracy\n"
        << "\twpclient x y z yaw max_speed accuracy timeout\n" << std::endl;
    return -1;
  }*/

  ros::NodeHandle nh_;

  ros::Subscriber wpRefsub = nh_.subscribe("waypoint_ref", 10, &mav::WaypointFollower::followingCB, &waypoint_follower);
  ros::Subscriber wpFeedbacksub = nh_.subscribe("UAV_position", 10, UAVpositionCB); //subscriber to UAV position topic
  //ros::Subscriber wpFeedbacksub = nh_.subscribe("fcu/waypoint/feedback", 10, feedbackCB);
  actionlib::SimpleActionClient<asctec_hl_comm::WaypointAction> ac(nh_, "fcu/waypoint", true);



  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goals.");

  asctec_hl_comm::WaypointGoal goal;
  ros::Publisher vis_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  visualization_msgs::Marker marker;

  //create a vector of waypoints for searching
  //read goals from file
  std::ifstream wp_file(filename.c_str());
  std::vector<wp> waypoints;
  std::string line, x, y, yaw;

  ROS_INFO("reading waypoints from %s",filename.c_str());

  if (wp_file.is_open())
  {
    while ( getline(wp_file,line) )
    {
	std::istringstream myline(line);
        getline(myline, x,   ',');
        getline(myline, y,   ',');
        getline(myline, yaw, ','); 
	//DEBUG
	//std::cout << x << " " << y << " " << yaw << std::endl;
	//create new wp
	wp tmp_wp;
	//fill new wp
        std::stringstream   data(x+y+yaw);
        data >> tmp_wp.x >> tmp_wp.y >> tmp_wp.yaw;

/*	tmp_wp.x   = std::stof(x);
	tmp_wp.y   = std::stof(y);
	tmp_wp.yaw = std::stof(yaw);*/
	//push new wp in the vector
	waypoints.push_back(tmp_wp);

    }
    wp_file.close();

    ROS_INFO("waypoints loaded");
  }

  else ROS_ERROR("Unable to open file %s",filename.c_str());

  bool search   = true;
  bool tracking = false;
  bool landing  = false;

  if(!searching_flag){
	search   = false;
  	tracking = true;
  	landing  = false;
	ROS_INFO("tracking mode");
  }
  else
	  ROS_INFO("search mode");




  int index_wp = 0;
  ros::Duration trackingDuration(tracking_time);//seconds
  ros::Time begin = ros::Time::now();
  ros::Duration timeUntilNewWP(1.2);//seconds

  ros::ServiceClient motorsClient = nh_.serviceClient<asctec_hl_comm::mav_ctrl_motors>("fcu/motor_control"); //client service for shut down motors
  asctec_hl_comm::mav_ctrl_motors srv_motors;

  while(ros::ok()){ 

	if(search){


		

		if (index_wp>=waypoints.size())
			index_wp = 0;
		
		goal.goal_pos.x = waypoints[index_wp].x;
		goal.goal_pos.y = waypoints[index_wp].y;
		goal.goal_pos.z = height_search;

		goal.max_speed.x = maxSpeed;
		goal.max_speed.y = maxSpeed;
		goal.max_speed.z = maxSpeed;

		goal.goal_yaw = waypoints[index_wp].yaw;

	  	goal.accuracy_position = accuracy;
		goal.accuracy_orientation = 0;
		
		goal.header.frame_id = waypoint_follower.get_frame_id().data;

		goal.timeout = timeout.toSec();	

		if(waypoint_follower.same_frame()) {

			ROS_INFO("sending goal to: %f %f", waypoints[index_wp].x, waypoints[index_wp].y);
			ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCB);

			//ac.waitForResult(ros::Duration(3.0));

			float x_diff = waypoint_follower.get_cur_x() - UAV_current_x; //waypoints[index_wp].x;
			float y_diff = waypoint_follower.get_cur_y() - UAV_current_y; //waypoints[index_wp].y;

			ROS_INFO("UAV    coords --> x: %f y: %f",UAV_current_x,UAV_current_y);
			ROS_INFO("target coords --> x: %f y: %f",waypoint_follower.get_cur_x(),waypoint_follower.get_cur_y());

			float dist_from_target = sqrt(x_diff*x_diff + y_diff*y_diff);

			ROS_INFO("distance from target: %f",dist_from_target);

			if( sqrt(x_diff*x_diff + y_diff*y_diff) < tollerance){
				search   = false;
				tracking = true;
				begin = ros::Time::now();
	
				ROS_INFO("tracking mode");
			}				
			else
				if(dist_from_wp<accuracy+0.1){
					index_wp++;
					timeUntilNewWP.sleep();
				}

		}
                else {
			ROS_ERROR("Waypoint reference in a different frame!!");//,waypoint_follower.get_cur_ref_frame().data);
			return 0;
		}



	}			


	if(tracking){

		  goal.goal_pos.x = waypoint_follower.get_cur_x();
		  goal.goal_pos.y = waypoint_follower.get_cur_y();
		  goal.goal_pos.z = 1.0;//height;

		  goal.max_speed.x = maxSpeed;
		  goal.max_speed.y = maxSpeed;
		  goal.max_speed.z = maxSpeed;

		  goal.goal_yaw = waypoint_follower.get_cur_yaw() + 3.141516;

		  goal.accuracy_position = accuracy;
		  goal.accuracy_orientation = 0;
		
		  goal.header.frame_id = waypoint_follower.get_frame_id().data;

		  goal.timeout = timeout.toSec();



		  if(waypoint_follower.same_frame()) {
			ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCB);
	//		ac.sendGoal(goal, &mav::WaypointFollower::doneCb, &mav::WaypointFollower::activeCb, &mav::WaypointFollower::feedbackCB);

			marker.header.frame_id = waypoint_follower.get_frame_id().data;
			marker.header.stamp = ros::Time();
			marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = goal.goal_pos.x;
			marker.pose.position.y = goal.goal_pos.y;
			marker.pose.position.z = goal.goal_pos.z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;

			vis_pub.publish( marker );
		  }	
		  else {
			ROS_WARN("Waypoint reference in a different frame!!");//,waypoint_follower.get_cur_ref_frame().data);
		  }

		if(ros::Time::now()-begin > trackingDuration){
			tracking   = false;
			landing    = true;
			//begin      = ros::Time::now();
			ROS_INFO("landing mode");
		}

	}
 	  

	if(landing){

		float x_diff = waypoint_follower.get_cur_x() - UAV_current_x; //meters
		float y_diff = waypoint_follower.get_cur_y() - UAV_current_y; //meters
		float z_diff = waypoint_follower.get_cur_z() - UAV_current_z; //meters

		float dist_from_wp = sqrt(x_diff*x_diff + y_diff*y_diff);// + z_diff*z_diff);

		ROS_INFO("distance of UAV from the current wp: %f",dist_from_wp);

		if( dist_from_wp<0.05 ){//to-do: insert parameter

			if( abs(z_diff)<0.18 ){
				 srv_motors.request.startMotors = false;
				 motorsClient.call(srv_motors);
				 ROS_INFO("motors switched off");
				 return 0;
			}


		}



                 

		  goal.goal_pos.x = waypoint_follower.get_cur_x();
		  goal.goal_pos.y = waypoint_follower.get_cur_y();
		  goal.goal_pos.z = waypoint_follower.get_cur_z() + 0.15;

		  goal.max_speed.x = maxSpeed;
		  goal.max_speed.y = maxSpeed;
		  goal.max_speed.z = maxSpeed;

		  goal.goal_yaw = waypoint_follower.get_cur_yaw();

		  goal.accuracy_position = accuracy;
		  goal.accuracy_orientation = 0;
		
		  goal.header.frame_id = waypoint_follower.get_frame_id().data;

		  goal.timeout = timeout.toSec();



		  if(waypoint_follower.same_frame()) {
			ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCB);
	//		ac.sendGoal(goal, &mav::WaypointFollower::doneCb, &mav::WaypointFollower::activeCb, &mav::WaypointFollower::feedbackCB);

			marker.header.frame_id = waypoint_follower.get_frame_id().data;
			marker.header.stamp = ros::Time();
			marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = goal.goal_pos.x;
			marker.pose.position.y = goal.goal_pos.y;
			marker.pose.position.z = goal.goal_pos.z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;

			vis_pub.publish( marker );
		  }	
		  else {
			ROS_WARN("Waypoint reference in a different frame!!");//,waypoint_follower.get_cur_ref_frame().data);
		  }


	}

	  loop_rate.sleep();  
	  ros::spinOnce();


/*
	  //wait for the action to return
	  bool finished_before_timeout = ac.waitForResult(timeout);

	  if (!finished_before_timeout)
	  {
	    ROS_WARN("Action did not finish before the time out.");
	  }
*/


  }

  return 0;
}
