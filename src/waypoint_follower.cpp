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

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/tf.h>

#include <asctec_hl_comm/WaypointAction.h>

#include "waypoint_follower.hpp"


	namespace mav {

			WaypointFollower::WaypointFollower(){

				m_current_ref_frame.data = "/odom";
				m_position_frame.data    = "/odom";
				m_current_x              = 0.0; // m
				m_current_y              = 0.0; // m
				m_current_z              = 0.0; // m
				m_current_yaw            = 0.0; // rad

			}

			WaypointFollower::~WaypointFollower(){

			}

			void WaypointFollower::followingCB(const geometry_msgs::TransformStamped &ts)
			{

				m_current_x         = ts.transform.translation.x;
				m_current_y 	    = ts.transform.translation.y;
				m_current_z 	    = ts.transform.translation.z;

				m_current_yaw		    = tf::getYaw(ts.transform.rotation);

				m_position_frame.data = ts.header.frame_id;

			}



			float WaypointFollower::get_cur_x()
			{
			  	return m_current_x;
			}

			float WaypointFollower::get_cur_y()
			{
			  	return m_current_y;
			}

			float WaypointFollower::get_cur_z()
			{
			  	return m_current_z;
			}

			float WaypointFollower::get_cur_yaw()
			{
			  	return m_current_yaw;
			}

			std_msgs::String WaypointFollower::get_frame_id()
			{
			  	return m_current_ref_frame;
			}

			int WaypointFollower::same_frame()
			{
			  	return m_current_ref_frame.data == m_position_frame.data;
			}



