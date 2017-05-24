/*********************************************************************
 *
 *  Copyright (c) 2015
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
 *   * Neither the name of the Author nor the names of its
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
#ifndef WAYPOINT_FOLLOWER_HPP
#define WAYPOINT_FOLLOWER_HPP


#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <asctec_hl_comm/WaypointAction.h>



	namespace mav {

		class WaypointFollower {
			private:

				float m_current_x;
				float m_current_y;
				float m_yaw;

				std_msgs::String m_current_ref_frame;
				std_msgs::String m_position_frame;


			public:
				WaypointFollower();
				~WaypointFollower();
				void WaypointFollower::followingCB(const geometry_msgs::TransformedStamped & ts);
				void WaypointFollower::feedbackCB(const asctec_hl_comm::WaypointFeedbackConstPtr & fb);
				void WaypointFollower::activeCb();
				void WaypointFollower::doneCb(const actionlib::SimpleClientGoalState& state, const asctec_hl_comm::WaypointResultConstPtr & result);

				float WaypointFollower::get_cur_x();
				float WaypointFollower::get_cur_y();
				float WaypointFollower::get_cur_yaw();
				std_msgs::String WaypointFollower::get_cur_ref_frame();

				int same_frame();

		};
	}


#endif //WAYPOINT_FOLLOWER_HPP
