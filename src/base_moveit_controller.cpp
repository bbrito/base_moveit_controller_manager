/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2016, Robert Haschke
 *   Copyright (c) 2017, Bruno Brito
 *  All rights reserved.
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
 *   * Neither the names of the authors nor the names of its
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
 *********************************************************************/

/* Author: Ioan Sucan, Robert Haschke, Bruno Brito */

#include <base_moveit_controller_manager/base_moveit_controller.h>
#include <ros/param.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <limits>

namespace base_moveit_controller_manager
{
BaseControllerHandle::BaseControllerHandle(const std::string& name, const std::vector<std::string>& joints,
                                       const ros::Publisher& pub)
  : moveit_controller_manager::MoveItControllerHandle(name), joints_(joints), pub_(pub)
{
  std::stringstream ss;
  ss << name << "' with joints [ ";
  std::copy(joints.begin(), joints.end(), std::ostream_iterator<std::string>(ss, " "));
  ss << "]";
  ROS_INFO_STREAM(ss.str());
  static const std::string MOVEIT_ACTION_NAME = "fake_base_controller";
  moveit_action_client_.reset(new actionlib::SimpleActionClient<predictive_control::trajAction>("/fake_base_controller", this));
	ROS_INFO("Waiting for MPC Controller");
	moveit_action_client_->waitForServer();
	ROS_INFO("MPC Controller Found!");
}

void BaseControllerHandle::getJoints(std::vector<std::string>& joints) const
{
  joints = joints_;
}

moveit_controller_manager::ExecutionStatus BaseControllerHandle::getLastExecutionStatus()
{
  return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
}

ThreadedController::ThreadedController(const std::string& name, const std::vector<std::string>& joints,
                                       const ros::Publisher& pub)
  : BaseControllerHandle(name, joints, pub)
{
}

ThreadedController::~ThreadedController()
{
  ThreadedController::cancelTrajectory();
}

void ThreadedController::cancelTrajectory()
{
  cancel_ = true;
  thread_.join();
}

bool ThreadedController::sendTrajectory(const moveit_msgs::RobotTrajectory& t)
{
  cancelTrajectory();  // cancel any previous fake motion
  cancel_ = false;
  status_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
  thread_ = boost::thread(boost::bind(&ThreadedController::execTrajectory, this, t));
  return true;
}

bool ThreadedController::cancelExecution()
{
  cancelTrajectory();
  ROS_INFO("Fake trajectory execution cancelled");
  status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
  return true;
}

bool ThreadedController::waitForExecution(const ros::Duration&)
{
  thread_.join();
  status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  return true;
}

moveit_controller_manager::ExecutionStatus ThreadedController::getLastExecutionStatus()
{
  return status_;
}

BaseBodyController::BaseBodyController(const std::string& name, const std::vector<std::string>& joints, const ros::Publisher& base_pub)
  : ThreadedController(name, joints, base_pub), rate_(10)
{
  double r;
  if (ros::param::get("~fake_interpolating_controller_rate", r))
    rate_ = ros::WallRate(r);
}

namespace
{
void interpolate(sensor_msgs::JointState& js, const trajectory_msgs::JointTrajectoryPoint& prev,
                 const trajectory_msgs::JointTrajectoryPoint& next, const ros::Duration& elapsed)
{
  double duration = (next.time_from_start - prev.time_from_start).toSec();
  double alpha = 1.0;
  if (duration > std::numeric_limits<double>::epsilon())
    alpha = (elapsed - prev.time_from_start).toSec() / duration;

  js.position.resize(prev.positions.size());
  for (std::size_t i = 0, end = prev.positions.size(); i < end; ++i)
  {
    js.position[i] = prev.positions[i] + alpha * (next.positions[i] - prev.positions[i]);
  }

}
}

void BaseBodyController::execTrajectory(const moveit_msgs::RobotTrajectory& t)
{
	ROS_WARN("WHOLE BODY CONTROLLER execution of trajectory");
	predictive_control::trajGoal goal;
	goal.trajectory.multi_dof_joint_trajectory = t.multi_dof_joint_trajectory;

  double ysqr, t3, t4;

  
	ROS_WARN_STREAM("t: " << t);
 
  // Convert quaternions to euler angle and store in rotation.z
  for ( int i = 0; i< ((int)goal.trajectory.multi_dof_joint_trajectory.points.size()) ; i++ ){
   
    ROS_INFO_STREAM("Quaternion angle" << goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z);
   
    ysqr = goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.y * goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.y;
    t3 = +2.0 * (goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.w * goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z
                  + goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.x * goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.y);
    t4 = +1.0 - 2.0 * (ysqr + goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z * goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z);
  
    goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z = atan2(t3, t4);
  
    ROS_INFO_STREAM("Euler angle" << goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z);

  }

	ROS_INFO_STREAM("Converted trajectory: " << goal.trajectory.multi_dof_joint_trajectory);
	moveit_action_client_->sendGoal(goal);

}

}  // end namespace base_moveit_controller_manager
