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

void BaseBodyController::constructClothoid(const moveit_msgs::RobotTrajectory& trajectory)
{

    ROS_INFO_STREAM("Constructing clothoid from planned trajectory");

    double clothoidLength = 0;
    double x0, y0, theta0, x1, y1, theta1;
    int res, traj_length;
    uint n_pts = 10;

    traj_length = trajectory.multi_dof_joint_trajectory.points.size();

    std::vector<double> k(traj_length), dk(traj_length), L(traj_length), L_cumul(traj_length);

    // Construct piecewise clothoid from trajectory points and store clothoid parameters
    for (int traj_it = 0; traj_it < traj_length - 1; traj_it++)
    {
        x0 = trajectory.multi_dof_joint_trajectory.points[traj_it].transforms[0].translation.x;
        y0 = trajectory.multi_dof_joint_trajectory.points[traj_it].transforms[0].translation.y;
        theta0 = trajectory.multi_dof_joint_trajectory.points[traj_it].transforms[0].rotation.z;
        
        x1 = trajectory.multi_dof_joint_trajectory.points[traj_it + 1].transforms[0].translation.x;
        y1 = trajectory.multi_dof_joint_trajectory.points[traj_it + 1].transforms[0].translation.y;
        theta1 = trajectory.multi_dof_joint_trajectory.points[traj_it + 1].transforms[0].rotation.z;

        res = Clothoid::buildClothoid(x0, y0, theta0, x1, y1, theta1, k[traj_it], dk[traj_it], L[traj_it]);

        clothoidLength += L[traj_it];
        L_cumul[traj_it] = clothoidLength;

    }

    // // Sample Clothoid for extra points on piecewise clothoid
    std::vector<double> X_sample(n_pts), Y_sample(n_pts), X_clothoid, Y_clothoid;

    for (int clothoid_it = 0; clothoid_it < traj_length - 1; clothoid_it++)
    {

        x0 = trajectory.multi_dof_joint_trajectory.points[clothoid_it].transforms[0].translation.x;
        y0 = trajectory.multi_dof_joint_trajectory.points[clothoid_it].transforms[0].translation.y;
        theta0 = trajectory.multi_dof_joint_trajectory.points[clothoid_it].transforms[0].rotation.z;
        
        res = Clothoid::pointsOnClothoid(x0, y0, theta0, k[clothoid_it], dk[clothoid_it], L[clothoid_it], n_pts, X_sample, Y_sample);

        if (clothoid_it == 0)
        {
            X_clothoid = X_sample;
            Y_clothoid = Y_sample;
        }
        else
        {
            X_clothoid.insert(X_clothoid.end(), X_sample.begin(), X_sample.end());
            Y_clothoid.insert(Y_clothoid.end(), Y_sample.begin(), Y_sample.end());
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
