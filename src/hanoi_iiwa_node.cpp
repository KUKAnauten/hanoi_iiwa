  /*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of the author nor the names of its
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

/* Author: Marcus Ebner */

/*
a1: -0.753815352917
a2: 1.1734058857
a3: 0.112757593393
a4: -1.68315970898
a5: -0.736448764801
a6: -1.34951746464
a7: 0.104109719396
*/

#include <iimoveit/robot_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <robotiq_s_model_control/s_model_msg_client.h>
#include <robotiq_s_model_control/s_model_api.h>

using namespace robotiq;

namespace hanoi {

class HanoiRobot : public iimoveit::RobotInterface {
private:
  boost::shared_ptr<robotiq_s_model_control::SModelMsgClient> sModelMsgClient_;
  std::vector<double> base_pose_jointspace_;
  geometry_msgs::PoseStamped base_pose_;
  geometry_msgs::PoseStamped left_tower_relPose_;
  geometry_msgs::PoseStamped right_tower_relPose_;
  geometry_msgs::PoseStamped center_tower_relPose_;
  double slice_height_;
  int nslices_left_;
  int nslices_center_;
  int nslices_right_;
  bool grapping_;


public:
  robotiq_s_model_control::SModelAPI gripper;


  HanoiRobot(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame)
      :  RobotInterface(node_handle, planning_group, base_frame),
         base_pose_jointspace_{-0.753815352917, 1.1734058857, 0.112757593393, -1.68315970898, -0.736448764801, -1.34951746464, 0.104109719396},
         sModelMsgClient_(new robotiq_s_model_control::SModelMsgClient(*node_handle)),
         gripper(sModelMsgClient_) {
    // Get endeffector pose from joint space goal
    robot_state_.setJointGroupPositions(joint_model_group_, base_pose_jointspace_);
    const Eigen::Affine3d& end_effector_state = robot_state_.getGlobalLinkTransform(move_group_.getEndEffectorLink());
    Eigen::Vector3d t(end_effector_state.translation());
    Eigen::Quaterniond q(end_effector_state.rotation());
    base_pose_.header.frame_id = "iiwa_link_0";
    base_pose_.pose.position.x = t[0];
    base_pose_.pose.position.y = t[1];
    base_pose_.pose.position.z = t[2];
    base_pose_.pose.orientation.x = q.x();
    base_pose_.pose.orientation.y = q.y();
    base_pose_.pose.orientation.z = q.z();
    base_pose_.pose.orientation.w = q.w();

    // Reset robot state to current state
    updateRobotState();
  }

  void moveToBasePose() {
    planAndMove(base_pose_jointspace_, std::string("base_pose_jointspace"), true);
  }

  void moveToBaseRelativePose(const geometry_msgs::Pose& relativePose, bool approvalRequired) {
    tf::Quaternion base_quaternion(base_pose_.pose.orientation.x, base_pose_.pose.orientation.y, base_pose_.pose.orientation.z, base_pose_.pose.orientation.w);
    tf::Quaternion next_quaternion(relativePose.orientation.x, relativePose.orientation.y, relativePose.orientation.z, relativePose.orientation.w);
    tf::Quaternion result_quaternion = next_quaternion * base_quaternion;
    result_quaternion.normalize();

    geometry_msgs::PoseStamped target_pose = base_pose_;
    target_pose.pose.position.x += relativePose.position.x;
    target_pose.pose.position.y += relativePose.position.y;
    target_pose.pose.position.z += relativePose.position.z;
    target_pose.pose.orientation.x = result_quaternion.getX();
    target_pose.pose.orientation.y = result_quaternion.getY();
    target_pose.pose.orientation.z = result_quaternion.getZ();
    target_pose.pose.orientation.w = result_quaternion.getW();

    planAndMove(target_pose, std::string("relative pose"), approvalRequired);
  }

  void moveToBaseRelativePosition(const geometry_msgs::Point relativePosition, bool approvalRequired) {
    geometry_msgs::PoseStamped target_pose = base_pose_;
    target_pose.pose.position.x += relativePosition.x;
    target_pose.pose.position.y += relativePosition.y;
    target_pose.pose.position.z += relativePosition.z;

    planAndMove(target_pose, std::string("relative pose"), approvalRequired);
  }

  void moveToBaseRelativePosition(double x, double y, double z, bool approvalRequired) {
    geometry_msgs::PoseStamped target_pose = base_pose_;
    target_pose.pose.position.x += x;
    target_pose.pose.position.y += y;
    target_pose.pose.position.z += z;

    planAndMove(target_pose, std::string("relative pose"), approvalRequired);
  }

  // For this button to work, the configuration has to be uploaded to the parameter server via a launch file like 'iiwa_stack/iiwa_ros/launch/toolbar_example.launch'.
  // Note: This has to be done BEFORE the Application is started via SmartPAD on the Sunrise Cabinet!
  void buttonEventCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Der Schmartie funktioniert: %s", msg->data.c_str());
    if(msg->data == "pose_get_pressed") {
      geometry_msgs::PoseStamped current_pose = getPose();
      ROS_INFO("Current Position = (%f, %f, %f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    }
  }

  void grabFromLeftTower() {}
  void putToLeftTower() {}
  void grabFromCenterTower() {}
  void putToCenterTower() {}
  void grabFromRightTower() {}
  void putToRightTower(){}
};
} // namespace hanoi

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_pose_follower");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  hanoi::HanoiRobot hanoi_robot(&node_handle, "manipulator", "world");

  hanoi_robot.gripper.setInitialization(INIT_ACTIVATION);
  hanoi_robot.gripper.setActionMode(ACTION_GO);
  // Setting "raw" values [0, 255] instead of mm/s for velocity, N for force, etc.
  hanoi_robot.gripper.setRawVelocity(255);
  hanoi_robot.gripper.setRawForce(50);

  hanoi_robot.moveToBasePose();
  
  hanoi_robot.gripper.setRawPosition(0);
  hanoi_robot.gripper.write();
  ros::Duration(2.5).sleep();

  hanoi_robot.moveToBaseRelativePosition(0.15, 0.0, 0.0, false);
  
  hanoi_robot.gripper.setRawPosition(255);
  hanoi_robot.gripper.write();
  ros::Duration(2.5).sleep();

  hanoi_robot.moveToBaseRelativePosition(0.15, 0.0, 0.4, false);
  hanoi_robot.moveToBaseRelativePosition(0.15, 0.4, 0.4, false);
  hanoi_robot.moveToBaseRelativePosition(0.15, 0.4, 0.0, false);

  hanoi_robot.gripper.setRawPosition(0);
  hanoi_robot.gripper.write();
  ros::Duration(2.5).sleep();

  hanoi_robot.moveToBaseRelativePosition(0.0, 0.4, 0.0, false);

  // Moving along cube corners
  // hanoi_robot.moveToBasePose();
  // double sdl = 0.15;
  // hanoi_robot.moveToBaseRelativePosition(0.0, sdl, 0.0, true);
  // hanoi_robot.moveToBaseRelativePosition(sdl, sdl, 0.0, false);
  // hanoi_robot.moveToBaseRelativePosition(sdl, 0.0, 0.0, false);
  // hanoi_robot.moveToBaseRelativePosition(sdl, 0.0, sdl, false);
  // hanoi_robot.moveToBaseRelativePosition(0.0, 0.0, sdl, false);
  // hanoi_robot.moveToBaseRelativePosition(0.0, sdl, sdl, false);
  // hanoi_robot.moveToBaseRelativePosition(sdl, sdl, sdl, false);
  // hanoi_robot.moveToBaseRelativePosition(0.0, 0.0, 0.0, false);
  
  ros::shutdown();
  return 0;
}
