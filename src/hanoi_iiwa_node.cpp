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

#include <cassert>
#include <iimoveit/robot_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <robotiq_s_model_control/s_model_msg_client.h>
#include <robotiq_s_model_control/s_model_api.h>

using namespace robotiq;

namespace hanoi {

class HanoiRobot : public iimoveit::RobotInterface {
private:
  boost::shared_ptr<robotiq_s_model_control::SModelMsgClient> sModelMsgClient_;
  std::vector<double> tower_poses_jointSpace_[3];
  geometry_msgs::PoseStamped tower_poses_[3];
  int tower_nSlices_[3];
  double slice_height_;
  bool grapping_;


public:
  robotiq_s_model_control::SModelAPI gripper;


  HanoiRobot(ros::NodeHandle* node_handle, const std::string& planning_group, const std::vector<double> base_pose, int tow1_nSlices, double slice_height)
      :  RobotInterface(node_handle, planning_group, base_pose),
         tower_poses_jointSpace_{base_pose, base_pose, base_pose},
         tower_nSlices_{tow1_nSlices, 0, 0},
         slice_height_(slice_height),
         sModelMsgClient_(new robotiq_s_model_control::SModelMsgClient(*node_handle)),
         gripper(sModelMsgClient_) {
    tower_poses_[0] = poseFromJointAngles(base_pose);
    tower_poses_[1] = poseFromJointAngles(base_pose);
    tower_poses_[2] = poseFromJointAngles(base_pose);
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

  void setTowerPose(int index, const std::vector<double>& pose) {
    assert(index >= 0 && index <= 2);
    tower_poses_jointSpace_[index] = pose;
    tower_poses_[index] = poseFromJointAngles(pose);
    std::cout << tower_poses_[index] << std::endl;

    if (index == 1) {
      base_pose_ = tower_poses_[1];
      base_pose_.pose.position.z += 0.3;
    }
  }

  void setTowerPose(int index, const PoseStamped& pose) {
    assert(index >= 0 && index <= 2);
    tower_poses_[index] = (pose);

    if (index == 1) {
      base_pose_ = tower_poses_[1];
      base_pose_.pose.position.z += 0.3;
    }
  }

  geometry_msgs::PoseStamped getTowerPose(int index) {
    assert(index >= 0 && index <= 2);
    return tower_poses_[index];
  }
  
  void gripperInit() {
    gripper.setInitialization(INIT_ACTIVATION);
    gripper.setGraspingMode(GRASP_PINCH);
    gripper.setActionMode(ACTION_GO);
    gripper.setRawVelocity(255);
    gripper.setRawForce(1);
    gripper.setRawPosition(0);
    gripper.write();
    waitForGripper();
  }

  void waitForGripper() {
    ros::Duration(0.1).sleep();
    do {
      if (!gripper.isInitialized()) std::cout <<  "Not initialized!" << std::endl;
      if (!gripper.isReady()) std::cout << "Not ready!" << std::endl;
      if (gripper.isMoving()) std::cout << "Still Moving!" << std::endl;
      gripper.read();
      ros::Duration(0.1).sleep();
    } while (ros::ok() && (!gripper.isInitialized() || !gripper.isReady() || gripper.isMoving()));
    ros::Duration(0.1).sleep();
  }

  void gripperClose() {
    gripper.setRawPosition(90);
    gripper.write();
    waitForGripper();
  }

  void gripperOpen() {
    gripper.setRawPosition(45);
    gripper.write();
    waitForGripper();
  }

  void checkPoses() {
    for (int i = 0; i < 3; ++i) {
      geometry_msgs::PoseStamped pose = tower_poses_[i];
      pose.pose.position.z += 0.13;
      planAndMove(pose, true);

      pose.pose.position.z -= 0.13;
      planAndMove(pose, true);

      pose.pose.position.z += 0.13;
      planAndMove(pose, true);
    }
  }

  void planAndMoveAboveTower(bool approvalRequired = true) {
    geometry_msgs::PoseStamped current_pose = getPose();
    double difference = current_pose.pose.position.z - tower_poses_[0].pose.position.z - 0.133;
    if (difference < 0) {
      current_pose.pose.position.z -= difference;
      planAndMove(current_pose, approvalRequired);
    }
  }

  void moveSlice(int from, int to) {
    assert(from >= 0 && from <= 2);
    assert(to >= 0 && to <= 2);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose_above_from = tower_poses_[from].pose;
    pose_above_from.position.z += 0.133;
    for (int i = 0; i < (13 - tower_nSlices_[from] + 1); ++i) {
      pose_above_from.position.z -= 0.01;
      waypoints.push_back(pose_above_from);
    }
    moveAlongCartesianPathInWorldCoords(waypoints, 0.01, 0, true, false);

    gripperClose();
    tower_nSlices_[from]--;

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = move_group_.getEndEffectorLink();
    geometry_msgs::PoseStamped current_pose = getPose();
    ocm.header.frame_id = current_pose.header.frame_id;
    ocm.orientation.w = current_pose.pose.orientation.w;
    ocm.orientation.x = current_pose.pose.orientation.x;
    ocm.orientation.y = current_pose.pose.orientation.y;
    ocm.orientation.z = current_pose.pose.orientation.z;
    ocm.absolute_x_axis_tolerance = 0.01;
    ocm.absolute_y_axis_tolerance = 0.01;
    ocm.absolute_z_axis_tolerance = 0.01;
    ocm.weight = 1.0;
    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group_.setPathConstraints(constraints);

    std::reverse(waypoints.begin(), waypoints.end());

    geometry_msgs::Pose pose_to = tower_poses_[to].pose;
    pose_to.position.z += 0.133;
    waypoints.push_back(pose_to);
    for (int i = 0; i < 3; ++i) {
      pose_to.position.z -= 0.01;
      waypoints.push_back(pose_to);
    }
    moveAlongCartesianPathInWorldCoords(waypoints, 0.01, 0, true, false);
    move_group_.clearPathConstraints();

    gripperOpen();

    pose_to.position.z += 0.03;
    planAndMove(pose_to, false);

    tower_nSlices_[to]++;
  }

  void moveTower(int height, int from, int to, int with) {
    assert(from >= 0 && from <= 2);
    assert(to >= 0 && to <= 2);
    assert(with >= 0 && with <= 2);

    if (height >= 1) {
      moveTower(height-1, from, with, to);
      moveSlice(from, to);
      moveTower(height-1, with, to, from);
    }
    else if (height < 0) {
      for (int i = 0; i < 3; ++i) {
        geometry_msgs::PoseStamped pose_above_from = tower_poses_[i];
        pose_above_from.pose.position.z += 0.15;
        planAndMove(pose_above_from, true);
        planAndMove(tower_poses_[i], true);
        planAndMove(pose_above_from, true);
      }
    }
  }

  void planAndMoveToBasePose(bool approvalRequired = true) {
    planAndMoveAboveTower(approvalRequired);
    RobotInterface::planAndMoveToBasePose(approvalRequired);
  }
};
} // namespace hanoi


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_pose_follower");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<double> base_pose_jointSpace{-0.30564, 0.37769, 0.59875, -1.14323, -0.19265, 1.62531, -0.65100};
  std::vector<double> tow0_pose_jointSpace{-0.782207548618, 0.699450016022, 0.556021511555, -1.25907731056, -0.363562077284, 1.28127717972, -1.78963983059};
  std::vector<double> tow1_pose_jointSpace{-0.304185926914, 0.562530577183, 0.556022703648, -1.47326624393, -0.308449208736, 1.19399666786, -1.26301240921};
  std::vector<double> tow2_pose_jointSpace{0.133967101574, 0.697635769844, 0.555947721004, -1.26097989082, -0.362120121717, 1.28161561489, -0.873358488083};

  hanoi::HanoiRobot hanoi_robot(&node_handle, "manipulator", base_pose_jointSpace, 3, 0.01);
  hanoi_robot.setTowerPose(0, tow0_pose_jointSpace);
  hanoi_robot.setTowerPose(1, tow1_pose_jointSpace);
  hanoi_robot.setTowerPose(2, tow2_pose_jointSpace);
  //hanoi_robot.checkPoses();
  //std::cout << hanoi_robot.getPose().pose.position.z - hanoi_robot.getTowerPose(0).pose.position.z << std::endl;
  hanoi_robot.planAndMoveToBasePose();
  hanoi_robot.gripperInit();
  hanoi_robot.waitForApproval();
  hanoi_robot.moveTower(3, 0, 2, 1);
  hanoi_robot.planAndMoveToBasePose(false);
  
  ros::shutdown();
  return 0;
}
