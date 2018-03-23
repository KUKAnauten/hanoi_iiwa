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
  }
  
  void gripperInit() {
    gripper.setInitialization(INIT_ACTIVATION);
    gripper.setGraspingMode(GRASP_PINCH);
    gripper.setActionMode(ACTION_GO);
    gripper.setRawVelocity(255);
    gripper.setRawForce(50);
    gripper.setRawPosition(0);
    gripper.write();
    waitForGripper();
  }

  void waitForGripper() {
    ros::Duration(0.1).sleep();
    do {
      //std::cout << (!gripper.isInitialized() ? "Not initialized!" : "") << std::endl;
      //std::cout << (!gripper.isReady() ? "Not ready!" : "") << std::endl;
      //std::cout << (gripper.isMoving() ? "Still Moving!" : "") << std::endl;
      gripper.read();
      ros::Duration(0.1).sleep();
    } while (ros::ok() && (!gripper.isInitialized() || !gripper.isReady() || gripper.isMoving()));
    //std::cout << "Done Waiting!" << std::endl;
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

  void moveSlice(int from, int to) {
    assert(from >= 0 && from <= 2);
    assert(to >= 0 && to <= 2);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose_above_from = tower_poses_[from].pose;
    pose_above_from.position.z += 0.13;
    waypoints.push_back(pose_above_from);
    geometry_msgs::Pose pose_slice_from = tower_poses_[from].pose;
    pose_slice_from.position.z += slice_height_ * --tower_nSlices_[from];
    waypoints.push_back(pose_slice_from);
    moveAlongCartesianPathInWorldCoords(waypoints, 0.01, 0.3, true);

    gripperClose();

    waypoints.clear();
    waypoints.push_back(pose_above_from);
    geometry_msgs::Pose pose_to = tower_poses_[to].pose;
    pose_to.position.z += 0.13;
    waypoints.push_back(pose_to);
    pose_to.position.z -= 0.03;
    waypoints.push_back(pose_to);
    moveAlongCartesianPathInWorldCoords(waypoints, 0.01, 0.3, true);

    gripperOpen();

    pose_to.position.z += 0.03;
    planAndMove(pose_to, true);


    tower_nSlices_[to] += 1;
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
  std::vector<double> tow0_pose_jointSpace{-0.31814, 0.69113, -0.27301, -1.19496, 0.20649, 1.31196, -1.36038};
  std::vector<double> tow1_pose_jointSpace{-0.26360, 0.51776, 0.45096, -1.51083, -0.20304, 1.14264, -0.80443};
  std::vector<double> tow2_pose_jointSpace{0.37647, 0.61587, 0.12677, -1.30368, 0.05558, 1.18335, -0.50994};

  hanoi::HanoiRobot hanoi_robot(&node_handle, "manipulator", base_pose_jointSpace, 3, 0.01);
  hanoi_robot.setTowerPose(0, tow0_pose_jointSpace);
  hanoi_robot.setTowerPose(1, tow1_pose_jointSpace);
  hanoi_robot.setTowerPose(2, tow2_pose_jointSpace);
  hanoi_robot.planAndMoveToBasePose();
  hanoi_robot.gripperInit();

  hanoi_robot.moveTower(3, 0, 2, 1);

  /*
  geometry_msgs::Pose start_pose = hanoi_robot.getBasePose().pose;
  std::vector<geometry_msgs::Pose> waypoints;
  start_pose.position.z -= 0.12;
  waypoints.push_back(start_pose);
  start_pose.position.x += 0.12;
  waypoints.push_back(start_pose);
  start_pose.position.y -= 0.12;
  //start_pose.position.z -= 0.12;
  waypoints.push_back(start_pose);
  hanoi_robot.moveAlongCartesianPathInWorldCoords(waypoints, 0.01, 0.3, true);
  hanoi_robot.planAndMoveInBasePoseCoordSys(0.0, 0.0, 0.05);

  /*
  hanoi_robot.gripper.setInitialization(INIT_ACTIVATION);
  hanoi_robot.gripper.setActionMode(ACTION_GO);
  // Setting "raw" values [0, 255] instead of mm/s for velocity, N for force, etc.
  hanoi_robot.gripper.setRawVelocity(255);
  hanoi_robot.gripper.setRawForce(50);

  ROS_INFO("Closing gripper!");
  hanoi_robot.gripper.setRawPosition(255);
  hanoi_robot.gripper.write();
  ros::Duration(2.5).sleep();

  ROS_INFO("Moving to base pose!");
  hanoi_robot.planAndMoveToBasePose();
  
  ROS_INFO("Opening gripper!");
  hanoi_robot.gripper.setRawPosition(0);
  hanoi_robot.gripper.write();
  ros::Duration(2.5).sleep();

  ROS_INFO("Moving forward!");
  hanoi_robot.planAndMoveRelativeToBasePose(0.1, 0.0, 0.0, false);
  
  ROS_INFO("Closing gripper!");
  hanoi_robot.gripper.setRawPosition(255);
  hanoi_robot.gripper.write();
  ros::Duration(2.5).sleep();

  ROS_INFO("Moving to different position!");
  // hanoi_robot.planAndMoveRelativeToBasePose(0.1, 0.0, 0.2, false);
  // hanoi_robot.planAndMoveRelativeToBasePose(0.1, 0.2, 0.2, false);
  // hanoi_robot.planAndMoveRelativeToBasePose(0.1, 0.2, 0.0, false);
  hanoi_robot.planAndMoveRelativeToBasePose(0.1, 0.0, 0.2, 0,0,0, false);
  hanoi_robot.planAndMoveRelativeToBasePose(0.1, 0.2, 0.2, 0,-1.0,0,false);
  hanoi_robot.planAndMoveRelativeToBasePose(0.1, 0.2, 0.0, 0,0,1,false);

  ROS_INFO("Opening gripper!");
  hanoi_robot.gripper.setRawPosition(0);
  hanoi_robot.gripper.write();
  ros::Duration(2.5).sleep();

  ROS_INFO("Moving backwards!");
  hanoi_robot.planAndMoveRelativeToBasePose(0.0, 0.2, 0.0, false);
  */

  // Moving along cube corners
  // hanoi_robot.planAndMoveToBasePose();
  // double sdl = 0.15;
  // hanoi_robot.planAndMoveRelativeToBasePose(0.0, sdl, 0.0, true);
  // hanoi_robot.planAndMoveRelativeToBasePose(sdl, sdl, 0.0, false);
  // hanoi_robot.planAndMoveRelativeToBasePose(sdl, 0.0, 0.0, false);
  // hanoi_robot.planAndMoveRelativeToBasePose(sdl, 0.0, sdl, false);
  // hanoi_robot.planAndMoveRelativeToBasePose(0.0, 0.0, sdl, false);
  // hanoi_robot.planAndMoveRelativeToBasePose(0.0, sdl, sdl, false);
  // hanoi_robot.planAndMoveRelativeToBasePose(sdl, sdl, sdl, false);
  // hanoi_robot.planAndMoveRelativeToBasePose(0.0, 0.0, 0.0, false);
  
  ros::shutdown();
  return 0;
}
