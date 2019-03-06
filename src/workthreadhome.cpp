#include "workthreadhome.h"
#include <QDebug>
#include <QMutex>
#include <iostream>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include <Eigen/Eigen>
#include <vector>
#include <memory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
//ADD

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/PointStamped.h>
#define  PI  3.14159265358979323846
#define  GOHOME  0
#define  J1ADD  1
#define  J1DEC  2
#define  J2ADD  3
#define  J2DEC  4
#define  J3ADD  5
#define  J3DEC  6
#define  J4ADD  7
#define  J4DEC  8
#define  J5ADD  9
#define  J5DEC  10
#define  J6ADD  11
#define  J6DEC  12
#define  JOINTGO  13
#define  POSGO  14
#define  POSXADD  15
#define  POSXDEC  16
#define  POSYADD  17
#define  POSYDEC  18
#define  POSZADD  19
#define  POSZDEC  20
#define  RECORDSIMULATION  21
#define  CHARACTERGO  22
WorkThreadhome::WorkThreadhome()
{
    istimer = false;
    stepsize = 10;
    velscale = 1;
}
void WorkThreadhome::closeThread()
{
    isStop = true;
}

void WorkThreadhome::run()
{
      ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    static const std::string PLANNING_GROUP = "arm";

    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
    // 原始指针经常被用来指代计划组以提高性能。
    const robot_state::JointModelGroup *joint_model_group =
        group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75; // above head of PR2，文字的位置放在了PR2的头上
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link","/moveit_visual_markers"));

    std::vector <double> joint;
    if(istimer == true)
    {
      while(1)
      {
        px=group.getCurrentPose().pose.position.x*1000;
        py=group.getCurrentPose().pose.position.y*1000;
        pz=group.getCurrentPose().pose.position.z*1000-478.055;
        ow=group.getCurrentPose().pose.orientation.w;
        ox=group.getCurrentPose().pose.orientation.x;
        oy=group.getCurrentPose().pose.orientation.y;
        oz=group.getCurrentPose().pose.orientation.z;
        j1=group.getCurrentJointValues().at(0)*180/PI;
        j2=group.getCurrentJointValues().at(1)*180/PI;
        j3=group.getCurrentJointValues().at(2)*180/PI;
        j4=group.getCurrentJointValues().at(3)*180/PI;
        j5=group.getCurrentJointValues().at(4)*180/PI;
        j6=group.getCurrentJointValues().at(5)*180/PI;
        sleep(1);
      }
    }
    else
    {
      switch(threadmode)
      {
          case GOHOME:
                {

                    joint.clear();
                    joint.push_back(0*PI/180);
                    joint.push_back(0*PI/180);
                    joint.push_back(90*PI/180);
                    joint.push_back(0*PI/180);
                    joint.push_back(45*PI/180);
                    joint.push_back(0*PI/180);

                    group.setJointValueTarget(joint);
                    //group.move();

                    group.setMaxVelocityScalingFactor(velscale);
                    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

                    ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");
                    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
                    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);//);
                    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("link_6"), joint_model_group,rvt::LIME_GREEN);
                   //搞了好久 必须定义关节轨迹
                   // visual_tools.publishTrajectoryPoints(my_plan.trajectory_,"link_6");
                    visual_tools.trigger();
                    //visual_tools.prompt("next step");
                    //让机械臂按照规划的轨迹开始运动。
                    if(success)
                        group.execute(my_plan);
                 }
          break;
          case  J1ADD:
                {
                    joint.clear();
                    joint.push_back(group.getCurrentJointValues().at(0)+(stepsize*PI/180));
                    joint.push_back(group.getCurrentJointValues().at(1));
                    joint.push_back(group.getCurrentJointValues().at(2));
                    joint.push_back(group.getCurrentJointValues().at(3));
                    joint.push_back(group.getCurrentJointValues().at(4));
                    joint.push_back(group.getCurrentJointValues().at(5));
                    group.setMaxVelocityScalingFactor(velscale);
                    group.setJointValueTarget(joint);
      //              group.move();
                    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                    ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                    //让机械臂按照规划的轨迹开始运动。
                    if(success)
                        group.execute(my_plan);

                }
          break;
          case  J1DEC:
              {
                  joint.clear();
                  joint.push_back(group.getCurrentJointValues().at(0)-(stepsize*PI/180));
                  joint.push_back(group.getCurrentJointValues().at(1));
                  joint.push_back(group.getCurrentJointValues().at(2));
                  joint.push_back(group.getCurrentJointValues().at(3));
                  joint.push_back(group.getCurrentJointValues().at(4));
                  joint.push_back(group.getCurrentJointValues().at(5));
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
              //              group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);

              }
          break;
          case  J2ADD:
              {
                  joint.clear();
                  joint.push_back(group.getCurrentJointValues().at(0));
                  joint.push_back(group.getCurrentJointValues().at(1)+(stepsize*PI/180));
                  joint.push_back(group.getCurrentJointValues().at(2));
                  joint.push_back(group.getCurrentJointValues().at(3));
                  joint.push_back(group.getCurrentJointValues().at(4));
                  joint.push_back(group.getCurrentJointValues().at(5));
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
              //              group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);

              }
          break;
          case  J2DEC:
              {
                  joint.clear();
                  joint.push_back(group.getCurrentJointValues().at(0));
                  joint.push_back(group.getCurrentJointValues().at(1)-(stepsize*PI/180));
                  joint.push_back(group.getCurrentJointValues().at(2));
                  joint.push_back(group.getCurrentJointValues().at(3));
                  joint.push_back(group.getCurrentJointValues().at(4));
                  joint.push_back(group.getCurrentJointValues().at(5));
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
              //              group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);

              }
          break;
          case  J3ADD:
              {
                  joint.clear();
                  joint.push_back(group.getCurrentJointValues().at(0));
                  joint.push_back(group.getCurrentJointValues().at(1));
                  joint.push_back(group.getCurrentJointValues().at(2)+(stepsize*PI/180));
                  joint.push_back(group.getCurrentJointValues().at(3));
                  joint.push_back(group.getCurrentJointValues().at(4));
                  joint.push_back(group.getCurrentJointValues().at(5));
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
              //              group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);

              }
          break;
          case  J3DEC:
              {
                  joint.clear();
                  joint.push_back(group.getCurrentJointValues().at(0));
                  joint.push_back(group.getCurrentJointValues().at(1));
                  joint.push_back(group.getCurrentJointValues().at(2)-(stepsize*PI/180));
                  joint.push_back(group.getCurrentJointValues().at(3));
                  joint.push_back(group.getCurrentJointValues().at(4));
                  joint.push_back(group.getCurrentJointValues().at(5));
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
              //              group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);

              }
          break;
          case  J4ADD:
              {
                  joint.clear();
                  joint.push_back(group.getCurrentJointValues().at(0));
                  joint.push_back(group.getCurrentJointValues().at(1));
                  joint.push_back(group.getCurrentJointValues().at(2));
                  joint.push_back(group.getCurrentJointValues().at(3)+(stepsize*PI/180));
                  joint.push_back(group.getCurrentJointValues().at(4));
                  joint.push_back(group.getCurrentJointValues().at(5));
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
              //              group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);

              }
          break;
          case  J4DEC:
              {
                  joint.clear();
                  joint.push_back(group.getCurrentJointValues().at(0));
                  joint.push_back(group.getCurrentJointValues().at(1));
                  joint.push_back(group.getCurrentJointValues().at(2));
                  joint.push_back(group.getCurrentJointValues().at(3)-(stepsize*PI/180));
                  joint.push_back(group.getCurrentJointValues().at(4));
                  joint.push_back(group.getCurrentJointValues().at(5));
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
              //              group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);

              }
          break;
          case  J5ADD:
              {
                  joint.clear();
                  joint.push_back(group.getCurrentJointValues().at(0));
                  joint.push_back(group.getCurrentJointValues().at(1));
                  joint.push_back(group.getCurrentJointValues().at(2));
                  joint.push_back(group.getCurrentJointValues().at(3));
                  joint.push_back(group.getCurrentJointValues().at(4)+(stepsize*PI/180));
                  joint.push_back(group.getCurrentJointValues().at(5));
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
              //              group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);

              }
          break;
          case  J5DEC:
              {
                  joint.clear();
                  joint.push_back(group.getCurrentJointValues().at(0));
                  joint.push_back(group.getCurrentJointValues().at(1));
                  joint.push_back(group.getCurrentJointValues().at(2));
                  joint.push_back(group.getCurrentJointValues().at(3));
                  joint.push_back(group.getCurrentJointValues().at(4)-(stepsize*PI/180));
                  joint.push_back(group.getCurrentJointValues().at(5));
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
              //              group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);

              }
          break;
          case  J6ADD:
              {
                  joint.clear();
                  joint.push_back(group.getCurrentJointValues().at(0));
                  joint.push_back(group.getCurrentJointValues().at(1));
                  joint.push_back(group.getCurrentJointValues().at(2));
                  joint.push_back(group.getCurrentJointValues().at(3));
                  joint.push_back(group.getCurrentJointValues().at(4));
                  joint.push_back(group.getCurrentJointValues().at(5)+(stepsize*PI/180));
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
              //              group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);

              }
          break;
          case  J6DEC:
              {
                  joint.clear();
                  joint.push_back(group.getCurrentJointValues().at(0));
                  joint.push_back(group.getCurrentJointValues().at(1));
                  joint.push_back(group.getCurrentJointValues().at(2));
                  joint.push_back(group.getCurrentJointValues().at(3));
                  joint.push_back(group.getCurrentJointValues().at(4));
                  joint.push_back(group.getCurrentJointValues().at(5)-(stepsize*PI/180));
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
              //              group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);


                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);

              }
          break;
          case  JOINTGO:
              {
                  joint.clear();
                  joint.push_back(joint1go*PI/180);
                  joint.push_back(joint2go*PI/180);
                  joint.push_back(joint3go*PI/180);
                  joint.push_back(joint4go*PI/180);
                  joint.push_back(joint5go*PI/180);
                  joint.push_back(joint6go*PI/180);
                  group.setMaxVelocityScalingFactor(velscale);
                  group.setJointValueTarget(joint);
                 // group.move();
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

                  ROS_INFO("Visualizing plan 1 (joint goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);
              }
          break;
          case  POSGO:
              {
                  geometry_msgs::Pose target_pose;
                  target_pose.orientation.w = owgo;
                  target_pose.orientation.x= oxgo;
                  target_pose.orientation.y = oygo;
                  target_pose.orientation.z = ozgo;

                  target_pose.position.x = pxgo/1000;
                  target_pose.position.y = pygo/1000;
                  target_pose.position.z = pzgo/1000;
                  group.setPoseTarget(target_pose);
                  group.setMaxVelocityScalingFactor(velscale);
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

                  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);
              }
          break;
          case  POSXADD:
              { 
                  geometry_msgs::Pose target_pose1;
                  target_pose1=group.getCurrentPose().pose;
                  target_pose1.position.x += stepsize/1000;
                  group.setPoseTarget(target_pose1);
                  group.setMaxVelocityScalingFactor(velscale);
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
                  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");


                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);
              }
          break;
          case  POSXDEC:
              {
                  geometry_msgs::Pose target_pose1;
                  target_pose1=group.getCurrentPose().pose;
                  target_pose1.position.x -= stepsize/1000;
                  group.setPoseTarget(target_pose1);
                  group.setMaxVelocityScalingFactor(velscale);
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

                  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);
              }
          break;
          case  POSYADD:
              {
                  geometry_msgs::Pose target_pose1;
                  target_pose1=group.getCurrentPose().pose;
                  target_pose1.position.y += stepsize/1000;
                  group.setPoseTarget(target_pose1);
                  group.setMaxVelocityScalingFactor(velscale);
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

                  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);
              }
          break;
          case  POSYDEC:
              {
                  geometry_msgs::Pose target_pose1;
                  target_pose1=group.getCurrentPose().pose;
                  target_pose1.position.y -= stepsize/1000;
                  group.setPoseTarget(target_pose1);
                  group.setMaxVelocityScalingFactor(velscale);
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

                  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);
              }
          break;
          case  POSZADD:
              {
                  geometry_msgs::Pose target_pose1;
                  target_pose1=group.getCurrentPose().pose;
                  target_pose1.position.z += stepsize/1000;
                  group.setPoseTarget(target_pose1);
                  group.setMaxVelocityScalingFactor(velscale);
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

                  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);
              }
          break;
          case  POSZDEC:
              {
                  geometry_msgs::Pose target_pose1;
                  target_pose1=group.getCurrentPose().pose;
                  target_pose1.position.z -= stepsize/1000;
                  group.setPoseTarget(target_pose1);
                  group.setMaxVelocityScalingFactor(velscale);
                  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
                  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

                  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

                  //让机械臂按照规划的轨迹开始运动。
                  if(success)
                      group.execute(my_plan);
              }
          break;
          case  RECORDSIMULATION:
              {
                  geometry_msgs::Pose target_pose;
                  geometry_msgs::Pose now_pose;
                  group.setStartState(*group.getCurrentState());
                  //设置运动路径
                  std::vector<geometry_msgs::Pose> waypoints;
                  now_pose=group.getCurrentPose().pose;
                  for(int i=0;i<recordpx.size();i++)
                  {
                    target_pose.orientation.w=recordow[i];
                    target_pose.orientation.x=recordox[i];
                    target_pose.orientation.y=recordoy[i];
                    target_pose.orientation.z=recordoz[i];
                    target_pose.position.x= recordpx[i]/1000;
                    target_pose.position.y= recordpy[i]/1000;
                    target_pose.position.z= recordpz[i]/1000;
                    waypoints.push_back(target_pose);
                  }
                  moveit_msgs::RobotTrajectory trajectory;
                  double fraction = group.computeCartesianPath(waypoints,
                                                                   0.01,  // eef_step
                                                                   0.0,   // jump_threshold
                                                                   trajectory
                                                                   );
                  ROS_INFO("Visualizing plan  (cartesian path) (%.2f%% acheived)",
                              fraction * 100.0);
                  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm");
                  // Second get a RobotTrajectory from trajectory
                  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
                  // Thrid create a IterativeParabolicTimeParameterization object
                  trajectory_processing::IterativeParabolicTimeParameterization iptp;
                  // Fourth compute computeTimeStamps
                  bool ItSuccess = iptp.computeTimeStamps(rt,0.01,0.01);
                  ROS_INFO("Computed time stamp %s",ItSuccess?"SUCCEDED":"FAILED");
                  // Get RobotTrajectory_msg from RobotTrajectory
                  rt.getRobotTrajectoryMsg(trajectory);

                  moveit::planning_interface::MoveGroupInterface::Plan plan;
                  plan.trajectory_ = trajectory;
                  group.execute(plan);
              }
          break;
          case  CHARACTERGO:
              {
                  geometry_msgs::Pose target_pose;
                  geometry_msgs::Pose now_pose;
                  group.setStartState(*group.getCurrentState());
                  //设置运动路径
                  std::vector<geometry_msgs::Pose> waypoints;
                  now_pose=group.getCurrentPose().pose;
                  for(int i=0;i<storex.size();i++)
                  {
                    target_pose.orientation.w=now_pose.orientation.w;
                    target_pose.orientation.x=now_pose.orientation.x;
                    target_pose.orientation.y=now_pose.orientation.y;
                    target_pose.orientation.z=now_pose.orientation.z;
                    target_pose.position.x= now_pose.position.x + storex[i]/1000;
                    target_pose.position.y= now_pose.position.y - storey[i]/1000;
                    target_pose.position.z= now_pose.position.z + storez[i]/1000;
                    waypoints.push_back(target_pose);
                  }
                  moveit_msgs::RobotTrajectory trajectory;
                  double fraction = group.computeCartesianPath(waypoints,
                                                                   0.01,  // eef_step
                                                                   0.0,   // jump_threshold
                                                                   trajectory
                                                                   );
                  ROS_INFO("Visualizing plan  (cartesian path) (%.2f%% acheived)",
                              fraction * 100.0);
                  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm");
                  // Second get a RobotTrajectory from trajectory
                  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
                  // Thrid create a IterativeParabolicTimeParameterization object
                  trajectory_processing::IterativeParabolicTimeParameterization iptp;
                  // Fourth compute computeTimeStamps
                  bool ItSuccess = iptp.computeTimeStamps(rt,0.01,0.01);
                  ROS_INFO("Computed time stamp %s",ItSuccess?"SUCCEDED":"FAILED");
                  // Get RobotTrajectory_msg from RobotTrajectory
                  rt.getRobotTrajectoryMsg(trajectory);

                  moveit::planning_interface::MoveGroupInterface::Plan plan;
                  plan.trajectory_ = trajectory;

                  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
                  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);//);
                  visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group->getLinkModel("link_6"), joint_model_group,rvt::LIME_GREEN);
                 //搞了好久 必须定义关节轨迹
                 // visual_tools.publishTrajectoryPoints(my_plan.trajectory_,"link_6");
                  visual_tools.trigger();
                  //visual_tools.prompt("next step");
                  group.execute(plan);
              }
          break;

      }
    }


}
