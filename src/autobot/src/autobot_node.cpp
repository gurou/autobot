/*******************************************************************************
 *
 * Copyright (c) 2014
 * All rights reserved.
 *
 * Hochschule Rosenheim
 * University of Applied Sciences
 * Electrical Engineering & Information Technology Department
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File Name     : autobot_node.cpp
 * Created on    : Jun 16, 2014
 * Author        : Guru, Julian, Andreas
 * Supervised by : Prof. Dr. Wolfgang Schittenhelm
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This software is published under MIT License.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Hochschule Rosenheim nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the MIT License as published by the Open Source
 * Initiative.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the MIT
 * License for more details.
 *
 * You should have received a copy of the MIT License along with this program.
 *
 ******************************************************************************/
#include <ros/ros.h>
#include <sstream>
#include <iostream>
using namespace std;

#include <boost/lexical_cast.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>

// Global variables
float xCoordinate = 0.0;        // Variable to hold the Goal's x position
bool goalEnded = false;
bool goalThreadStarted = false;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void spinThread()
{
  ros::spin();
}

void ConstructAndSendGoal(MoveBaseClient *ac)
{
  while (!ac->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server");
  }

  goalThreadStarted = true;

  // Construct goal
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = xCoordinate;
  goal.target_pose.pose.position.y = 0;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal and wait for the goal to complete or abort
  ROS_INFO("Sending goal");
  ac->sendGoal(goal);
  ac->waitForResult();

  // Check for the status of the goal
  if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("You have arrived to the goal position");
  else
  {
    ROS_INFO("The base failed for some reason");
  }

  // Set the goalEnded flag to true. This flag is set irrespective of whether
  // the goal succeeded or failed.
  goalEnded = true;

  cout << "Press any key to exit";
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    cout << "Incorrect arguments" << endl;
    return 0;
  }
  else
  {
    try
    {
      xCoordinate = boost::lexical_cast<float>(argv[1]);
    }
    catch (boost::bad_lexical_cast const&)
    {
      cout << "Error: input string was not valid" << std::endl;
      cout << "Cannot send goal to YouBot";
      return 0;
    }
  }

  ros::init(argc, argv, "navigation_goals");

  // Spin the ROS thread
  boost::thread spin_thread(&spinThread);

  // Construct the MoveBaseClient Object
  MoveBaseClient ac("move_base", true);

  // Spin a thread to construct the goal and send it to move_base
  boost::thread sendGoal_thread(&ConstructAndSendGoal, &ac);

  // Monitor for key input to cancel the goal
  while(goalEnded != true)
  {
    int ch;
    ch = getchar();
    if(goalEnded == true)
    {
      cout << "Goal Ended. Exiting program." << endl;
    }
    else
    {
      if ((ch == 'c') || (ch == 'C'))
      {
        ac.cancelAllGoals();
      }
      else
      {
        cout << "YouBot is still pursuing the goal. Press 'c' or 'C' to "
                "cancel it." << endl;
      }
    }
  }

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  // Exit
  return 0;
}
